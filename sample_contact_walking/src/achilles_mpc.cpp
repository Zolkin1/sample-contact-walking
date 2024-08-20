#include <vector>

#include "obelisk_node.h"
#include "obelisk_ros_utils.h"

// TODO: remove
#include "obelisk_estimator.h"

#include "sample_contact_walking/achilles_mpc.h"

// TODO:
//  - Verify that if I start on the trajectory that the MPC is well behaved
//  - Allow user to set the initial condition in the yaml
//  - Check for paths first relative to $SAMPLE_WALKING_ROOT then as a global path
//  - Add ROS diagonstic messages: https://docs.foxglove.dev/docs/visualization/panels/diagnostics#diagnosticarray
//  - Try plotting just the first MPC solve and see what the trajectory is. Why does it swing its leg really far out
//  - Try tuning the weights to make the leg not swing so far to the side. Maybe up some configuration weights
//  - Clean up code

namespace achilles
{
    AchillesController::AchillesController(const std::string& name) 
    : obelisk::ObeliskController<obelisk_control_msgs::msg::PDFeedForward, obelisk_estimator_msgs::msg::EstimatedState>(name), 
        recieved_first_state_(false), first_mpc_computed_(false), ctrl_state_(NoOutput), traj_start_time_(0) {
        // TODO: Remove
        // std::this_thread::sleep_for (std::chrono::seconds(10));

        // For now model the feet as point contacts
        contact_state_.contacts.emplace("left_ankle_pitch", torc::models::Contact(torc::models::PointContact, false));
        contact_state_.contacts.emplace("right_ankle_pitch", torc::models::Contact(torc::models::PointContact, false));

        // TODO: Removed because it caused serious jitter on the MPC computations, so running in a seperate thread.
        // this->RegisterObkTimer("timer_mpc_setting", "mpc_timer", std::bind(&AchillesController::ComputeMpc, this));
        this->declare_parameter<double>("mpc_loop_period_sec", 0.01);
        this->declare_parameter<long>("max_mpc_solves", -1);

        // --- Debug Publisher and Timer --- //
        this->RegisterObkTimer("state_viz_timer_setting", "state_viz_timer", std::bind(&AchillesController::PublishTrajStateViz, this));
        this->RegisterObkPublisher<obelisk_estimator_msgs::msg::EstimatedState>("state_viz_pub_setting", "state_viz_pub");

        this->declare_parameter<std::string>("viz_pub_setting");
        // RCLCPP_ERROR_STREAM(this->get_logger(), "viz pub settings: " << this->get_parameter("viz_pub_setting").as_string());
        auto viz_pub = this->CreatePublisherFromConfigStr<visualization_msgs::msg::MarkerArray>(this->get_parameter("viz_pub_setting").as_string());
        this->publishers_["viz_pub"] = std::make_shared<obelisk::internal::ObeliskPublisher<visualization_msgs::msg::MarkerArray>>(viz_pub);

        //  Update model
        this->declare_parameter<std::string>("urdf_path", "");
        std::filesystem::path urdf_path(this->get_parameter("urdf_path").as_string());

        // Create model
        std::string model_name = name + "_model";
        model_ = std::make_unique<torc::models::FullOrderRigidBody>(model_name, urdf_path);

        // Create and configure MPC
        mpc_ = std::make_unique<torc::mpc::FullOrderMpc>("achilles_mpc_obk", this->get_parameter("params_path").as_string(), urdf_path);
        RCLCPP_INFO_STREAM(this->get_logger(), "MPC Created...");

        mpc_->Configure();
        RCLCPP_INFO_STREAM(this->get_logger(), "MPC Configured...");

        // Make the contact schedule
        this->declare_parameter<double>("swing_time", 0.3);
        this->declare_parameter<double>("first_swing_time", 1);
        this->declare_parameter<double>("double_stance_time", 0.0);
        this->declare_parameter<bool>("right_foot_first", true);

        this->get_parameter("swing_time", swing_time_);
        this->get_parameter("first_swing_time", first_swing_time_);
        this->get_parameter("double_stance_time", double_stance_time_);
        this->get_parameter("right_foot_first", right_foot_first_);


        contact_schedule_.SetFrames(mpc_->GetContactFrames());

        AddPeriodicContacts();

        if (right_foot_first_) {
            contact_schedule_.InsertContact("foot_front_left", 0, first_swing_time_ + swing_time_);
            contact_schedule_.InsertContact("foot_rear_left", 0, first_swing_time_ + swing_time_);
            contact_schedule_.InsertContact("foot_front_right", 0, first_swing_time_);
            contact_schedule_.InsertContact("foot_rear_right", 0, first_swing_time_);
        } else {
            contact_schedule_.InsertContact("foot_front_left", 0, first_swing_time_);
            contact_schedule_.InsertContact("foot_rear_left", 0, first_swing_time_);
            contact_schedule_.InsertContact("foot_front_right", 0, first_swing_time_ + swing_time_);
            contact_schedule_.InsertContact("foot_rear_right", 0, first_swing_time_ + swing_time_);
        }

        this->declare_parameter<double>("default_swing_height", 0.1);
        this->declare_parameter<double>("default_stand_foot_height", 0.0);
        this->declare_parameter<double>("apex_time", 0.5);
        mpc_->UpdateContactScheduleAndSwingTraj(contact_schedule_,
            this->get_parameter("default_swing_height").as_double(),
            this->get_parameter("default_stand_foot_height").as_double(),
            this->get_parameter("apex_time").as_double());

        // TODO: Check the quaternion reference, it might be wrong
        // Setup q and v targets
        q_target_.resize(model_->GetConfigDim());
        q_target_ << 0., 0, 0.9,    // position
                    0, 0, 0, 1,     // quaternion
                    0, 0, -0.26,    // L hips joints
                    0.65, -0.43,    // L knee, ankle
                    0, 0, 0, 0,     // L shoulders and elbow
                    0, 0, -0.26,    // R hip joints
                    0.65, -0.43,    // R knee ankle
                    0, 0, 0, 0;     // R shoulders and elbow

        v_target_.resize(model_->GetVelDim());
        v_target_ << 0, 0, 0,
                    0, 0, 0,
                    0, 0, 0,
                    0, 0, 0, 
                    0, 0, 0,
                    0, 0, 0,
                    0, 0, 0,
                    0, 0, 0;
        mpc_->SetConstantConfigTarget(q_target_);
        mpc_->SetConstantVelTarget(v_target_);

        // Set initial conditions
        // TODO: Do this from yaml
        q_ic_ = q_target_;
        q_ic_(0) = 0;
        v_ic_ = v_target_;
        v_ic_.setZero();

        // Setup swing trajectories

        // Create default trajectory
        traj_out_.UpdateSizes(model_->GetConfigDim(), model_->GetVelDim(), model_->GetNumInputs(), mpc_->GetContactFrames(), mpc_->GetNumNodes());
        traj_out_.SetDefault(q_target_);
        traj_out_.SetDtVector(mpc_->GetDtVector());


        // Warm start trajectory forces
        double time = 0;
        for (int i = 0; i < traj_out_.GetNumNodes(); i++) {
            int num_contacts = 0;
            for (const auto& frame : mpc_->GetContactFrames()) {
                if (contact_schedule_.InContact(frame, time)) {
                    num_contacts++;
                }
            }

            for (const auto& frame : mpc_->GetContactFrames()) {
                if (contact_schedule_.InContact(frame, time)) {
                    traj_out_.SetForce(i, frame, {0, 0, 9.81*model_->GetMass()/num_contacts});
                }
            }

            time += traj_out_.GetDtVec()[i];
        }

        mpc_->SetWarmStartTrajectory(traj_out_);
        traj_mpc_ = traj_out_;
        RCLCPP_INFO_STREAM(this->get_logger(), "Warm start trajectory created...");

        // TODO: Verify the initial conditions
        mpc_->ComputeNLP(q_ic_, v_ic_, traj_mpc_);
        traj_out_ = traj_mpc_;
        traj_start_time_ = -1;

        // Compute an initial MPC at the initial condition

        // TODO: Put back
        // Go to initial condition
        TransitionState(SeekInitialCond);
        // TransitionState(Mpc);

        // Visualization information
        this->declare_parameter<std::vector<std::string>>("viz_frames", {""});
        viz_frames_ = this->get_parameter("viz_frames").as_string_array();
        std::erase_if(viz_frames_, [this](std::string& frame)
            {
                if (this->model_->GetFrameIdx(frame) == -1) {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Viz frame: " << frame << " is not found in the URDF! Ignoring!");
                    return true;
                }
                RCLCPP_INFO_STREAM(this->get_logger(), "Viz frame: " << frame << " is being used.");
                return false;
            });

        this->declare_parameter<double>("viz_force_scale", 1.0);
        this->declare_parameter<bool>("viz_forces", false);
        this->declare_parameter<std::vector<std::string>>("force_frames", {""});

        viz_forces_ = this->get_parameter("viz_forces").as_bool();
        scale_forces_ = this->get_parameter("viz_force_scale").as_double();
        this->get_parameter("force_frames", force_frames_);

        // TODO: Spin up MPC thread
        mpc_thread_ = std::thread(&AchillesController::MpcThread, this);
    }

    void AchillesController::UpdateXHat(const obelisk_estimator_msgs::msg::EstimatedState& msg) {
        // Get the mutex to protect the states
        std::lock_guard<std::mutex> lock(est_state_mut_);
        
        q_.resize(msg.q_base.size() + msg.q_joints.size());
        v_.resize(msg.v_base.size() + msg.v_joints.size());


        // Configuration
        for (size_t i = 0; i < msg.q_base.size(); i++) {
            q_(i) = msg.q_base.at(i);
        }

        for (size_t i = 0; i < msg.q_joints.size(); i++) {
            q_(i + msg.q_base.size()) = msg.q_joints.at(i);
        }

        // Velocity
        for (size_t i = 0; i < msg.v_base.size(); i++) {
            v_(i) = msg.v_base.at(i);
        }

        for (size_t i = 0; i < msg.v_joints.size(); i++) {
            v_(i + msg.v_base.size()) = msg.v_joints.at(i);
        }

        // TODO: Update contact in MPC with SetContactSchedule
        // TODO: Update contact states properly
        contact_state_.contacts.at("left_ankle_pitch").state = true;
        contact_state_.contacts.at("right_ankle_pitch").state = true;

        if (q_.size() != model_->GetConfigDim()) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "received q does not match the size of the model");
        }

        if (v_.size() != model_->GetVelDim()) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "received v does not match the size of the model");
        }

        if (!recieved_first_state_ && q_.size() == model_->GetConfigDim() && v_.size() == model_->GetVelDim()) {
            recieved_first_state_ = true;
        }
    }

    // Removed because the timing was not consistent
    // void AchillesController::ComputeMpc() {
    //     if (recieved_first_state_) {
    //         RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Computing first trajectory.");
    //         vectorx_t q, v;
    //         {
    //             // Get the mutex to protect the states
    //             std::lock_guard<std::mutex> lock(est_state_mut_);

    //             // Create current state
    //             q = q_;
    //             v = v_;
    //         }

    //         // If in NLP mode, then pause the timer and compute the NLP
    //         // Then restart the timer after the NLP is solved.
    //         // Change state to normal MPC
    //         // This only works if the robot can hold its position for the enough time for the computation

    //         // TODO: Consider putting back
    //         // Get the current time
    //         // double time = this->get_clock()->now().seconds();

    //         // Shift the contact schedule
    //         contact_schedule_.ShiftContacts(-traj_mpc_.GetDtVec()[0]);    // TODO: Do I need a mutex on this later?
    //         mpc_->UpdateContactScheduleAndSwingTraj(contact_schedule_,
    //             this->get_parameter("default_swing_height").as_double(),
    //             this->get_parameter("default_stand_foot_height").as_double(), 0.5);

    //         if (mpc_comps_ < 20) {
    //             // std::cout << "mpc compute #" << mpc_comps_ << std::endl;
    //             // std::cout << "q: " << q.transpose() << std::endl;
    //             // std::cout << "v: " << v.transpose() << std::endl;

    //             mpc_->Compute(q, v, traj_mpc_);
    //             mpc_comps_++;
    //             {
    //                 // Get the traj mutex to protect it
    //                 std::lock_guard<std::mutex> lock(traj_out_mut_);
    //                 traj_out_ = traj_mpc_;

    //                 // Assign time time too
    //                 double time = this->get_clock()->now().seconds();
    //                 traj_start_time_ = time;
    //             }
    //             // RCLCPP_INFO_STREAM(this->get_logger(), "MPC Computation Completed!");
    //             if (GetState() != Mpc) {
    //                 // TODO: Put back
    //                 TransitionState(Mpc);
    //             }
    //         } else {
    //             static bool printed = false;
    //             if (!printed) {
    //                 mpc_->PrintStatistics();
    //                 mpc_->PrintContactSchedule();
    //                 printed = true;
    //             }
    //         }

    //         // TODO: If this is slow, then I need to move it
    //         PublishTrajViz(traj_mpc_, viz_frames_);

    //     }
    // }

    // Running the MPC in its own the thread seems to make the timing more consistent and overall faster
    // If I still need more, I can try to adjust the thread prio
    // Experimentally, note that the faster I run it, the more consistent (and faster, up to a limit) it is
    void AchillesController::MpcThread() {
        const long mpc_loop_rate_ns = this->get_parameter("mpc_loop_period_sec").as_double()*1e9;
        RCLCPP_INFO_STREAM(this->get_logger(), "MPC loop period set to: " << mpc_loop_rate_ns << "ns.");

        const long max_mpc_solves = this->get_parameter("max_mpc_solves").as_int();
        
        static bool first_loop = true;
        auto prev_time = this->now();

        while (true) {
            // Start the timer
            auto start_time = this->now();

            if (recieved_first_state_) {
                RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Computing first trajectory.");

                if (first_loop) {
                    prev_time = this->now();
                    first_loop = false;
                }

                vectorx_t q, v;
                {
                    // Get the mutex to protect the states
                    std::lock_guard<std::mutex> lock(est_state_mut_);

                    // Create current state
                    q = q_;
                    v = v_;
                }

                // If in NLP mode, then pause the timer and compute the NLP
                // Then restart the timer after the NLP is solved.
                // Change state to normal MPC
                // This only works if the robot can hold its position for the enough time for the computation

                // Shift the contact schedule
                auto current_time = this->now();
                contact_schedule_.ShiftContacts(-(current_time - prev_time).nanoseconds()/1e9);    // TODO: Do I need a mutex on this later?
                prev_time = this->now();
                mpc_->UpdateContactScheduleAndSwingTraj(contact_schedule_,
                    this->get_parameter("default_swing_height").as_double(),
                    this->get_parameter("default_stand_foot_height").as_double(),
                    this->get_parameter("apex_time").as_double());

                AddPeriodicContacts();


                if (max_mpc_solves < 0 || mpc_->GetTotalSolves() < max_mpc_solves) {

                    mpc_->Compute(q, v, traj_mpc_);
                    {
                        // Get the traj mutex to protect it
                        std::lock_guard<std::mutex> lock(traj_out_mut_);
                        traj_out_ = traj_mpc_;

                        // Assign time time too
                        double time = this->get_clock()->now().seconds();
                        traj_start_time_ = time;
                    }
                    if (GetState() != Mpc) {
                        TransitionState(Mpc);
                    }
                } else {
                    static bool printed = false;
                    if (!printed) {
                        // mpc_->PrintStatistics();
                        // mpc_->PrintContactSchedule();
                        mpc_->PrintAggregateStats();
                        printed = true;
                    }
                }

                // TODO: If this is slow, then I need to move it
                PublishTrajViz(traj_mpc_, viz_frames_);
            }

            // Stop timer
            auto stop_time = this->now(); 

            // Compute difference
            const long time_left = mpc_loop_rate_ns - (stop_time - start_time).nanoseconds();
            if (time_left > 0) {
                std::this_thread::sleep_for(std::chrono::nanoseconds(time_left));
            } else {
                RCLCPP_WARN_STREAM(this->get_logger(), "MPC computation took longer than loop rate allowed for. " << std::abs(time_left)*1e-6 << "ms over time.");
            }
        }
    }

    obelisk_control_msgs::msg::PDFeedForward AchillesController::ComputeControl() {
        // This callback does not need any access to the state as it outputs PD
        //  setpoints from the trajectory.

        obelisk_control_msgs::msg::PDFeedForward msg;
        if (GetState() != NoOutput) {
            RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Publishing first control.");
            
            vectorx_t q, v, tau;

            if (GetState() == Mpc) {
                // Get the traj mutex to protect it
                {
                    std::lock_guard<std::mutex> lock(traj_out_mut_);

                    if (traj_start_time_ < 0) {
                        traj_start_time_ = this->get_clock()->now().seconds();
                    }

                    double time = this->get_clock()->now().seconds();
                    // TODO: Do I need to use nanoseconds?
                    double time_into_traj = time - traj_start_time_;
                    // RCLCPP_INFO_STREAM(this->get_logger(), "Time into traj: " << time_into_traj);

                    traj_out_.GetConfigInterp(time_into_traj, q);
                    traj_out_.GetVelocityInterp(time_into_traj, v);
                    traj_out_.GetTorqueInterp(time_into_traj, tau);
                
                    // --- TODO: Remove! -- This is a safety in case the interpolation time is too large
                    if (q.size() == 0) {
                        // TODO: remove!
                        // throw std::runtime_error("Ending controller!");
                        q = traj_out_.GetConfiguration(traj_out_.GetNumNodes()-1);
                    }

                    if (v.size() == 0) {
                        v = traj_out_.GetVelocity(traj_out_.GetNumNodes()-1);
                    }

                    if (tau.size() == 0) {
                        tau = traj_out_.GetTau(traj_out_.GetNumNodes()-1);
                    }
                    // ---
                }
            } else if (ctrl_state_ == SeekInitialCond) {
                q = q_ic_;
                v = v_ic_;
                tau = vectorx_t::Zero(model_->GetNumInputs());
            }

            // Make the message
            vectorx_t u_mujoco = ConvertControlToMujocoU(q.tail(model_->GetNumInputs()),
                v.tail(model_->GetNumInputs()), tau);
            ConvertEigenToStd(u_mujoco, msg.u_mujoco);
            ConvertEigenToStd(q.tail(model_->GetNumInputs()), msg.pos_target);
            ConvertEigenToStd(v.tail(model_->GetNumInputs()), msg.vel_target);
            ConvertEigenToStd(tau, msg.feed_forward);
            
            if (msg.u_mujoco.size() != 54) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Message's u_mujoco is incorrectly sized. Size: " << msg.u_mujoco.size());
            }

            msg.header.stamp = this->now();

            this->GetPublisher<obelisk_control_msgs::msg::PDFeedForward>(this->ctrl_key_)->publish(msg);
            
        }

        return msg;
    }

    void AchillesController::ConvertEigenToStd(const vectorx_t& eig_vec, std::vector<double>& std_vec) {
        std_vec.clear();
        for (int i = 0; i < eig_vec.size(); i++) {
            std_vec.emplace_back(eig_vec(i));
        }
    }

    vectorx_t AchillesController::ConvertControlToMujocoU(const vectorx_t& pos_target, const vectorx_t& vel_target, const vectorx_t& feed_forward) {
        vectorx_t u(pos_target.size() + vel_target.size() + feed_forward.size());
        u << pos_target, vel_target, feed_forward;
        return u;
    }

    void AchillesController::TransitionState(const ControllerState& new_state) {
        std::lock_guard<std::mutex> lock(ctrl_state_mut_);

        ctrl_state_ = new_state;
        std::string new_state_str;

        switch (new_state) {
        case SeekInitialCond:
            new_state_str = "Seek Initial Condition";
            break;
        case Mpc:
            new_state_str = "MPC";
        case NoOutput:
            new_state_str = "No Output";
        default:
            break;
        }

        RCLCPP_INFO_STREAM(this->get_logger(), "Transitioning to state: " << new_state_str);
    }

    AchillesController::ControllerState AchillesController::GetState() {
        std::lock_guard<std::mutex> lock(ctrl_state_mut_);
        return ctrl_state_;
    }

    void AchillesController::PublishTrajViz(const torc::mpc::Trajectory& traj, const std::vector<std::string>& viz_frames) {
        // TODO: Visualize forces
        // Compute FK for each frame and add the point to a marker message
        visualization_msgs::msg::MarkerArray msg;
        msg.markers.resize(viz_frames.size());
        for (int i = 0; i < viz_frames.size(); i++) {
            msg.markers[i].type = visualization_msgs::msg::Marker::LINE_STRIP;
            msg.markers[i].header.frame_id = "world";
            msg.markers[i].header.stamp = this->now();
            msg.markers[i].ns = "mpc_traj_viz";
            msg.markers[i].id = i;
            msg.markers[i].action = visualization_msgs::msg::Marker::MODIFY;

            msg.markers[i].scale.x = 0.01; // Width of the line segments
        }


        for (int node = 0; node < traj.GetNumNodes(); node++) {
            model_->FirstOrderFK(traj.GetConfiguration(node));

            for (int i = 0; i < viz_frames.size(); i++) {
                vector3_t frame_pos = model_->GetFrameState(viz_frames[i]).placement.translation();
                geometry_msgs::msg::Point point;
                point.x = frame_pos(0);
                point.y = frame_pos(1);
                point.z = frame_pos(2);

                msg.markers[i].points.emplace_back(point);

                // *** Note *** The color is according to the node number, not necessarily the dt
                // Color according to node
                float num_nodes = traj.GetNumNodes();
                float green_part = (num_nodes - node)/num_nodes;
                std_msgs::msg::ColorRGBA color;
                color.r = 1.0 - green_part;
                color.g = green_part;
                color.b = 0;
                color.a = 1;
                msg.markers[i].colors.push_back(color);
            }
        }

        this->GetPublisher<visualization_msgs::msg::MarkerArray>("viz_pub")->publish(msg);

        // Publish force arrows
        if (viz_forces_ && traj_start_time_ >= 0) {
            visualization_msgs::msg::MarkerArray force_msg;
            force_msg.markers.resize(force_frames_.size());
            model_->FirstOrderFK(traj.GetConfiguration(0));
            for (int i = 0; i < force_frames_.size(); i++) {
                force_msg.markers[i].type = visualization_msgs::msg::Marker::LINE_STRIP;
                force_msg.markers[i].header.frame_id = "world";
                force_msg.markers[i].header.stamp = this->now();
                force_msg.markers[i].ns = "mpc_force_viz";
                force_msg.markers[i].id = i;
                force_msg.markers[i].action = visualization_msgs::msg::Marker::MODIFY;

                force_msg.markers[i].scale.x = 0.01; // Width of the arrows

                vector3_t frame_pos = model_->GetFrameState(force_frames_[i]).placement.translation();
                geometry_msgs::msg::Point start_point;
                start_point.x = frame_pos(0);
                start_point.y = frame_pos(1);
                start_point.z = frame_pos(2);

                force_msg.markers[i].points.emplace_back(start_point);

                vector3_t force_interp;
                {
                    double time = this->get_clock()->now().seconds();
                    double time_into_traj = time - traj_start_time_;
                    std::lock_guard<std::mutex> lock(traj_out_mut_);
                    traj_out_.GetForceInterp(time_into_traj, force_frames_[i], force_interp);
                }


                geometry_msgs::msg::Point end_point;
                // TODO: Interpolate the forces
                end_point.x = start_point.x + force_interp(0)*scale_forces_;
                end_point.y = start_point.y + force_interp(1)*scale_forces_;
                end_point.z = start_point.z + force_interp(2)*scale_forces_;

                force_msg.markers[i].points.emplace_back(end_point);

                // *** Note *** The color is according to the node number, not necessarily the dt
                // Color according to node
                std_msgs::msg::ColorRGBA color;
                color.r = 0;
                color.g = 0;
                color.b = 1;
                color.a = 1;
                force_msg.markers[i].colors.push_back(color);
                force_msg.markers[i].colors.push_back(color);
            }

            this->GetPublisher<visualization_msgs::msg::MarkerArray>("viz_pub")->publish(force_msg);
        }
    }

    void AchillesController::PublishTrajStateViz() {
        // std::lock_guard<std::mutex> lock(traj_out_mut_);

        // if (traj_start_time_ < 0) {
        //     traj_start_time_ = this->get_clock()->now().seconds();
        // }

        // double time = this->get_clock()->now().seconds();
        // // TODO: Do I need to use nanoseconds?
        // double time_into_traj = time - traj_start_time_;
        // // double time_into_traj = 0;

        // // RCLCPP_INFO_STREAM(this->get_logger(), "Time into traj: " << time_into_traj);
        // vectorx_t q;
        // vectorx_t v;
        // traj_out_.GetConfigInterp(time_into_traj, q);
        // traj_out_.GetVelocityInterp(time_into_traj, v);

        // // traj_out_.GetConfigInterp(0.01, q);
        // obelisk_estimator_msgs::msg::EstimatedState msg;
        // msg.base_link_name = "torso";
        // vectorx_t q_head = q.head<FLOATING_POS_SIZE>();
        // vectorx_t q_tail = q.tail(model_->GetNumInputs());
        // msg.q_base = torc::utils::EigenToStdVector(q_head);
        // msg.q_joints = torc::utils::EigenToStdVector(q_tail);

        // msg.joint_names.resize(q_tail.size());
        // msg.joint_names[0] = "left_hip_yaw_joint";
        // msg.joint_names[1] = "left_hip_roll_joint";
        // msg.joint_names[2] = "left_hip_pitch_joint";
        // msg.joint_names[3] = "left_knee_pitch_joint";
        // msg.joint_names[4] = "left_ankle_pitch_joint";
        // msg.joint_names[5] = "left_shoulder_pitch_joint";
        // msg.joint_names[6] = "left_shoulder_roll_joint";
        // msg.joint_names[7] = "left_shoulder_yaw_joint";
        // msg.joint_names[8] = "left_elbow_pitch_joint";
        // msg.joint_names[9] = "right_hip_yaw_joint";
        // msg.joint_names[10] = "right_hip_roll_joint";
        // msg.joint_names[11] = "right_hip_pitch_joint";
        // msg.joint_names[12] = "right_knee_pitch_joint";
        // msg.joint_names[13] = "right_ankle_pitch_joint";
        // msg.joint_names[14] = "right_shoulder_pitch_joint";
        // msg.joint_names[15] = "right_shoulder_roll_joint";
        // msg.joint_names[16] = "right_shoulder_yaw_joint";
        // msg.joint_names[17] = "right_elbow_pitch_joint";

        // // traj_out_.GetVelocityInterp(0.01, v);
        // vectorx_t v_head = v.head<FLOATING_VEL_SIZE>();
        // vectorx_t v_tail = v.tail(model_->GetNumInputs());

        // // vectorx_t temp = vectorx_t::Zero(6);
        // msg.v_base = torc::utils::EigenToStdVector(v_head);
        // msg.v_joints = torc::utils::EigenToStdVector(v_tail);

        // msg.header.stamp = this->now();

        // this->GetPublisher<obelisk_estimator_msgs::msg::EstimatedState>("state_viz_pub")->publish(msg);
    }

    void AchillesController::AddPeriodicContacts() {
        // For each contact determine last contact
        // If the last contact is within 1 second, then add another contact swing_time_ seconds later that lasts for swing_time_ + double_stance_time_
        for (const auto& frame : mpc_->GetContactFrames()) {
            const double last_contact_time = contact_schedule_.GetLastContactTime(frame);
            if (last_contact_time < 1) {
                contact_schedule_.InsertContact(frame, last_contact_time + swing_time_,  last_contact_time + 2*swing_time_ + double_stance_time_);
            }
        } 
    }

} // namespace achilles


int main(int argc, char* argv[]) {
    obelisk::utils::SpinObelisk<achilles::AchillesController, rclcpp::executors::MultiThreadedExecutor>(
        argc, argv, "whole_body_controller");
}
