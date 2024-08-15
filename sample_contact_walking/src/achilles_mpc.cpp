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
//  - Update ObeliskNode, line 788 (is msg obelsik to be capical "T")
//  - Update ObeliskNode sensor data to have a stamped header


namespace achilles
{
    AchillesController::AchillesController(const std::string& name) 
    : obelisk::ObeliskController<obelisk_control_msgs::msg::PDFeedForward, obelisk_estimator_msgs::msg::EstimatedState>(name), 
        recieved_first_state_(false), first_mpc_computed_(false), ctrl_state_(NoOutput), traj_start_time_(0), mpc_comps_(0) {
        // For now model the feet as point contacts
        contact_state_.contacts.emplace("left_ankle_pitch", torc::models::Contact(torc::models::PointContact, false));
        contact_state_.contacts.emplace("right_ankle_pitch", torc::models::Contact(torc::models::PointContact, false));

        this->RegisterObkTimer("timer_mpc_setting", "mpc_timer", std::bind(&AchillesController::ComputeMpc, this));

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
        contact_schedule_.SetFrames(mpc_->GetContactFrames());
        // contact_schedule_.InsertContact("right_foot", 0, 0.3);
        // contact_schedule_.InsertContact("right_foot", 0.6, 1);
        // contact_schedule_.InsertContact("left_foot", 0.3, 0.7);
        // this->contact_schedule_.InsertContact("right_foot", 1.3, 1000);
        // this->contact_schedule_.InsertContact("left_foot", 1, 1000);

        contact_schedule_.InsertContact("foot_front_left", 0, 1000);
        contact_schedule_.InsertContact("foot_rear_left", 0, 1000);
        contact_schedule_.InsertContact("foot_front_right", 0, 1000);
        contact_schedule_.InsertContact("foot_rear_right", 0, 1000);

        // contact_schedule_.InsertContact("foot_front_right", 1, 1000);
        // contact_schedule_.InsertContact("foot_rear_right", 1, 1000);

        // contact_schedule_.InsertContact("foot_front_right", 0, 0.7);
        // contact_schedule_.InsertContact("foot_rear_right", 0, 0.7);

        // contact_schedule_.InsertContact("foot_front_left", 0, 0.3);
        // contact_schedule_.InsertContact("foot_rear_left", 0, 0.3);

        // contact_schedule_.InsertContact("foot_front_left", 0.7, 1000);
        // contact_schedule_.InsertContact("foot_rear_left", 0.7, 1000);

        this->declare_parameter<double>("default_swing_height", 0.2);
        mpc_->UpdateContactScheduleAndSwingTraj(contact_schedule_,
            this->get_parameter("default_swing_height").as_double(), 0.00, 0.5);

        // Setup q and v targets
        q_target_.resize(model_->GetConfigDim());
        q_target_ << 0., 0, 0.97,    // position
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
        std::vector<double> dt(mpc_->GetNumNodes());
        std::fill(dt.begin(), dt.end(), 0.02);
        traj_out_.SetDtVector(dt);
        traj_out_.SetDefault(q_target_);
        traj_mpc_ = traj_out_;

        mpc_->SetWarmStartTrajectory(traj_out_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Warm start trajectory created...");

        mpc_->ComputeNLP(q_ic_, v_ic_, traj_mpc_);
        traj_out_ = traj_mpc_;
        traj_start_time_ = -1;

        // Compute an initial MPC at the initial condition

        // TODO: Put back
        // Go to initial condition
        // TransitionState(SeekInitialCond);
        TransitionState(Mpc);

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

    void AchillesController::ComputeMpc() {
        if (recieved_first_state_) {
            RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Computing first trajectory.");
            vectorx_t q, v;
            {
                // Get the mutex to protect the states
                std::lock_guard<std::mutex> lock(est_state_mut_);

                // Create current state
                q = q_;
                v = v_;
            }

            // RCLCPP_INFO_STREAM(this->get_logger(), "q: " << q.transpose());
            // RCLCPP_INFO_STREAM(this->get_logger(), "v: " << v.transpose());

            // TODO: Consider putting back
            // Get the current time
            // double time = this->get_clock()->now().seconds();

            // TODO: remove
            // traj_mpc_.GetVelocityInterp(0.02, v);
            // traj_mpc_.GetConfigInterp(0.02, q);
            // Shift the contact schedule
            contact_schedule_.ShiftContacts(-traj_mpc_.GetDtVec()[0]);    // TODO: Do I need a mutex on this later?
            mpc_->UpdateContactScheduleAndSwingTraj(contact_schedule_, 
                this->get_parameter("default_swing_height").as_double(), 0.0, 0.5);

            if (mpc_comps_ < 1000) {
                // std::cout << "mpc compute #" << mpc_comps_ << std::endl;
                // std::cout << "q: " << q.transpose() << std::endl;
                // std::cout << "v: " << v.transpose() << std::endl;

                mpc_->Compute(q, v, traj_mpc_);
                // mpc_->Compute(q, v, traj_mpc_); // TODO: Remove
                mpc_comps_++;
                {
                    // Get the traj mutex to protect it
                    std::lock_guard<std::mutex> lock(traj_out_mut_);
                    traj_out_ = traj_mpc_;

                    // Assign time time too
                    double time = this->get_clock()->now().seconds();
                    traj_start_time_ = time;
                }
                // RCLCPP_INFO_STREAM(this->get_logger(), "MPC Computation Completed!");
                if (GetState() != Mpc) {
                    TransitionState(Mpc);
                }
            } else {
                static bool printed = false;
                if (!printed) {
                    mpc_->PrintStatistics();
                    mpc_->PrintContactSchedule();
                    printed = true;
                }
            }

            // TODO: If this is slow, then I need to move it
            PublishTrajViz(traj_mpc_, viz_frames_);

        }
    }

    obelisk_control_msgs::msg::PDFeedForward AchillesController::ComputeControl() {
        // This callback does not need any access to the state as it outputs PD
        //  setpoints from the trajectory.

        obelisk_control_msgs::msg::PDFeedForward msg;
        if (ctrl_state_ != NoOutput) {
            RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Publishing first control.");
            
            vectorx_t q, v, tau;

            if (ctrl_state_ == Mpc) {
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
                }
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
    }

    void AchillesController::PublishTrajStateViz() {
        // std::lock_guard<std::mutex> lock(traj_out_mut_);

        // if (traj_start_time_ < 0) {
        //     traj_start_time_ = this->get_clock()->now().seconds();
        // }

        // double time = this->get_clock()->now().seconds();
        // // TODO: Do I need to use nanoseconds?
        // double time_into_traj = time - traj_start_time_;
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

        // msg.v_base = torc::utils::EigenToStdVector(v_head);
        // msg.v_joints = torc::utils::EigenToStdVector(v_tail);

        // this->GetPublisher<obelisk_estimator_msgs::msg::EstimatedState>("state_viz_pub")->publish(msg);
    }

} // namespace achilles


int main(int argc, char* argv[]) {
    obelisk::utils::SpinObelisk<achilles::AchillesController, rclcpp::executors::MultiThreadedExecutor>(
        argc, argv, "whole_body_controller");
}
