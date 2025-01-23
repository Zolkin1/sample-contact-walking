#include <vector>
#include <pthread.h>

#include "obelisk_node.h"
#include "obelisk_ros_utils.h"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "sensor_msgs/msg/joy_feedback.hpp"
// TODO: remove
#include "obelisk_estimator.h"

// ------ Mujoco Debug ----- //
// #include <GLFW/glfw3.h>
// ------ Mujoco Debug ----- //

#include "sample_contact_walking/robot_mpc.h"

#include "sample_contact_msgs/msg/commanded_target.hpp"

// TODO:
//  - Check for paths first relative to $SAMPLE_WALKING_ROOT then as a global path
//  - Add ROS diagonstic messages: https://docs.foxglove.dev/docs/visualization/panels/diagnostics#diagnosticarray
//  - I think the polytope commands are not being calculated correctly. When I go full tilt against the border I am still not getting the next polytope until after the end
//      of the horizon which seems suspicious as I should cross into the other polytope very soon.

namespace robot
{
    MpcController::MpcController(const std::string& name) 
    : obelisk::ObeliskController<obelisk_control_msgs::msg::PDFeedForward, obelisk_estimator_msgs::msg::EstimatedState>(name), 
        recieved_first_state_(false), first_mpc_computed_(false), ctrl_state_(NoOutput), traj_start_time_(-1), recieved_polytope_(false) {

        mujoco_sim_instance_ = this;

        this->declare_parameter<double>("mpc_loop_period_sec", 0.01);
        this->declare_parameter<long>("max_mpc_solves", -1);

        // --- Debug Publisher and Timer --- //
        this->RegisterObkTimer("state_viz_timer_setting", "state_viz_timer", std::bind(&MpcController::PublishTrajStateViz, this));
        this->RegisterObkPublisher<obelisk_estimator_msgs::msg::EstimatedState>("state_viz_pub_setting", "state_viz_pub");

        this->declare_parameter<std::string>("viz_pub_setting");
        // RCLCPP_ERROR_STREAM(this->get_logger(), "viz pub settings: " << this->get_parameter("viz_pub_setting").as_string());
        auto viz_pub = this->CreatePublisherFromConfigStr<visualization_msgs::msg::MarkerArray>(this->get_parameter("viz_pub_setting").as_string());
        this->publishers_["viz_pub"] = std::make_shared<obelisk::internal::ObeliskPublisher<visualization_msgs::msg::MarkerArray>>(viz_pub);

        // ----- Joystick Subscriber ----- //
        this->RegisterObkSubscription<sensor_msgs::msg::Joy>(
                    "joystick_sub_setting", "joystick_sub",
                    std::bind(&MpcController::JoystickCallback, this, std::placeholders::_1));

        // ----- Target Publisher ----- //
        this->RegisterObkPublisher<sample_contact_msgs::msg::CommandedTarget>(
                    "target_pub_setting", "target_pub");

        // ----- Contact Schedule Subscriber ----- //
        this->RegisterObkSubscription<sample_contact_msgs::msg::ContactSchedule>(
                    "contact_schedule_sub_setting", "contact_schedule_sub",
                    std::bind(&MpcController::ContactScheduleCallback, this, std::placeholders::_1));

        //  Update model
        this->declare_parameter<std::string>("urdf_path", "");
        std::filesystem::path urdf_path(this->get_parameter("urdf_path").as_string());

        // Create model
        this->declare_parameter<std::string>("robot_name", "");
        std::string robot_name = this->get_parameter("robot_name").as_string();
        RCLCPP_INFO_STREAM(this->get_logger(), "Config yaml robot name: " << robot_name);
        std::string model_name = name + robot_name + "_model";
        model_ = std::make_unique<torc::models::FullOrderRigidBody>(model_name, urdf_path);

        // ------------------------------------ //
        // ----- Create and configure MPC ----- //
        // ------------------------------------ //
        // ---------- Settings ---------- //
        mpc_settings_ = std::make_shared<torc::mpc::MpcSettings>(this->get_parameter("params_path").as_string());
        // mpc_settings_ = std::make_shared<torc::mpc::MpcSettings>("/home/zolkin/torc/tests/test_data/g1_mpc_config.yaml");
        mpc_settings_->Print();
        mpc_model_ = std::make_unique<torc::models::FullOrderRigidBody>(model_name, urdf_path, mpc_settings_->joint_skip_names, mpc_settings_->joint_skip_values);
        // mpc_model_ = std::make_unique<torc::models::FullOrderRigidBody>(model_name, "/home/zolkin/torc/tests/test_data/g1_hand.urdf", mpc_settings_->joint_skip_names, mpc_settings_->joint_skip_values);
        // ---------- Constraints ---------- //
        torc::models::FullOrderRigidBody mpc_model_temp(model_name, urdf_path, mpc_settings_->joint_skip_names, mpc_settings_->joint_skip_values);
        // torc::models::FullOrderRigidBody mpc_model_temp(model_name, "/home/zolkin/torc/tests/test_data/g1_hand.urdf", mpc_settings_->joint_skip_names, mpc_settings_->joint_skip_values);
        // Dynamics //
        std::vector<torc::mpc::DynamicsConstraint> dynamics_constraints;
        dynamics_constraints.emplace_back(mpc_model_temp, mpc_settings_->contact_frames, "robot_full_order",
            mpc_settings_->deriv_lib_path, mpc_settings_->compile_derivs, true, 0, mpc_settings_->nodes_full_dynamics);
        dynamics_constraints.emplace_back(mpc_model_temp, mpc_settings_->contact_frames, "robot_centroidal", mpc_settings_->deriv_lib_path,
            mpc_settings_->compile_derivs, false, mpc_settings_->nodes_full_dynamics, mpc_settings_->nodes);

        // Box constraints //
        // Config
        std::vector<int> config_lims_idxs;
        for (int i = 0; i < mpc_model_->GetConfigDim() - torc::mpc::FLOATING_BASE; ++i) {
            config_lims_idxs.push_back(i + torc::mpc::FLOATING_VEL);
        }
        torc::mpc::BoxConstraint config_box(1, mpc_settings_->nodes, model_name + "config_box",
            mpc_model_->GetLowerConfigLimits().tail(mpc_model_->GetConfigDim() - torc::mpc::FLOATING_BASE),
            mpc_model_->GetUpperConfigLimits().tail(mpc_model_->GetConfigDim() - torc::mpc::FLOATING_BASE),
            config_lims_idxs);

        // Vel
        std::vector<int> vel_lims_idxs;
        for (int i = 0; i < mpc_model_->GetVelDim() - torc::mpc::FLOATING_VEL; ++i) {
            vel_lims_idxs.push_back(i + torc::mpc::FLOATING_VEL);
        }
        torc::mpc::BoxConstraint vel_box(1, mpc_settings_->nodes, model_name + "vel_box",
            -mpc_model_->GetVelocityJointLimits().tail(mpc_model_->GetVelDim() - torc::mpc::FLOATING_VEL),
            mpc_model_->GetVelocityJointLimits().tail(mpc_model_->GetVelDim() - torc::mpc::FLOATING_VEL),
            vel_lims_idxs);

        // Torque
        std::vector<int> tau_lims_idxs;
        for (int i = 0; i < mpc_model_->GetVelDim() - torc::mpc::FLOATING_VEL; ++i) {
            tau_lims_idxs.push_back(i);
        }
        torc::mpc::BoxConstraint tau_box(0, mpc_settings_->nodes, model_name + "tau_box",
            -mpc_model_->GetTorqueJointLimits().tail(mpc_model_->GetVelDim() - torc::mpc::FLOATING_VEL),
            mpc_model_->GetTorqueJointLimits().tail(mpc_model_->GetVelDim() - torc::mpc::FLOATING_VEL),
            tau_lims_idxs);

        // ---------- Friction Cone Constraints ---------- //
        torc::mpc::FrictionConeConstraint friction_cone_constraint(0,mpc_settings_->nodes-1, model_name + "friction_cone_cone",
            mpc_settings_->friction_coef, mpc_settings_->friction_margin, mpc_settings_->deriv_lib_path, mpc_settings_->compile_derivs);

        // ---------- Swing Constraints ---------- //
        torc::mpc::SwingConstraint swing_constraint(2, mpc_settings_->nodes, model_name + "swing_constraint", mpc_model_temp, mpc_settings_->contact_frames,
            mpc_settings_->deriv_lib_path, mpc_settings_->compile_derivs);

        // ---------- Holonomic Constraints ---------- //
        torc::mpc::HolonomicConstraint holonomic_constraint(2, mpc_settings_->nodes, model_name + "holonomic_constraint", mpc_model_temp, 
            mpc_settings_->contact_frames, mpc_settings_->deriv_lib_path, mpc_settings_->compile_derivs);  // The -1 in the last node helps with weird issues (feasibility I think)

        std::cout << "===== Constraints Created =====" << std::endl;

        // --------------------------------- //
        // ------------- Costs ------------- //
        // --------------------------------- //
        // ---------- Velocity Tracking ---------- //
        torc::mpc::LinearLsCost vel_tracking(0, mpc_settings_->nodes, model_name + "vel_tracking", mpc_settings_->cost_data.at(1).weight,
            mpc_settings_->deriv_lib_path, mpc_settings_->compile_derivs);

        // ---------- Tau Tracking ---------- //
        torc::mpc::LinearLsCost tau_tracking(0, mpc_settings_->nodes, model_name + "tau_tracking", mpc_settings_->cost_data.at(2).weight,
            mpc_settings_->deriv_lib_path, mpc_settings_->compile_derivs);

        // ---------- Force Tracking ---------- //
        torc::mpc::LinearLsCost force_tracking(0, mpc_settings_->nodes, model_name + "force_tracking", mpc_settings_->cost_data.at(3).weight,
            mpc_settings_->deriv_lib_path, mpc_settings_->compile_derivs);

        // ---------- Config Tracking ---------- //
        torc::mpc::ConfigTrackingCost config_tracking(0, mpc_settings_->nodes, model_name + "config_tracking", mpc_settings_->cost_data.at(0).weight,
            mpc_settings_->deriv_lib_path, mpc_settings_->compile_derivs, mpc_model_temp);

        std::cout << "===== Costs Created =====" << std::endl;

        // --------------------------------- //
        // -------------- MPC -------------- //
        // --------------------------------- //
        // torc::mpc::MpcSettings mpc_settings_temp("/home/zolkin/torc/tests/test_data/g1_mpc_config.yaml");
        torc::mpc::MpcSettings mpc_settings_temp(this->get_parameter("params_path").as_string());
        mpc_ = std::make_shared<torc::mpc::HpipmMpc>(mpc_settings_temp, mpc_model_temp);

        std::cout << "===== MPC Created =====" << std::endl;
        mpc_->SetDynamicsConstraints(std::move(dynamics_constraints));
        mpc_->SetConfigBox(config_box);
        mpc_->SetVelBox(vel_box);
        mpc_->SetTauBox(tau_box);
        mpc_->SetFrictionCone(std::move(friction_cone_constraint));
        mpc_->SetSwingConstraint(std::move(swing_constraint));
        mpc_->SetHolonomicConstraint(std::move(holonomic_constraint));
        std::cout << "===== MPC Constraints Added =====" << std::endl;

        mpc_->SetVelTrackingCost(std::move(vel_tracking));
        mpc_->SetTauTrackingCost(std::move(tau_tracking));
        mpc_->SetForceTrackingCost(std::move(force_tracking));
        mpc_->SetConfigTrackingCost(std::move(config_tracking));
        std::cout << "===== MPC Costs Added =====" << std::endl;

        // torc::mpc::SimpleTrajectory q_target(mpc_model_->GetConfigDim(), mpc_settings_->nodes);
        // q_target.SetAllData(mpc_settings_->q_target);
        // mpc_->SetConfigTarget(q_target);

        // torc::mpc::SimpleTrajectory v_target(mpc_model_->GetVelDim(), mpc_settings_->nodes);
        // v_target.SetAllData(mpc_settings_->v_target);
        // mpc_->SetVelTarget(v_target);

        // mpc_->SetLinTrajConfig(q_target);
        // mpc_->SetLinTrajVel(v_target);

        // torc::mpc::ContactSchedule cs(mpc_settings_->contact_frames);
        // cs.InsertSwing("right_toe", 0.1, 0.4);
        // cs.InsertSwing("right_heel", 0.1, 0.4);
        // cs.InsertSwing("left_toe", 0.4, 0.8);   // TODO: Why does making it swing past the end of the traj hurt it?
        // cs.InsertSwing("left_heel", 0.4, 0.8);
        // mpc_->UpdateContactSchedule(cs);

        // vectorx_t q = mpc_settings_->q_target;

        // mpc_->Compute(q, vectorx_t::Zero(mpc_model_->GetVelDim()), traj_mpc_);
        // traj_mpc_.ExportToCSV(std::filesystem::current_path() / "trajectory_output.csv");

        // --------------------------------- //
        // ------------ Targets ------------ //
        // --------------------------------- //
        torc::mpc::Trajectory traj;
        q_target_ = torc::mpc::SimpleTrajectory(mpc_model_->GetConfigDim(), mpc_settings_->nodes);
        q_target_->SetAllData(mpc_settings_->q_target);
        // mpc_->SetConfigTarget(q_target_.value());
        z_target_ = mpc_settings_->q_target[2];

        v_target_ = torc::mpc::SimpleTrajectory(mpc_model_->GetVelDim(), mpc_settings_->nodes);
        v_target_->SetAllData(mpc_settings_->v_target);
        // mpc_->SetVelTarget(v_target_.value());

        // mpc_->SetLinTrajConfig(q_target_.value());
        // mpc_->SetLinTrajVel(v_target_.value());

        // Other variables
        this->declare_parameter<std::vector<long int>>("skip_indexes", {-1});
        skipped_joint_indexes_ = this->get_parameter("skip_indexes").as_integer_array();
        if (skipped_joint_indexes_[0] == -1 && (model_->GetConfigDim() == mpc_model_->GetConfigDim())) {
            RCLCPP_INFO_STREAM(this->get_logger(), "No joint to skip.");
        } else if (skipped_joint_indexes_.size() != model_->GetConfigDim() - mpc_model_->GetConfigDim()) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Provided skip indexes don't match the model differences! Got size: " << skipped_joint_indexes_.size() << ", expected: " 
                << model_->GetConfigDim() - mpc_model_->GetConfigDim());
        }

        // Parse contact schedule info
        ParseContactParameters();
        

        // // Setup q and v targets
        // this->declare_parameter<std::vector<double>>("target_config");
        // this->declare_parameter<std::vector<double>>("target_vel");
        // std::vector<double> q_targ_temp, v_targ_temp;

        // this->get_parameter("target_config", q_targ_temp);
        // this->get_parameter("target_vel", v_targ_temp);

        // vectorx_t q_target_eig = torc::utils::StdToEigenVector(q_targ_temp);
        // vectorx_t v_target_eig = torc::utils::StdToEigenVector(v_targ_temp);
        // q_target_ = torc::mpc::SimpleTrajectory(mpc_model_->GetConfigDim(), mpc_->GetNumNodes());
        // v_target_ = torc::mpc::SimpleTrajectory(mpc_model_->GetVelDim(), mpc_->GetNumNodes());
        // q_target_->SetAllData(q_target_eig);
        // v_target_->SetAllData(v_target_eig);
        // z_target_ = q_target_.value()[0](2);

        // mpc_->SetConfigTarget(q_target_.value());
        // mpc_->SetVelTarget(v_target_.value());

        // Set initial conditions
        this->declare_parameter<std::vector<double>>("mpc_ic_config");
        this->declare_parameter<std::vector<double>>("mpc_ic_vel");

        std::vector<double> q_ic_temp, v_ic_temp;
        q_ic_temp = this->get_parameter("mpc_ic_config").as_double_array();
        v_ic_temp = this->get_parameter("mpc_ic_vel").as_double_array();

        q_ic_ = torc::utils::StdToEigenVector(q_ic_temp);
        v_ic_ = torc::utils::StdToEigenVector(v_ic_temp);

        // RCLCPP_INFO_STREAM(this->get_logger(), "q target: " << q_target_.value()[0].transpose());
        // RCLCPP_INFO_STREAM(this->get_logger(), "v target: " << v_target_.value()[0].transpose());
        // RCLCPP_INFO_STREAM(this->get_logger(), "q ic: " << q_ic_.transpose());
        // RCLCPP_INFO_STREAM(this->get_logger(), "v ic: " << v_ic_.transpose());

        this->declare_parameter<bool>("fixed_target", true);
        this->get_parameter("fixed_target", fixed_target_);
        this->declare_parameter<bool>("controller_target", false);
        this->get_parameter("controller_target", controller_target_);

        // // Contact Polytope defaults
        // matrixx_t A_temp = matrixx_t::Identity(2, 2);
        // Eigen::Vector4d b_temp = Eigen::Vector4d::Zero();
        // b_temp << 10, 10, -10, -10;
        // for (const auto& frame : mpc_->GetContactFrames()) {
        //     // contact_polytopes_.insert({frame, {}});
        //     for (int i = 0; i < contact_schedule_.GetNumContacts(frame); i++) {
        //         contact_schedule_.SetPolytope(frame, i, A_temp, b_temp);
        //         // contact_polytopes_[frame].emplace_back((A_temp, b_temp));
        //     }
        // }

        // mpc_->SetWarmStartTrajectory(traj_out_);
        traj_out_ = mpc_->GetTrajectory();
        traj_out_.SetConfiguration(0, q_ic_);
        traj_out_.SetVelocity(0, v_ic_);
        traj_mpc_ = traj_out_;
        // mpc_->SetLinTrajConfig(traj_out_.GetConfigTrajectory());
        // mpc_->SetLinTrajVel(traj_out_.GetVelocityTrajectory());
        // RCLCPP_INFO_STREAM(this->get_logger(), "Warm start trajectory created...");

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

        for (const auto& frame : force_frames_) {
            std::cout << "Force frame: " << frame << std::endl;
        }

        this->declare_parameter<std::vector<std::string>>("polytope_frames", {""});
        this->get_parameter("polytope_frames", viz_polytope_frames_);

        // Spin up MPC thread
        mpc_thread_ = std::thread(&MpcController::MpcThread, this);

        // For target viz
        torso_mocap_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        this->MakeTargetTorsoMocapTransform();
    }

    void MpcController::UpdateXHat(const obelisk_estimator_msgs::msg::EstimatedState& msg) {
        // Get the mutex to protect the states
        std::lock_guard<std::mutex> lock(est_state_mut_);
        
        q_.resize(mpc_model_->GetConfigDim());
        v_.resize(mpc_model_->GetVelDim());


        // Configuration
        for (size_t i = 0; i < msg.q_base.size(); i++) {
            q_(i) = msg.q_base.at(i);
        }

        // Only receive the joints that we use in the MPC
        for (size_t i = 0; i < msg.q_joints.size(); i++) {
            const auto joint_idx = mpc_model_->GetJointID(msg.joint_names[i]);
            if (joint_idx.has_value()) {
                if (joint_idx.value() < 2) {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid joint name!");
                }
                q_(joint_idx.value() - 2 + msg.q_base.size()) = msg.q_joints.at(i);     // Offset for the root and base joints
            } else if (!model_->GetJointID(msg.joint_names[i]).has_value()) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Joint " << msg.joint_names[i] << " not found in the full robot model!");
            }
        }

        // Velocity
        for (size_t i = 0; i < msg.v_base.size(); i++) {
            v_(i) = msg.v_base.at(i);
        }

        for (size_t i = 0; i < msg.v_joints.size(); i++) {
            const auto joint_idx = mpc_model_->GetJointID(msg.joint_names[i]);
            if (joint_idx.has_value()) {
                if (joint_idx.value() < 2) {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid joint name!");
                }
                v_(joint_idx.value() - 2 + msg.v_base.size()) = msg.v_joints.at(i);     // Offset for the root and base joints
            } else if (!model_->GetJointID(msg.joint_names[i]).has_value()) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Joint " << msg.joint_names[i] << " not found in the full robot model!");
            }
        }

        if (!recieved_first_state_ && q_.size() == mpc_model_->GetConfigDim() && v_.size() == mpc_model_->GetVelDim() && q_.segment<QUAT_VARS>(POS_VARS).norm() > 0.95) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Recieved first message! q: " << q_.transpose());
            recieved_first_state_ = true;
        }
    }

    // Running the MPC in its own the thread seems to make the timing more consistent and overall faster
    // If I still need more, I can try to adjust the thread prio
    // Experimentally, note that the faster I run it, the more consistent (and faster, up to a limit) it is
    void MpcController::MpcThread() {
        const long mpc_loop_rate_ns = this->get_parameter("mpc_loop_period_sec").as_double()*1e9;
        RCLCPP_INFO_STREAM(this->get_logger(), "MPC loop period set to: " << mpc_loop_rate_ns << "ns.");

        const long max_mpc_solves = this->get_parameter("max_mpc_solves").as_int();
        
        // std::vector<double> stance_height(mpc_settings_->num_contact_locations);
        // int frame_idx = 0;
        // for (const auto& frame : mpc_settings_->contact_frames) {
        //     stance_height[frame_idx] = this->get_parameter("default_stand_foot_height").as_double();
        //     frame_idx++;
        // }

        static bool first_loop = true;
        auto prev_time = this->now();

        while (true) {
            // Start the timer
            auto start_time = this->now();

            if (recieved_first_state_ && GetState() == Mpc) {
                RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Computing first trajectory.");

                vectorx_t q, v;

                if (first_loop) {
                    // Read in state
                    {
                        // Get the mutex to protect the states
                        std::lock_guard<std::mutex> lock(est_state_mut_);

                        // Create current state
                        q = q_;
                        v = v_;
                    }

                    // TODO: Remove
                    q = q_ic_;
                    v = v_ic_;
                    // TODO: Fix the state for when we re-enter this loop
                    {
                        std::lock_guard<std::mutex> lock(polytope_mutex_);
                        // mpc_->UpdateContactSchedule(contact_schedule_);  // TODO: Put back
                    }
                    vectorx_t q_ref = q;
                    q_ref(2) = z_target_;
                    // mpc_->GenerateCostReference(q_ref, v, q_target_.value(), v_target_.value(), contact_schedule_);

                    // TODO: Remove
                    v.setZero();
                    q = mpc_settings_->q_target;

                    torc::mpc::SimpleTrajectory q_target(mpc_model_->GetConfigDim(), mpc_settings_->nodes);
                    q_target.SetAllData(mpc_settings_->q_target);
                    mpc_->SetConfigTarget(q_target);

                    torc::mpc::SimpleTrajectory v_target(mpc_model_->GetVelDim(), mpc_settings_->nodes);
                    v_target.SetAllData(mpc_settings_->v_target);
                    mpc_->SetVelTarget(v_target);

                    mpc_->SetLinTrajConfig(q_target);
                    mpc_->SetLinTrajVel(v_target);

                    torc::mpc::ContactSchedule cs(mpc_settings_->contact_frames);
                    cs.InsertSwing("right_toe", 0.1, 0.4);
                    cs.InsertSwing("right_heel", 0.1, 0.4);
                    cs.InsertSwing("left_toe", 0.4, 0.8);   // TODO: Why does making it swing past the end of the traj hurt it?
                    cs.InsertSwing("left_heel", 0.4, 0.8);
                    mpc_->UpdateContactSchedule(cs);

                    double time = this->now().seconds();
                    mpc_->Compute(q, vectorx_t::Zero(mpc_model_->GetVelDim()), traj_mpc_);
                    traj_mpc_.ExportToCSV(std::filesystem::current_path() / "trajectory_output.csv");
                    // mpc_->Compute(q, v, traj_mpc_);
                    // mpc_->Compute(q, v, traj_mpc_);
                    {
                        // Get the traj mutex to protect it
                        std::lock_guard<std::mutex> lock(traj_out_mut_);
                        traj_out_ = traj_mpc_;

                        // Assign time time too
                        traj_start_time_ = time;
                    }

                    prev_time = this->now();
                }



                // Shift the contact schedule
                {
                    std::lock_guard<std::mutex> lock(polytope_mutex_);
                    auto current_time = this->now();
                    double time_shift_sec = (current_time - prev_time).nanoseconds()/1e9;
                    contact_schedule_.ShiftSwings(-time_shift_sec);    // TODO: Do I need a mutex on this later?
                    next_left_insertion_time_ -= time_shift_sec;
                    next_right_insertion_time_ -= time_shift_sec;
                }
                prev_time = this->now();

                // TODO: Do I Need this - can I remove it?
                // TODO: If I need to, I can go back to using the measured foot height
                // std::vector<double> stance_height(mpc_settings_->num_contact_locations);
                // int frame_idx = 0;
                // for (const auto& frame : mpc_settings_->contact_frames) {
                //     stance_height[frame_idx] = this->get_parameter("default_stand_foot_height").as_double();
                //     frame_idx++;
                // }

                if (!recieved_polytope_) {
                    UpdateContactPolytopes();
                } 
                
                // ----- No Reference ----- //
                {
                    std::lock_guard<std::mutex> lock(polytope_mutex_);
                    mpc_->UpdateContactSchedule(contact_schedule_);
                    // mpc_->UpdateContactScheduleAndSwingTraj(contact_schedule_,
                    //     this->get_parameter("default_swing_height").as_double(),
                    //     stance_height,
                    //     this->get_parameter("apex_time").as_double());
                } 
                AddPeriodicContacts();   // Don't use when getting CS from the other node
                
                
                // ----- Read in state ----- //
                // TODO: If statement is only here for when we remove sim
                if (first_loop) {
                    first_loop = false;
                } else {
                    {
                        // Get the mutex to protect the states
                        std::lock_guard<std::mutex> lock(est_state_mut_);

                        // Create current state
                        q = q_;
                        v = v_;
                    }
                }
                // ----- No Reference ----- //
                if (!fixed_target_ || controller_target_) {
                    UpdateMpcTargets(q);
                    mpc_->SetConfigTarget(q_target_.value());
                    mpc_->SetVelTarget(v_target_.value());
                }

                // // ----- Reference Generation ----- //
                // // TODO: Unclear if this really provides a performance improvement as the stack works without it.
                // UpdateMpcTargets(q);
                // vectorx_t q_ref = q;
                // q_ref(2) = z_target_;
                // {
                //     std::lock_guard<std::mutex> lock(polytope_mutex_);
                //     mpc_->UpdateContactScheduleAndSwingTraj(contact_schedule_,
                //         this->get_parameter("default_swing_height").as_double(),
                //         stance_height,
                //         this->get_parameter("apex_time").as_double());
                //     // AddPeriodicContacts();   // Don't use when getting CS from the other node

                //     mpc_->GenerateCostReference(q, v, q_target_.value(), v_target_.value(), contact_schedule_);
                // }

                double time = this->now().seconds();
                // TODO: Remove
                v.setZero();
                // ---- Solve MPC ----- //
                // mpc_->Compute(q, v, traj_mpc_);
                {
                    // Get the traj mutex to protect it
                    std::lock_guard<std::mutex> lock(traj_out_mut_);
                    traj_out_ = traj_mpc_;

                    // Assign start time too
                    traj_start_time_ = time;
                }

                // Every tenth solve check to see if the user wants to print stats
                if (mpc_->GetSolveCounter() % 10 == 0 && print_timings_) {
                    // TODO: re-implement
                    // mpc_->PrintAggregateStats();
                    print_timings_ = false;
                }

                // TODO: If this is slow, then I need to move it
                PublishTrajViz(traj_mpc_, viz_frames_);
            } else {
                first_loop = true;
            }

            // Stop timer
            auto stop_time = this->now(); 

            // Compute difference
            const long time_left = mpc_loop_rate_ns - (stop_time - start_time).nanoseconds();
            // std::cout << "MPC Loop time (ms): " << static_cast<double>((stop_time - start_time).nanoseconds())/1e6 << std::endl;
            if (time_left > 0) {
                while ((-(this->now() - start_time).nanoseconds() + mpc_loop_rate_ns) > 0) {}
            } else {
                // TODO: Put back!
                // RCLCPP_WARN_STREAM(this->get_logger(), "MPC computation took longer than loop rate allowed for. " << std::abs(time_left)*1e-6 << "ms over time.");
            }
        }
    }

    obelisk_control_msgs::msg::PDFeedForward MpcController::ComputeControl() {
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
                    
                    double time_into_traj = 0;
                    if (traj_start_time_ >= 0) {
                        double time = this->get_clock()->now().seconds();
                        // TODO: Do I need to use nanoseconds?
                        time_into_traj = time - traj_start_time_;
                    }
                    
                    traj_out_.GetConfigInterp(time_into_traj, q);
                    traj_out_.GetVelocityInterp(time_into_traj, v);
                    traj_out_.GetTorqueInterp(time_into_traj, tau);

                    // This is a safety in case the interpolation time is too large
                    if (q.size() == 0) {
                        RCLCPP_WARN_STREAM(this->get_logger(), "Trajectory interpolation time is after the trajectory ends!");
                        q = traj_out_.GetConfiguration(traj_out_.GetNumNodes()-1);
                    }

                    if (v.size() == 0) {
                        v = traj_out_.GetVelocity(traj_out_.GetNumNodes()-1);
                    }

                    if (tau.size() == 0) {
                        tau = traj_out_.GetTau(traj_out_.GetNumNodes()-1);
                    }
                }
            } else if (ctrl_state_ == SeekInitialCond) {
                q = q_ic_;
                v = v_ic_;
                tau = vectorx_t::Zero(mpc_model_->GetNumInputs());
            }

            // Check if we need to insert other elements into the targets
            if (q.size() != model_->GetConfigDim()) {
                const auto& joint_skip_values = mpc_settings_->joint_skip_values;
                std::vector<double> q_vec(q.data(), q.data() + q.size());
                std::vector<double> v_vec(v.data(), v.data() + v.size());
                std::vector<double> tau_vec(tau.data(), tau.data() + tau.size());

                for (int i = 0; i < skipped_joint_indexes_.size(); i++) {
                    // TODO: Compute angle so that ankle is parallel to the ground
                    q_vec.insert(q_vec.begin() + FLOATING_POS_SIZE + skipped_joint_indexes_[i], joint_skip_values[i]);
                    v_vec.insert(v_vec.begin() + FLOATING_VEL_SIZE + skipped_joint_indexes_[i], 0);
                    tau_vec.insert(tau_vec.begin() + skipped_joint_indexes_[i], 0);
                }

                Eigen::Map<Eigen::VectorXd> q_map(q_vec.data(), q_vec.size());
                Eigen::Map<Eigen::VectorXd> v_map(v_vec.data(), v_vec.size());
                Eigen::Map<Eigen::VectorXd> tau_map(tau_vec.data(), tau_vec.size());

                q = q_map;
                v = v_map;
                tau = tau_map;
            }

            // Make the message
            vectorx_t u_mujoco = ConvertControlToMujocoU(q.tail(model_->GetNumInputs()),
                v.tail(model_->GetNumInputs()), tau);

            ConvertEigenToStd(u_mujoco, msg.u_mujoco);
            ConvertEigenToStd(q.tail(model_->GetNumInputs()), msg.pos_target);
            ConvertEigenToStd(v.tail(model_->GetNumInputs()), msg.vel_target);
            ConvertEigenToStd(tau, msg.feed_forward);
            
            if (msg.u_mujoco.size() != 3*model_->GetNumInputs()) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Message's u_mujoco is incorrectly sized. Size: " << msg.u_mujoco.size());
            }

            msg.header.stamp = this->now();
            this->GetPublisher<obelisk_control_msgs::msg::PDFeedForward>(this->ctrl_key_)->publish(msg);
        }

        return msg;
    }

    void MpcController::ConvertEigenToStd(const vectorx_t& eig_vec, std::vector<double>& std_vec) {
        std_vec.clear();
        for (int i = 0; i < eig_vec.size(); i++) {
            std_vec.emplace_back(eig_vec(i));
        }
    }

    vectorx_t MpcController::ConvertControlToMujocoU(const vectorx_t& pos_target, const vectorx_t& vel_target, const vectorx_t& feed_forward) {
        vectorx_t u(pos_target.size() + vel_target.size() + feed_forward.size());
        u << pos_target, vel_target, feed_forward;
        return u;
    }

    void MpcController::TransitionState(const ControllerState& new_state) {
        std::lock_guard<std::mutex> lock(ctrl_state_mut_);

        ctrl_state_ = new_state;
        std::string new_state_str = GetStateString(new_state);

        RCLCPP_INFO_STREAM(this->get_logger(), "Transitioning to state: " << new_state_str);
    }

    MpcController::ControllerState MpcController::GetState() {
        std::lock_guard<std::mutex> lock(ctrl_state_mut_);
        return ctrl_state_;
    }

    void MpcController::UpdateMpcTargets(const vectorx_t& q) {
        std::lock_guard<std::mutex> lock(target_state_mut_);

        const std::vector<double>& dt_vec = mpc_settings_->dt;

        q_target_.value()[0](0) = q(0);
        q_target_.value()[0](1) = q(1);
        q_target_.value()[0](2) = z_target_;

        const quat_t q_quat(q.segment<QUAT_VARS>(POS_VARS));
        vector3_t euler_angles = q_quat.toRotationMatrix().eulerAngles(2, 1, 0);

        // quat_t yaw_quaternion(Eigen::AngleAxisd(euler_angles[2], Eigen::Vector3d::UnitZ()));
        quat_t yaw_quaternion(q_quat.w(), 0, 0, q_quat.z()); 

        q_target_.value()[0](3) = yaw_quaternion.x();
        q_target_.value()[0](4) = yaw_quaternion.y();
        q_target_.value()[0](5) = yaw_quaternion.z();
        q_target_.value()[0](6) = yaw_quaternion.w();

        for (int i = 1; i < q_target_->GetNumNodes(); i++) {
            // const quat_t quat(q_target_.value()[i-1].segment<QUAT_VARS>(POS_VARS));
            // const matrix3_t R = quat.toRotationMatrix(); // TODO: Double check this isn't needed anymore!
            vectorx_t v = v_target_.value()[i];

            q_target_.value()[i] = pinocchio::integrate(mpc_model_->GetModel(), q_target_.value()[i-1], dt_vec[i-1]*v);
            q_target_.value()[i](2) = z_target_;
        }
    }

    // TODO: Remove when the call back is fully set up
    void MpcController::UpdateContactPolytopes() {
        std::lock_guard<std::mutex> lock(polytope_mutex_);
        matrixx_t A_temp = matrixx_t::Identity(2, 2);
        Eigen::Vector4d b_temp = Eigen::Vector4d::Zero();
        int frame_idx = 0;
        for (const auto& frame : mpc_settings_->contact_frames) {
            b_temp << 10 + q_(0), 10 + q_(1), -10 + q_(0), -10 + q_(1); //10, 10, -10, -10;
            if (frame_idx % 2 == 0) {
                b_temp = b_temp + Eigen::Vector4d::Constant(0.1*(frame_idx + 1));
            } else {
                b_temp = b_temp + Eigen::Vector4d::Constant(-0.1*(frame_idx));
            }
            for (int i = 0; i < contact_schedule_.GetNumContacts(frame); i++) {
                contact_schedule_.SetPolytope(frame, i, A_temp, b_temp);
            }

            frame_idx++;
        }

    }

    std::string MpcController::GetStateString(const ControllerState& state) {
        switch (state) {
        case SeekInitialCond:
            return "Seek Initial Condition";
            break;
        case Mpc:
            return "MPC";
        case NoOutput:
            return "No Output";
        default:
            RCLCPP_ERROR_STREAM(this->get_logger(), "State cannot be printed!");
            return "";
            break;
        }
    }

    void MpcController::PublishTrajViz(const torc::mpc::Trajectory& traj, const std::vector<std::string>& viz_frames) {
        // Compute FK for each frame and add the point to a marker message
        // TODO: Change this function so I don't need the lock the whole time
        visualization_msgs::msg::MarkerArray msg;
        msg.markers.resize(viz_frames.size() + force_frames_.size());
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
            mpc_model_->FirstOrderFK(traj.GetConfiguration(node));

            for (int i = 0; i < viz_frames.size(); i++) {
                vector3_t frame_pos = mpc_model_->GetFrameState(viz_frames[i]).placement.translation();
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

        // this->GetPublisher<visualization_msgs::msg::MarkerArray>("viz_pub")->publish(msg);

        // TODO: There is a bug in this block of code for the quad
        // Publish force arrows
        if (viz_forces_ && traj_start_time_ >= 0) {
            // visualization_msgs::msg::MarkerArray force_msg;
            // force_msg.markers.resize(force_frames_.size());
            mpc_model_->FirstOrderFK(traj.GetConfiguration(0));
            for (int i = viz_frames.size(); i < force_frames_.size() + viz_frames.size(); i++) {
                msg.markers[i].type = visualization_msgs::msg::Marker::LINE_STRIP;
                msg.markers[i].header.frame_id = "world";
                msg.markers[i].header.stamp = this->now();
                msg.markers[i].ns = "mpc_force_viz";
                msg.markers[i].id = i;
                msg.markers[i].action = visualization_msgs::msg::Marker::MODIFY;

                msg.markers[i].scale.x = 0.01; // Width of the arrows

                vector3_t frame_pos = mpc_model_->GetFrameState(force_frames_[i - viz_frames_.size()]).placement.translation();
                geometry_msgs::msg::Point start_point;
                start_point.x = frame_pos(0);
                start_point.y = frame_pos(1);
                start_point.z = frame_pos(2);

                msg.markers[i].points.emplace_back(start_point);

                vector3_t force_interp;
                {
                    double time = this->get_clock()->now().seconds();
                    double time_into_traj = time - traj_start_time_;
                    std::lock_guard<std::mutex> lock(traj_out_mut_);
                    traj_out_.GetForceInterp(time_into_traj, force_frames_[i - viz_frames_.size()], force_interp);
                }


                geometry_msgs::msg::Point end_point;
                end_point.x = start_point.x + force_interp(0)*scale_forces_;
                end_point.y = start_point.y + force_interp(1)*scale_forces_;
                end_point.z = start_point.z + force_interp(2)*scale_forces_;

                msg.markers[i].points.emplace_back(end_point);

                // *** Note *** The color is according to the node number, not necessarily the dt
                // Color according to node
                std_msgs::msg::ColorRGBA color;
                color.r = 0;
                color.g = 0;
                color.b = 1;
                color.a = 1;
                msg.markers[i].colors.push_back(color);
                msg.markers[i].colors.push_back(color);
            }
            
            // TODO: Merge this publish with the one above
            // this->GetPublisher<visualization_msgs::msg::MarkerArray>("viz_pub")->publish(msg);            
        }

        std::lock_guard<std::mutex> lock(polytope_mutex_);

        int num_polytope_markers = 0;
        for (const auto& frame : viz_polytope_frames_) {
            num_polytope_markers += contact_schedule_.GetNumContacts(frame);
        }
        
        msg.markers.resize(viz_frames.size() + force_frames_.size() + num_polytope_markers);

        // TODO: There is a bug in this block of code for the quad
        int i = viz_frames.size() + force_frames_.size();
        int frame_idx = 0;
        for (const auto& frame : viz_polytope_frames_) {
            // Visualize contact polytopes
            std::vector<torc::mpc::ContactInfo> polytope_vec;
            int num_contacts;
            {
                // Grab the contact polytopes
                // std::lock_guard<std::mutex> lock(polytope_mutex_); // Grabbed above

                polytope_vec = contact_schedule_.GetPolytopes(frame);
                num_contacts = contact_schedule_.GetNumContacts(frame);
            }

            if (num_contacts != polytope_vec.size()) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Frame: " << frame << " size: " << num_contacts << " num poly: " << polytope_vec.size());
            }

            std_msgs::msg::ColorRGBA color;
            // if (frame_idx % 2 == 0) {
            //     color.r = 1;
            //     color.g = 0;
            //     color.b = 1;
            //     color.a = 1;
            // } else {
            //     color.r = 1;
            //     color.g = 1;
            //     color.b = 0;
            //     color.a = 1;
            // }
            if (frame_idx == 0) {
                color.r = 1;
                color.g = 0;
                color.b = 1;
                color.a = 1;
            } else if (frame_idx == 1) {
                color.r = 1;
                color.g = 1;
                color.b = 1;
                color.a = 1;
            } else if (frame_idx == 2) {
                color.r = 1;
                color.g = 0;
                color.b = 0;
                color.a = 1;
            } else {
                color.r = 1;
                color.g = 1;
                color.b = 0;
                color.a = 1;
            }

            if (polytope_vec.size() != num_contacts) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Contact polytopes not of the correct size!");
            }

            for (const auto& polytope : polytope_vec) {
                // RCLCPP_INFO_STREAM(this->get_logger(), "A: " << polytope.A_ << "\nb: " << polytope.b_.transpose());

                msg.markers[i].type = visualization_msgs::msg::Marker::LINE_STRIP;
                msg.markers[i].header.frame_id = "world";
                msg.markers[i].header.stamp = this->now();
                msg.markers[i].ns = "contact_polytope";
                msg.markers[i].id = i;
                msg.markers[i].action = visualization_msgs::msg::Marker::MODIFY;

                msg.markers[i].scale.x = 0.02;

                // TODO: Do better
                // TODO: Check this to make sure it will work for more than the default polytope
                geometry_msgs::msg::Point corner;
                corner.z = 0;

                // std::cout << "b: " << polytope.b_.transpose() << std::endl;
                corner.x = polytope.b_(0);
                corner.y = polytope.b_(1);
                msg.markers[i].points.emplace_back(corner);
                msg.markers[i].colors.push_back(color);

                corner.x = polytope.b_(0);
                corner.y = polytope.b_(3);
                msg.markers[i].points.emplace_back(corner);
                msg.markers[i].colors.push_back(color);

                corner.x = polytope.b_(2);
                corner.y = polytope.b_(3);
                msg.markers[i].points.emplace_back(corner);
                msg.markers[i].colors.push_back(color);

                corner.x = polytope.b_(2);
                corner.y = polytope.b_(1);
                msg.markers[i].points.emplace_back(corner);
                msg.markers[i].colors.push_back(color);

                corner.x = polytope.b_(0);
                corner.y = polytope.b_(1);
                msg.markers[i].points.emplace_back(corner);
                msg.markers[i].colors.push_back(color);


                i++;
            }

            frame_idx++;
        }

        this->GetPublisher<visualization_msgs::msg::MarkerArray>("viz_pub")->publish(msg);            
    }

    void MpcController::PublishTrajStateViz() {
        // ---------- Achilles ---------- //
        // std::lock_guard<std::mutex> lock(traj_out_mut_);

        // if (traj_start_time_ < 0) {
        //     traj_start_time_ = this->get_clock()->now().seconds();
        // }
        // obelisk_estimator_msgs::msg::EstimatedState msg;

        // double time = this->get_clock()->now().seconds();
        // // TODO: Do I need to use nanoseconds?
        // double time_into_traj = time - traj_start_time_;
        // // double time_into_traj = 0;

        // // RCLCPP_INFO_STREAM(this->get_logger(), "Time into traj: " << time_into_traj);
        // vectorx_t q = vectorx_t::Zero(model_->GetConfigDim());
        // vectorx_t v = vectorx_t::Zero(model_->GetVelDim());
        // traj_out_.GetConfigInterp(time_into_traj, q);
        // traj_out_.GetVelocityInterp(time_into_traj, v);

        // // traj_out_.GetConfigInterp(0.01, q);
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

        // if (!sim_ready_) {
        //     this->GetPublisher<obelisk_estimator_msgs::msg::EstimatedState>("state_viz_pub")->publish(msg);
        // }

        // // ---------- Go2 ---------- //
        // // TODO: Consider putting back
        // // if (traj_start_time_ < 0) {
        // //     traj_start_time_ = this->get_clock()->now().seconds();
        // // }
        // obelisk_estimator_msgs::msg::EstimatedState msg;

        // vectorx_t q = vectorx_t::Zero(mpc_model_->GetConfigDim());
        // vectorx_t v = vectorx_t::Zero(mpc_model_->GetVelDim());
        // double time = this->get_clock()->now().seconds();
        // {
        //     std::lock_guard<std::mutex> lock(traj_out_mut_);
        //     double time_into_traj = time - traj_start_time_;

        //     // RCLCPP_INFO_STREAM(this->get_logger(), "Time into traj: " << time_into_traj);
        //     q = vectorx_t::Zero(model_->GetConfigDim());
        //     v = vectorx_t::Zero(model_->GetVelDim());
        //     if (GetState() == Mpc) {
        //         traj_out_.GetConfigInterp(time_into_traj, q);
        //         traj_out_.GetVelocityInterp(time_into_traj, v);
        //     } else {
        //         q = q_ic_;
        //         v = v_ic_;
        //     }
        // }

        // // traj_out_.GetConfigInterp(0.01, q);
        // msg.base_link_name = "torso";
        // vectorx_t q_head = q.head<FLOATING_POS_SIZE>();
        // vectorx_t q_tail = q.tail(model_->GetNumInputs());
        // msg.q_base = torc::utils::EigenToStdVector(q_head);
        // msg.q_joints.resize(model_->GetNumInputs());
        // msg.v_joints.resize(model_->GetNumInputs());

        // msg.joint_names.resize(q_tail.size());
        // msg.joint_names[0] = "FL_hip_joint";
        // msg.joint_names[1] = "FR_hip_joint";
        // msg.joint_names[2] = "RL_hip_joint";
        // msg.joint_names[3] = "RR_hip_joint";
        // msg.joint_names[4] = "FL_thigh_joint";
        // msg.joint_names[5] = "FR_thigh_joint";
        // msg.joint_names[6] = "RL_thigh_joint";
        // msg.joint_names[7] = "RR_thigh_joint";
        // msg.joint_names[8] = "FL_calf_joint";
        // msg.joint_names[9] = "FR_calf_joint";
        // msg.joint_names[10] = "RL_calf_joint";
        // msg.joint_names[11] = "RR_calf_joint";

        // for (int i = 0; i < msg.joint_names.size(); i++) {
        //     const auto idx = model_->GetJointID(msg.joint_names[i]);
        //     if (idx.has_value()) {
        //         msg.q_joints[i] = q(5 + idx.value());
        //         if (4 + idx.value() > v.size()) {
        //             RCLCPP_ERROR_STREAM(this->get_logger(), "v idx out of bounds!");
        //         } else {
        //             msg.v_joints[i] = v(4 + idx.value());
        //         }
        //     } else {
        //         RCLCPP_ERROR_STREAM(this->get_logger(), "Joint index not found!");
        //     }
        // }

        // // traj_out_.GetVelocityInterp(0.01, v);
        // vectorx_t v_head = v.head<FLOATING_VEL_SIZE>();

        // // vectorx_t temp = vectorx_t::Zero(6);
        // msg.v_base = torc::utils::EigenToStdVector(v_head);

        // msg.header.stamp = this->now();

        // // RCLCPP_ERROR_STREAM(this->get_logger(), "Publishing state viz");
        // // std::cerr << "Here " << std::endl;

        // // if (!sim_ready_) {
        // this->GetPublisher<obelisk_estimator_msgs::msg::EstimatedState>("state_viz_pub")->publish(msg);
        // // }

        // ---------- G1 ---------- //
        obelisk_estimator_msgs::msg::EstimatedState msg;

        vectorx_t q = vectorx_t::Zero(mpc_model_->GetConfigDim());
        vectorx_t v = vectorx_t::Zero(mpc_model_->GetVelDim());

        double time = this->get_clock()->now().seconds();
        // TODO: Do I need to use nanoseconds?
        // double time_into_traj = 0.75;
        {
            std::lock_guard<std::mutex> lock(traj_out_mut_);
            double time_into_traj = time - traj_start_time_;
            // RCLCPP_INFO_STREAM(this->get_logger(), "Time: " << time);
            // RCLCPP_INFO_STREAM(this->get_logger(), "Traj start time: " << traj_start_time_);
            // double time_into_traj = 0;

            // RCLCPP_INFO_STREAM(this->get_logger(), "Time into traj: " << time_into_traj)
            if (GetState() == Mpc) {
                // RCLCPP_WARN_STREAM(this->get_logger(), "Time into traj: " << time_into_traj);
                traj_out_.GetConfigInterp(time_into_traj, q);
                traj_out_.GetVelocityInterp(time_into_traj, v);
            } else {
                q = q_ic_;
                v = v_ic_;
            }
        }

        // traj_out_.GetConfigInterp(0.01, q);
        msg.base_link_name = "pelvis";
        vectorx_t q_head = q.head<FLOATING_POS_SIZE>();
        vectorx_t q_tail = q.tail(model_->GetNumInputs());
        msg.q_base = torc::utils::EigenToStdVector(q_head);
        msg.q_joints.resize(model_->GetNumInputs());
        msg.v_joints.resize(model_->GetNumInputs());

        msg.joint_names.resize(q_tail.size());
        // Left Leg
        msg.joint_names[0] = "left_hip_pitch_joint";
        msg.joint_names[1] = "left_hip_roll_joint";
        msg.joint_names[2] = "left_hip_yaw_joint";
        msg.joint_names[3] = "left_knee_joint";
        msg.joint_names[4] = "left_ankle_pitch_joint";
        msg.joint_names[5] = "left_ankle_roll_joint";

        // Right Leg
        msg.joint_names[6] = "right_hip_pitch_joint";
        msg.joint_names[7] = "right_hip_roll_joint";
        msg.joint_names[8] = "right_hip_yaw_joint";
        msg.joint_names[9] = "right_knee_joint";
        msg.joint_names[10] = "right_ankle_pitch_joint";
        msg.joint_names[11] = "right_ankle_roll_joint";

        // Torso
        msg.joint_names[12] = "waist_yaw_joint";
        msg.joint_names[13] = "waist_roll_joint";
        msg.joint_names[14] = "waist_pitch_joint";

        // Left Arm
        msg.joint_names[15] = "left_shoulder_pitch_joint";
        msg.joint_names[16] = "left_shoulder_roll_joint";
        msg.joint_names[17] = "left_shoulder_yaw_joint";
        msg.joint_names[18] = "left_elbow_joint";
        msg.joint_names[19] = "left_wrist_roll_joint";
        msg.joint_names[20] = "left_wrist_pitch_joint";
        msg.joint_names[21] = "left_wrist_yaw_joint";

        // Left Hand
        msg.joint_names[22] = "left_hand_thumb_0_joint";
        msg.joint_names[23] = "left_hand_thumb_1_joint";
        msg.joint_names[24] = "left_hand_thumb_2_joint";
        msg.joint_names[25] = "left_hand_middle_0_joint";
        msg.joint_names[26] = "left_hand_middle_1_joint";
        msg.joint_names[27] = "left_hand_index_0_joint";
        msg.joint_names[28] = "left_hand_index_1_joint";

        // Right Arm
        msg.joint_names[29] = "right_shoulder_pitch_joint";
        msg.joint_names[30] = "right_shoulder_roll_joint";
        msg.joint_names[31] = "right_shoulder_yaw_joint";
        msg.joint_names[32] = "right_elbow_joint";
        msg.joint_names[33] = "right_wrist_roll_joint";
        msg.joint_names[34] = "right_wrist_pitch_joint";
        msg.joint_names[35] = "right_wrist_yaw_joint";

        // Right Hand
        msg.joint_names[36] = "right_hand_thumb_0_joint";
        msg.joint_names[37] = "right_hand_thumb_1_joint";
        msg.joint_names[38] = "right_hand_thumb_2_joint";
        msg.joint_names[39] = "right_hand_middle_0_joint";
        msg.joint_names[40] = "right_hand_middle_1_joint";
        msg.joint_names[41] = "right_hand_index_0_joint";
        msg.joint_names[42] = "right_hand_index_1_joint";

        const auto joint_skip_names = mpc_settings_->joint_skip_names;
        const auto joint_skip_values = mpc_settings_->joint_skip_values;

        for (int i = 0; i < msg.joint_names.size(); i++) {
            const auto idx = mpc_model_->GetJointID(msg.joint_names[i]);
            if (idx.has_value()) {
                msg.q_joints[i] = q(5 + idx.value());
                if (4 + idx.value() > v.size()) {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "v idx out of bounds!");
                } else {
                    msg.v_joints[i] = v(4 + idx.value());
                }
            } else if (std::find(joint_skip_names.begin(), joint_skip_names.end(), msg.joint_names[i]) != joint_skip_names.end()) {     // Check if the joint is a skipped joint
                // Find the index
                for (int skip_idx = 0; skip_idx < joint_skip_names.size(); skip_idx++) {
                    if (joint_skip_names[skip_idx] == msg.joint_names[i]) {
                        msg.q_joints[i] = joint_skip_values[skip_idx];
                        break;
                    }
                } 
            } else {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Joint index not found!");
            }
        }

        // traj_out_.GetVelocityInterp(0.01, v);
        vectorx_t v_head = v.head<FLOATING_VEL_SIZE>();

        // vectorx_t temp = vectorx_t::Zero(6);
        msg.v_base = torc::utils::EigenToStdVector(v_head);

        msg.header.stamp = this->now();

        // if (!sim_ready_) {
        this->GetPublisher<obelisk_estimator_msgs::msg::EstimatedState>("state_viz_pub")->publish(msg);
        // }
    }

    void MpcController::MakeTargetTorsoMocapTransform() {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "target/torso_mocap_site";     // This must match the base link in the estimated state
        t.child_frame_id = "target/base_link"; //"target/base_link";  "target/torso"           // Must match the the base link in the urdf

        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(
        0.0,
        0.0, //3.14,
        0.0); //3.14);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        torso_mocap_broadcaster_->sendTransform(t);
    } 

    void MpcController::AddPeriodicContacts() {
        std::lock_guard<std::mutex> lock(polytope_mutex_);

        while (next_right_insertion_time_ < 1) {
            for (const auto& frame : right_frames_) {
                contact_schedule_.InsertSwingByDuration(frame, next_right_insertion_time_,  swing_time_);
            }

            next_right_insertion_time_ += 2*swing_time_;
        }

        while (next_left_insertion_time_ < 1) {
            for (const auto& frame : left_frames_) {
                contact_schedule_.InsertSwingByDuration(frame, next_left_insertion_time_,  swing_time_);
            }

            next_left_insertion_time_ += 2*swing_time_;
        }

        // Set polytopes for newly created contacts
        for (const auto& frame : mpc_settings_->contact_frames) {
            if (contact_schedule_.GetNumContacts(frame) > 0 && contact_schedule_.GetPolytopes(frame).back().A_.size() == 0) {
                torc::mpc::ContactInfo polytope = contact_schedule_.GetDefaultContactInfo();
                if (contact_schedule_.GetNumContacts(frame) > 1) {
                    polytope = contact_schedule_.GetPolytopes(frame).at(contact_schedule_.GetPolytopes(frame).size() - 2);
                }
                contact_schedule_.SetPolytope(frame, contact_schedule_.GetNumContacts(frame) - 1, polytope.A_, polytope.b_);
            }
        }

        contact_schedule_.CleanContacts(-1); //-0.1);
        // for (const auto& frame : mpc_->GetContactFrames()) {
        //     std::cout << frame << " num Contacts: " << contact_schedule_.GetNumContacts(frame) << std::endl;
        // }
    }

    void MpcController::ParseContactParameters() {
        // Make the contact schedule
        this->declare_parameter<double>("swing_time", 0.3);
        this->declare_parameter<double>("first_swing_time", 1);
        this->declare_parameter<double>("double_stance_time", 0.0);
        this->declare_parameter<bool>("right_foot_first", true);
        this->declare_parameter<std::vector<std::string>>("right_frames");
        this->declare_parameter<std::vector<std::string>>("left_frames");

        this->get_parameter("swing_time", swing_time_);
        this->get_parameter("first_swing_time", first_swing_time_);
        this->get_parameter("double_stance_time", double_stance_time_);
        this->get_parameter("right_foot_first", right_foot_first_);
        this->get_parameter("right_frames", right_frames_);
        this->get_parameter("left_frames", left_frames_);

        if (right_frames_.empty()) {
            throw std::runtime_error("No right foot frames provided!");
        }

        if (left_frames_.empty()) {
            throw std::runtime_error("No left foot frames provided!");
        }


        contact_schedule_.SetFrames(mpc_settings_->contact_frames);

        if (right_foot_first_) {
            for (const auto& rf : right_frames_) {
                contact_schedule_.InsertSwingByDuration(rf, first_swing_time_, swing_time_);
            }
            next_right_insertion_time_ = first_swing_time_ + 2*swing_time_;
            next_left_insertion_time_ = first_swing_time_ + swing_time_;
        } else {
            for (const auto& lf : right_frames_) {
                contact_schedule_.InsertSwingByDuration(lf, first_swing_time_, swing_time_);
            }
            next_left_insertion_time_ = first_swing_time_ + 2*swing_time_;
            next_right_insertion_time_ = first_swing_time_ + swing_time_;
        }

        AddPeriodicContacts();

        // TODO: Maybe put back
        // mpc_->UpdateContactSchedule(contact_schedule_);

        // mpc_->PrintContactSchedule();    // TODO: Consider re-implementing
        // for (const auto& frame : mpc_settings_->contact_frames) {
        //     mpc_->PrintSwingTraj(frame);
        // }
    }

    void MpcController::ContactScheduleCallback(const sample_contact_msgs::msg::ContactSchedule& msg) {
        // RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Recieved first contact schedule.");
        // recieved_polytope_ = true;
        // std::lock_guard<std::mutex> lock(polytope_mutex_);

        // // Grab the contact polytopes for the legs currently in swing (where they next land)
        // std::map<std::string, torc::mpc::ContactInfo> swing_polys;
        // for (const auto& [frame, swings] : contact_schedule_.GetScheduleMap()) {
        //     for (int i = 0; i < contact_schedule_.GetNumContacts(frame); i++) {
        //         if ((i > 0 && swings[i-1].first < 0 && swings[i-1].second > 0)) {
        //             swing_polys.insert({frame, contact_schedule_.GetPolytopes(frame)[i]});
        //         }
        //     }
        // }
        
        // // TODO: Consider removing if this doesn't work
        // contact_schedule_.CleanContacts(10);

        // for (const auto& contact_info : msg.contact_info) {
        //     const std::string& frame = contact_info.robot_contact_frame;

        //     const auto& sched_map = contact_schedule_.GetScheduleMap();
        //     if (sched_map.contains(frame)) {
        //         // std::cout << "swing times size: " << contact_info.swing_times.size() << std::endl;
        //         // TODO: Try Assigning the swing times from the command
        //         for (int i = 0; i < contact_info.swing_times.size()/2; i++) {
        //             contact_schedule_.InsertSwing(frame, contact_info.swing_times.at(2*i), contact_info.swing_times.at(2*i + 1));
        //             // std::cout << "inserting a swing!" << std::endl;
        //         }

        //         // std::cout << "Commanded swing times" << std::endl;
        //         // for (int i = 0; i < contact_info.swing_times.size(); i++) {
        //         //     std::cout << "i: " << i << ", swing times: " << contact_info.swing_times[i] << std::endl;
        //         // }
        //         // std::cout << "Current swing times" << std::endl;
        //         // for (int i = 0; i < sched_map.at(frame).size(); i++) {
        //         //     std::cout << "i: " << i << ", swing times: " << sched_map.at(frame)[i].first << ", " << sched_map.at(frame)[i].second << std::endl;
        //         // }

        //         if (contact_schedule_.GetNumContacts(frame) != contact_info.swing_times.size()/2 + 1) {
        //             std::cerr << "cs contact num: " << contact_schedule_.GetNumContacts(frame) << std::endl;
        //             std::cerr << "ci contact num: " << contact_info.swing_times.size()/2 + 1 << std::endl;
        //             throw std::runtime_error("Contacts not moved correctly!");
        //         }

        //         // Extract contact polytopes
        //         for (int i = 0; i < contact_schedule_.GetNumContacts(frame); i++) {
        //             // Only update the polytope if we haven't started the swing - may want to allow part of the swing

        //             const auto& swings = contact_schedule_.GetScheduleMap().at(frame);

        //             // TODO: Put back
        //             if (i == 0 || !(i > 0 && swings[i-1].first < 0 && swings[i-1].second > 0) || !swing_polys.contains(frame)) {
        //                 auto polytope = contact_info.polytopes.back();
        //                 if (i < contact_info.polytopes.size()) {
        //                     polytope = contact_info.polytopes[i];
        //                 } 

        //                 std::vector<double> a_mat = polytope.a_mat;

        //                 Eigen::Map<matrixx_t> A(a_mat.data(), 2, 2);
        //                 Eigen::Vector4d b(polytope.b_vec.data());
        //                 if (i < contact_schedule_.GetPolytopes(frame).size()) {
        //                     contact_schedule_.SetPolytope(frame, i, A, b);
        //                 }
        //             } else {
        //                 contact_schedule_.SetPolytope(frame, i, swing_polys.at(frame).A_, swing_polys.at(frame).b_);
        //             }
        //         }

        //         // std::cout << "done with " << frame << std::endl;
        //     } else {
        //         RCLCPP_ERROR_STREAM(this->get_logger(), frame << " is not a valid frame for the MPC contacts.");
        //     }
        // }
    }

    void MpcController::JoystickCallback(const sensor_msgs::msg::Joy& msg) {
        // ----- Axes ----- //
        constexpr int DPAD_VERTICAL = 7;
        constexpr int DPAD_HORIZONTAL = 6;

        constexpr int RIGHT_JOY_VERT = 4;
        constexpr int RIGHT_JOY_HORZ = 3;

        constexpr int LEFT_JOY_VERT = 1;
        constexpr int LEFT_JOY_HORZ = 0;

        constexpr int LEFT_TRIGGER = 2;
        constexpr int RIGHT_TRIGGER = 5;

        // ----- Buttons ----- //
        constexpr int A = 0;
        constexpr int B = 1;
        constexpr int X = 2;
        constexpr int Y = 3;

        constexpr int LEFT_BUMPER = 4;
        constexpr int RIGHT_BUMPER = 5;

        constexpr int MENU = 7;
        constexpr int SQUARES = 6;

        static rclcpp::Time last_menu_press = this->now();
        static rclcpp::Time last_A_press = this->now();
        static rclcpp::Time last_X_press = this->now();
        static rclcpp::Time last_B_press = this->now();
        static rclcpp::Time last_target_update = this->now();

        if (msg.buttons[MENU] && (this->now() - last_menu_press).seconds() > 1e-1) {
            RCLCPP_INFO_STREAM(this->get_logger(), "Press the menu button (three horizontal lines) to recieve this message.\n"
                "Press (X) to toggle between MPC and PD to the initial condition.\n"
                "Press (A) to print the current settings.\n"
                "Press (Y) to cycle through gaits.\n"
                "Press (B) to print the MPC timing statistics.\n"
                "Press the vertical DPAD to adjust the nominal standing height.\n"
                "Right joystick to change the target angle.\n"
                "Left joystick to adjust the desired velocity.");

            last_menu_press = this->now();
        }

        if (msg.buttons[X] && (this->now() - last_X_press).seconds() > 2e-1) {
            if (GetState() == SeekInitialCond) {
                TransitionState(Mpc);
            } else if (GetState() == NoOutput) {
                TransitionState(Mpc);
            } else if (GetState() == Mpc) {
                TransitionState(SeekInitialCond);
            }

            last_X_press = this->now();
        }

        if (msg.buttons[B] && (this->now() - last_B_press).seconds() > 2e-1) {
            print_timings_ = true;
        }

        if (msg.buttons[A] && (this->now() - last_A_press).seconds() > 1e-1) {
            std::lock_guard<std::mutex> lock(target_state_mut_);
            RCLCPP_INFO_STREAM(this->get_logger(), "Current state: " << GetStateString(GetState())
                    << "\nz target: " << z_target_ 
                    << "\nx vel target: " << v_target_.value()[0](0)
                    << "\ny vel target: " << v_target_.value()[0](1));
            last_A_press = this->now();
        }

        if (controller_target_ && (this->now() - last_target_update).seconds() > 1e-1) {
            std::lock_guard<std::mutex> lock(target_state_mut_);
            if (msg.axes[DPAD_VERTICAL] == 1) {
                // Update target z height
                z_target_ += 0.005;
            } else if (msg.axes[DPAD_VERTICAL] == -1) {
                // Update target z height
                z_target_ -= 0.005;
            }

            // For now the joy stick in one to one in velocity
            // In the future we may want to cap the speed above or below 1 and/or we may want a different curve to map them (i.e. log or quadratic)
            for (int i = 0; i < v_target_.value().GetNumNodes(); i++) {
                // TODO: Add some kind of "damping" so the target velocity doesnt change too much
                v_target_.value()[i](0) = msg.axes[LEFT_JOY_VERT];
                v_target_.value()[i](1) = msg.axes[LEFT_JOY_HORZ];
            }
            
            // TODO: Add a angular velocity target too using the right joystick
            for (int i = 0; i < v_target_.value().GetNumNodes(); i++) {
                v_target_.value()[i](5) = 1*msg.axes[RIGHT_JOY_HORZ];
            }
        }
        
        sample_contact_msgs::msg::CommandedTarget target_msg;
        target_msg.z_target = z_target_;
        for (int i = 0; i < v_target_.value()[0].size(); i++) {
            target_msg.v_target.emplace_back(v_target_.value()[0][i]);
        }

        this->GetPublisher<sample_contact_msgs::msg::CommandedTarget>("target_pub")->publish(target_msg);

    }

} // namespace robot


int main(int argc, char* argv[]) {
    obelisk::utils::SpinObelisk<robot::MpcController, rclcpp::executors::MultiThreadedExecutor>(
        argc, argv, "whole_body_controller");
}
