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

#include "torc_timer.h"

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

        // // ----- Contact Schedule Subscriber ----- //
        // this->RegisterObkSubscription<sample_contact_msgs::msg::ContactSchedule>(
        //             "contact_schedule_sub_setting", "contact_schedule_sub",
        //             std::bind(&MpcController::ContactScheduleCallback, this, std::placeholders::_1));

        // ----- Contact Schedule Subscriber ----- //
        this->RegisterObkSubscription<sample_contact_msgs::msg::ContactPolytopeArray>(
                    "polytope_sub_setting", "contact_polytope_sub",
                    std::bind(&MpcController::ContactPolytopeCallback, this, std::placeholders::_1));

        this->declare_parameter("robot_type", -1);


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

        // Make the polytope frame associations
        std::vector<std::pair<std::string, std::string>> poly_contact_frames;
        poly_contact_frames.emplace_back(mpc_settings_->polytope_frames[0], mpc_settings_->contact_frames[0]);
        poly_contact_frames.emplace_back(mpc_settings_->polytope_frames[0], mpc_settings_->contact_frames[1]);
        poly_contact_frames.emplace_back(mpc_settings_->polytope_frames[1], mpc_settings_->contact_frames[2]);
        poly_contact_frames.emplace_back(mpc_settings_->polytope_frames[1], mpc_settings_->contact_frames[3]);
        mpc_settings_->poly_contact_pairs = poly_contact_frames;

        mpc_model_ = std::make_unique<torc::models::FullOrderRigidBody>(model_name, urdf_path, mpc_settings_->joint_skip_names, mpc_settings_->joint_skip_values);
        // mpc_model_ = std::make_unique<torc::models::FullOrderRigidBody>(model_name, "/home/zolkin/torc/tests/test_data/g1_hand.urdf", mpc_settings_->joint_skip_names, mpc_settings_->joint_skip_values);
        
        torc::models::FullOrderRigidBody mpc_model_temp(model_name, urdf_path, mpc_settings_->joint_skip_names, mpc_settings_->joint_skip_values);

        // Reference Generator
        ref_gen_ = std::make_unique<torc::mpc::ReferenceGenerator>(mpc_settings_->nodes, mpc_settings_->contact_frames, mpc_settings_->dt,
            mpc_model_temp, mpc_settings_->polytope_delta);

        // ---------- Constraints ---------- //
        // torc::models::FullOrderRigidBody mpc_model_temp(model_name, "/home/zolkin/torc/tests/test_data/g1_hand.urdf", mpc_settings_->joint_skip_names, mpc_settings_->joint_skip_values);
        // Dynamics //
        // ---------- Full Order Dynamics ---------- //
        torc::mpc::DynamicsConstraint dynamics_constraint(mpc_model_temp, mpc_settings_->contact_frames, model_name + "_robot_full_order",
            mpc_settings_->deriv_lib_path, mpc_settings_->compile_derivs, 0, mpc_settings_->nodes_full_dynamics);

        // ---------- Centroidal Dynamics ---------- //
        torc::mpc::CentroidalDynamicsConstraint centroidal_dynamics(mpc_model_temp, mpc_settings_->contact_frames, model_name + "_robot_centroidal",
            mpc_settings_->deriv_lib_path, mpc_settings_->compile_derivs, mpc_settings_->nodes_full_dynamics, mpc_settings_->nodes); // 0, mpc_settings_->nodes
        // ---------- SRB Dynamics ---------- //
        torc::mpc::SRBConstraint srb_dynamics(mpc_settings_->nodes_full_dynamics - 100, mpc_settings_->nodes - 100, // 0 - 100, mpc_settings_->nodes - 100,
             model_name + "_robot_srb",
            mpc_settings_->contact_frames, mpc_settings_->deriv_lib_path, mpc_settings_->compile_derivs, mpc_model_temp, mpc_settings_->q_target);

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

        // Force
        std::vector<int> force_lim_idxs;
        for (int i = 0; i < 3; i++) {
            force_lim_idxs.push_back(i);
        }
        vectorx_t stance_lb(3), stance_ub(3);
        stance_lb << -1000, -1000, mpc_settings_->min_grf;
        stance_ub << 1000, 1000, mpc_settings_->max_grf;
        torc::mpc::BoxConstraint stance_force_box(0, mpc_settings_->nodes, "stance_force_box",
            stance_lb, // Minimum force on the ground
            stance_ub,
            force_lim_idxs);

        vectorx_t swing_lb(3), swing_ub(3);
        swing_lb << 0, 0, 0;
        swing_ub << 0, 0, 0;
        torc::mpc::BoxConstraint swing_force_box(0, mpc_settings_->nodes, "swing_force_box",
            swing_lb, // Minimum force on the ground
            swing_ub,
            force_lim_idxs);

        // ---------- Friction Cone Constraints ---------- //
        torc::mpc::FrictionConeConstraint friction_cone_constraint(0, mpc_settings_->nodes - 1, model_name + "friction_cone_cone",
            mpc_settings_->friction_coef, mpc_settings_->friction_margin, mpc_settings_->deriv_lib_path, mpc_settings_->compile_derivs);

        // ---------- Swing Constraints ---------- //
        torc::mpc::SwingConstraint swing_constraint(mpc_settings_->swing_start_node, mpc_settings_->swing_end_node, model_name + "swing_constraint",
            mpc_model_temp, mpc_settings_->contact_frames,
            mpc_settings_->deriv_lib_path, mpc_settings_->compile_derivs);

        // ---------- Holonomic Constraints ---------- //
        //2, nodes
        torc::mpc::HolonomicConstraint holonomic_constraint(mpc_settings_->holonomic_start_node, mpc_settings_->holonomic_end_node, model_name + "holonomic_constraint", mpc_model_temp, 
            mpc_settings_->contact_frames, mpc_settings_->deriv_lib_path, mpc_settings_->compile_derivs);  // The -1 in the last node helps with weird issues (feasibility I think)

        // ---------- Collision Constraints ---------- //
        torc::mpc::CollisionConstraint collision_constraint(mpc_settings_->collision_start_node, mpc_settings_->collision_end_node,
            model_name + "collision_constraint", mpc_model_temp, mpc_settings_->deriv_lib_path, mpc_settings_->compile_derivs, mpc_settings_->collision_data);

        // ---------- Polytope Constraints ---------- //
        torc::mpc::PolytopeConstraint polytope_constraint(mpc_settings_->polytope_start_node, mpc_settings_->polytope_end_node, model_name + "polytope_constraint",
            mpc_settings_->polytope_frames,
            mpc_settings_->deriv_lib_path, 
            mpc_settings_->compile_derivs,
            mpc_model_temp);

        std::cout << "===== Constraints Created =====" << std::endl;

        // --------------------------------- //
        // ------------- Costs ------------- //
        // --------------------------------- //
        // ---------- Velocity Tracking ---------- //
        torc::mpc::LinearLsCost vel_tracking(0, mpc_settings_->nodes, model_name + "vel_tracking",
            mpc_settings_->deriv_lib_path, mpc_settings_->compile_derivs, mpc_settings_->cost_data.at(1).weight.size());

        // ---------- Tau Tracking ---------- //
        torc::mpc::LinearLsCost tau_tracking(0, mpc_settings_->nodes, model_name + "tau_tracking",
            mpc_settings_->deriv_lib_path, mpc_settings_->compile_derivs, mpc_settings_->cost_data.at(2).weight.size());

        // ---------- Force Tracking ---------- //
        torc::mpc::LinearLsCost force_tracking(0, mpc_settings_->nodes, model_name + "force_tracking",
            mpc_settings_->deriv_lib_path, mpc_settings_->compile_derivs, mpc_settings_->cost_data.at(3).weight.size());

        // ---------- Config Tracking ---------- //
        torc::mpc::ConfigTrackingCost config_tracking(0, mpc_settings_->nodes, model_name + "config_tracking", mpc_settings_->cost_data.at(0).weight.size(),
            mpc_settings_->deriv_lib_path, mpc_settings_->compile_derivs, mpc_model_temp);

        // ---------- Forward Kinematics Tracking ---------- //
        // For now they all need the same weight
        // Need to read the contact frames from the settings (TODO, later)
        RCLCPP_INFO_STREAM(this->get_logger(), "FK weight: " << mpc_settings_->cost_data.at(4).weight.transpose());
        torc::mpc::ForwardKinematicsCost fk_cost(0, mpc_settings_->nodes, model_name + "fk_cost", mpc_settings_->cost_data.at(4).weight.size(),
            mpc_settings_->deriv_lib_path, mpc_settings_->compile_derivs, mpc_model_temp, mpc_settings_->contact_frames);


        std::cout << "===== Costs Created =====" << std::endl;

        // --------------------------------- //
        // -------------- MPC -------------- //
        // --------------------------------- //
        // torc::mpc::MpcSettings mpc_settings_temp("/home/zolkin/torc/tests/test_data/g1_mpc_config.yaml");
        torc::mpc::MpcSettings mpc_settings_temp(this->get_parameter("params_path").as_string());
        mpc_settings_temp.poly_contact_pairs = poly_contact_frames;
        mpc_ = std::make_shared<torc::mpc::HpipmMpc>(mpc_settings_temp, mpc_model_temp);
        std::cout << "===== MPC Created =====" << std::endl;

        mpc_->SetDynamicsConstraints(std::move(dynamics_constraint));
        mpc_->SetCentroidalDynamicsConstraints(std::move(centroidal_dynamics));
        // mpc_->SetSrbConstraint(std::move(srb_dynamics));
        mpc_->SetConfigBox(config_box);
        mpc_->SetVelBox(vel_box);
        mpc_->SetTauBox(tau_box);
        mpc_->SetForceBox(stance_force_box, swing_force_box);
        mpc_->SetFrictionCone(std::move(friction_cone_constraint));
        mpc_->SetSwingConstraint(std::move(swing_constraint));
        mpc_->SetHolonomicConstraint(std::move(holonomic_constraint));
        mpc_->SetCollisionConstraint(std::move(collision_constraint));
        mpc_->SetPolytopeConstraint(std::move(polytope_constraint));
        std::cout << "===== MPC Constraints Added =====" << std::endl;

        mpc_->SetVelTrackingCost(std::move(vel_tracking));
        mpc_->SetTauTrackingCost(std::move(tau_tracking));
        mpc_->SetForceTrackingCost(std::move(force_tracking));
        mpc_->SetConfigTrackingCost(std::move(config_tracking));
        mpc_->SetFowardKinematicsCost(std::move(fk_cost));  // TODO: Fix for biped
        std::cout << "===== MPC Costs Added =====" << std::endl;

        // --------------------------------- //
        // -------------- WBC -------------- //
        // --------------------------------- //
        wbc_settings_ = std::make_shared<torc::controller::WbcSettings>(this->get_parameter("params_path").as_string());
        wbc_model_ = std::make_unique<torc::models::FullOrderRigidBody>(model_name, urdf_path, mpc_settings_->joint_skip_names, mpc_settings_->joint_skip_values); //wbc_settings_->skip_joints, wbc_settings_->joint_values);
        wbc_controller_ = std::make_unique<torc::controller::WbcController>(mpc_model_temp, mpc_settings_->contact_frames,
            *wbc_settings_, mpc_settings_->friction_coef, wbc_settings_->verbose, mpc_settings_->deriv_lib_path, wbc_settings_->compile_derivs);

        // --------------------------------- //
        // ------------ Targets ------------ //
        // --------------------------------- //
        torc::mpc::Trajectory traj;
        q_target_ = torc::mpc::SimpleTrajectory(mpc_model_->GetConfigDim(), mpc_settings_->nodes);
        q_target_->SetAllData(mpc_settings_->q_target);
        mpc_->SetConfigTarget(q_target_.value());
        z_target_ = mpc_settings_->q_target[2];

        v_target_ = torc::mpc::SimpleTrajectory(mpc_model_->GetVelDim(), mpc_settings_->nodes);
        v_target_->SetAllData(mpc_settings_->v_target);
        mpc_->SetVelTarget(v_target_.value());

        mpc_->SetLinTrajConfig(q_target_.value());
        mpc_->SetLinTrajVel(v_target_.value());

        // --------------------------------- //
        // ---------- Skip Joints ---------- //
        // --------------------------------- //        
        this->declare_parameter<std::vector<long int>>("skip_indexes", {-1});
        mpc_skipped_joint_indexes_ = this->get_parameter("skip_indexes").as_integer_array();
        if (mpc_skipped_joint_indexes_[0] == -1 && (model_->GetConfigDim() == mpc_model_->GetConfigDim())) {
            RCLCPP_INFO_STREAM(this->get_logger(), "No joint to skip.");
        } else if (mpc_skipped_joint_indexes_.size() != model_->GetConfigDim() - mpc_model_->GetConfigDim()) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Provided skip indexes don't match the model differences! Got size: " << mpc_skipped_joint_indexes_.size() << ", expected: " 
                << model_->GetConfigDim() - mpc_model_->GetConfigDim());
        }
        this->declare_parameter<std::vector<double>>("skip_joint_vals", {-1});
        mpc_skipped_joint_vals_ = this->get_parameter("skip_joint_vals").as_double_array();

        if (mpc_skipped_joint_vals_.size() != mpc_skipped_joint_indexes_.size()) {
            throw std::runtime_error("MPC skip joint indexes size does not match the corresponding value array!");
        }

        // this->declare_parameter<std::vector<long int>>("wbc_skip_indexes", {-1});
        // wbc_skipped_joint_indexes_ = this->get_parameter("wbc_skip_indexes").as_integer_array();

        // --------------------------------- //
        // Make the joint names
        // --------------------------------- //
        this->declare_parameter<std::vector<std::string>>("control_joint_names", {""});
        control_joint_names_ = this->get_parameter("control_joint_names").as_string_array();

        RCLCPP_INFO_STREAM(this->get_logger(), "Control joint names:");
        for (int i = 0; i < control_joint_names_.size(); i++) {
            RCLCPP_INFO_STREAM(this->get_logger(), control_joint_names_[i]);
        }

        this->declare_parameter<std::vector<double>>("kp", {0.});
        this->declare_parameter<std::vector<double>>("kd", {0.});
        kp_ = this->get_parameter("kp").as_double_array();
        kd_ = this->get_parameter("kd").as_double_array();

        if (kp_.size() != kd_.size() || kp_.size() != control_joint_names_.size()) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "kp size: " << kp_.size());
            RCLCPP_ERROR_STREAM(this->get_logger(), "kd size: " << kd_.size());
            RCLCPP_ERROR_STREAM(this->get_logger(), "joint name size: " << control_joint_names_.size());
            throw std::runtime_error("kp, kd, or joint name sizes don't match!");
        }


        // Parse contact schedule info
        ParseContactParameters();

        // Step Planning
        std::vector<torc::mpc::ContactInfo> contact_polytopes;
        torc::mpc::vector4_t b;
        b << 10, 10, -10, -10;
        contact_polytopes.emplace_back(torc::mpc::matrix2_t::Identity(), b);
        this->declare_parameter<std::vector<double>>("foot_offsets", {-1});
        std::vector<double> contact_offsets = this->get_parameter("foot_offsets").as_double_array();
        step_planner_ = std::make_unique<torc::step_planning::StepPlanner>(contact_polytopes, mpc_settings_->contact_frames, contact_offsets,
            0.4, mpc_settings_->polytope_delta);

        
        // Set initial conditions
        this->declare_parameter<std::vector<double>>("mpc_ic_config");
        this->declare_parameter<std::vector<double>>("mpc_ic_vel");

        std::vector<double> q_ic_temp, v_ic_temp;
        q_ic_temp = this->get_parameter("mpc_ic_config").as_double_array();
        v_ic_temp = this->get_parameter("mpc_ic_vel").as_double_array();

        q_ic_ = torc::utils::StdToEigenVector(q_ic_temp);
        v_ic_ = torc::utils::StdToEigenVector(v_ic_temp);

        this->declare_parameter<bool>("fixed_target", true);
        this->get_parameter("fixed_target", fixed_target_);
        this->declare_parameter<bool>("controller_target", false);
        this->get_parameter("controller_target", controller_target_);

        traj_out_ = mpc_->GetTrajectory();
        traj_out_.SetConfiguration(0, q_ic_);
        traj_out_.SetVelocity(0, v_ic_);
        traj_mpc_ = traj_out_;
        mpc_->SetLinTraj(traj_mpc_);


        // Go to initial condition
        TransitionState(SeekInitialCond);

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
            RCLCPP_INFO_STREAM(this->get_logger(), "Viz force frame: " << frame);
        }

        this->declare_parameter<std::vector<std::string>>("polytope_frames", {""});
        this->get_parameter("polytope_frames", viz_polytope_frames_);

        this->declare_parameter<std::string>("mpc_loop_log", {"mpc_loop_log.csv"});
        this->declare_parameter<std::string>("mpc_timing_log", {"mpc_timing_log.csv"});
        std::string mpc_loop_log_name = this->get_parameter("mpc_loop_log").as_string();
        std::string mpc_timing_log_name = this->get_parameter("mpc_timing_log").as_string();

        this->get_parameter("polytope_frames", viz_polytope_frames_);

        time_offset_ = this->now().seconds();
        log_file_.open("mpc_logs/" + mpc_loop_log_name);
        timing_log_file_.open("mpc_logs/" + mpc_timing_log_name);

        // Spin up MPC thread
        mpc_thread_ = std::thread(&MpcController::MpcThread, this);

        // For target viz
        torso_mocap_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        this->MakeTargetTorsoMocapTransform();
    }

    MpcController::~MpcController() {
        log_file_.close();
        timing_log_file_.close();

        if (mpc_thread_.joinable()) {
            mpc_thread_.join();
        }
    }

    void MpcController::UpdateXHat(const obelisk_estimator_msgs::msg::EstimatedState& msg) {
        // Get the mutex to protect the states
        std::lock_guard<std::mutex> lock(est_state_mut_);
        
        q_.resize(mpc_model_->GetConfigDim());
        v_.resize(mpc_model_->GetVelDim());

        if (msg.joint_names.size() != msg.q_joints.size()) {
            throw std::runtime_error("[UpdateXHat] msg is not self consistent!");
        }


        // Configuration
        for (size_t i = 0; i < msg.q_base.size(); i++) {
            q_(i) = msg.q_base.at(i);
        }

        if ((q_.segment<4>(3).norm() - 1) > 1e-4) {
            recieved_first_state_ = false;
            RCLCPP_ERROR_STREAM(this->get_logger(), "q floating base: " << q_.head<7>());
            throw std::runtime_error("recieved q is not normalized!");
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
                throw std::runtime_error("[UpdateXHat] Joint name not found!");
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
                throw std::runtime_error("[UpdateXHat] Joint name not found!");
            }
        }

        if (GetState() != Mpc) {
            // Set the initial condition x-y-z position to where the robot is
            q_ic_.head<7>() = q_.head<7>();
            q_target_.value()[0].head<7>() = q_.head<7>();
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
        // Prevents uncessary cache misses by pinning to a CPU
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(2, &cpuset);  // Pin to CPU 2

        pthread_t thread = pthread_self();
        if (pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset) != 0) {
            perror("pthread_setaffinity_np");
        }

        struct sched_param param;
        param.sched_priority = 99;
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);

        const long mpc_loop_rate_ns = this->get_parameter("mpc_loop_period_sec").as_double()*1e9;
        RCLCPP_INFO_STREAM(this->get_logger(), "MPC loop period set to: " << mpc_loop_rate_ns << "ns.");
        
        static bool first_loop = true;

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

                    // // TODO: Remove
                    // q = q_ic_;
                    // v = v_ic_;
                    // TODO: Fix the state for when we re-enter this loop
                    {
                        std::lock_guard<std::mutex> lock(polytope_mutex_);
                        mpc_->UpdateContactSchedule(contact_schedule_);  // TODO: There is an issue with polytopes here
                    }

                    UpdateMpcTargets(q);
                    mpc_->SetConfigTarget(q_target_.value());
                    mpc_->SetVelTarget(v_target_.value());

                    for (int i = 0; i < 2; i++) {   // TODO: If i run too many iterations of this then sometimes I get a quat normilazation error in the next loop
                        mpc_->Compute(this->now().seconds() - time_offset_, q, v, traj_mpc_);
                    }
                    double time = this->now().seconds();
                    {
                        // Get the traj mutex to protect it
                        std::lock_guard<std::mutex> lock(traj_out_mut_);
                        traj_out_ = traj_mpc_;

                        // Assign time time too
                        traj_start_time_ = time;
                    }

                    // prev_time = this->now();

                    first_loop = false;
                }

                // ------------------------------------ //
                // ---------- MPC Computation --------- //
                // ------------------------------------ //
                double prep_time = PreperationPhase();
                const auto [fb_time, fb_prep_time] = FeedbackPhase();

                // Log timing
                timing_log_file_ << this->now().seconds() - time_offset_ << "," << prep_time + fb_prep_time << "," << fb_time << std::endl;

                std::cout << "Prep time took " << prep_time + fb_prep_time << " ms" << std::endl;
                std::cout << "Feedback time took " << fb_time << " ms" << std::endl;

                // TODO: If this is slow, then I need to move it
                torc::utils::TORCTimer viz_timer;
                viz_timer.Tic();
                PublishTrajViz(traj_mpc_, viz_frames_);
                viz_timer.Toc();
                std::cout << "viz publish took " << viz_timer.Duration<std::chrono::microseconds>().count()/1000.0 << "ms" << std::endl;
            } else {
                first_loop = true;
            }

            // Stop timer
            auto stop_time = this->now(); 

            // Compute difference
            const long time_left = mpc_loop_rate_ns - (stop_time - start_time).nanoseconds();
            // if ((stop_time - start_time).nanoseconds()*1e-6 > 100) {
            //     std::cerr << "MPC Setup: " << setup_timer.Duration<std::chrono::microseconds>().count()/1000.0 << "ms" << std::endl;
            //     std::cerr << "MPC Compute + Copy: " << mpc_timer.Duration<std::chrono::microseconds>().count()/1000.0 << "ms" << std::endl;
            //     throw std::runtime_error("MPC computation took longer than 100ms!");
            // }
            // std::cout << "MPC Loop time (ms): " << static_cast<double>((stop_time - start_time).nanoseconds())/1e6 << std::endl;
            if (time_left > 0) {
                while ((-(this->now() - start_time).nanoseconds() + mpc_loop_rate_ns) > 0) {}
            } else {
                // TODO: Put back!
                // RCLCPP_WARN_STREAM(this->get_logger(), "MPC computation took longer than loop rate allowed for. " << std::abs(time_left)*1e-6 << "ms over time.");
            }
        }
    }

    double MpcController::PreperationPhase() {
        torc::utils::TORCTimer timer;
        timer.Tic();

        static auto prev_time = this->now();

        // Update contact schedule and polytopes
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

        if (!recieved_polytope_) {
            UpdateContactPolytopes();
        } 
        
        // ----- No Reference ----- //
        torc::utils::TORCTimer step_planner_timer;
        {
            std::lock_guard<std::mutex> lock(polytope_mutex_);
            // TODO: Look into what target to use, for now just use the old targets
            // TODO: Consider making this update at a slower rate (20-50Hz)
            step_planner_timer.Tic();
            step_planner_->PlanStepsHeuristic(q_target_.value(), mpc_settings_->dt, contact_schedule_, nom_footholds_, projected_footholds_);
            step_planner_timer.Toc();
            mpc_->UpdateContactSchedule(contact_schedule_); // TODO: Need to do this at the same time as the reference generation
        }
        AddPeriodicContacts();   // Don't use when getting CS from the other node
                

        // Linearize around current trajectory
        mpc_->CreateQPData();

        timer.Toc();

        std::cout << "step planner took " << step_planner_timer.Duration<std::chrono::microseconds>().count()/1000.0 << " ms" << std::endl;

        return timer.Duration<std::chrono::microseconds>().count()/1000.0;
    }

    std::pair<double, double> MpcController::FeedbackPhase() {
        torc::utils::TORCTimer timer;
        timer.Tic();

        vectorx_t q, v;

        static bool first_loop = true;
        const long max_mpc_solves = this->get_parameter("max_mpc_solves").as_int();

        // ----- Read in state ----- //
        // // TODO: If statement is only here for when we remove sim
        // if (first_loop) {   // TODO: Remove when I go back to sim
        //     // TODO: Remove
        //     q = q_ic_;
        //     v = v_ic_;
        //     first_loop = false;
        // } else {
            {
                // Get the mutex to protect the states
                std::lock_guard<std::mutex> lock(est_state_mut_);

                // Create current state
                q = q_;
                v = v_;
            }
        // }
        q.segment<4>(3).normalize();
        if (std::abs(q.segment<4>(3).norm() - 1) > 1e-8) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "q: " << q.transpose());
            throw std::runtime_error("quat has zero norm!");
        }

        // ----- No Reference ----- //
        if (!fixed_target_ || controller_target_) {
            UpdateMpcTargets(q);
            mpc_->SetConfigTarget(q_target_.value());
            mpc_->SetVelTarget(v_target_.value());
        }

        // Reference generation
        {
            std::lock_guard<std::mutex> lock(polytope_mutex_);
            mpc_->UpdateContactSchedule(contact_schedule_);
            std::map<std::string, std::vector<torc::mpc::vector3_t>> contact_foot_pos;
            const auto [q_ref, v_ref] = ref_gen_->GenerateReference(q, v, q_target_.value(), v_target_.value(), mpc_->GetSwingTrajectory(),
                mpc_settings_->hip_offsets, contact_schedule_, contact_foot_pos);

            mpc_->SetForwardKinematicsTarget(contact_foot_pos);
        }

        double mpc_start_time = this->now().seconds();
        if (mpc_->GetSolveCounter() < max_mpc_solves) {
            // ---- Solve MPC ----- //
            // double time = this->now().seconds();        // NOTE: The quad uses the time here
            mpc_->Compute(mpc_start_time - time_offset_, q, v, traj_mpc_);
            double time = this->now().seconds();    // NOTE: The humanoid uses the time here
            {
                // Get the traj mutex to protect it
                std::lock_guard<std::mutex> lock(traj_out_mut_);
                traj_out_ = traj_mpc_;

                // Assign start time too
                traj_start_time_ = time;
            }
        }
        timer.Toc();

        torc::utils::TORCTimer prep_timer;
        prep_timer.Tic();
        // Part of the preperation phase
        mpc_->LogMPCCompute(mpc_start_time - time_offset_, q, v);
        prep_timer.Toc();

        return {timer.Duration<std::chrono::microseconds>().count()/1000.0, prep_timer.Duration<std::chrono::microseconds>().count()/1000.0};
    }

    obelisk_control_msgs::msg::PDFeedForward MpcController::ComputeControl() {
        // This callback does not need any access to the state as it outputs PD
        //  setpoints from the trajectory.

        obelisk_control_msgs::msg::PDFeedForward msg;

        if (GetState() != NoOutput) {
            RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Publishing first control.");
            
            vectorx_t q, v, tau, F(3*mpc_settings_->num_contact_locations);
            std::vector<bool> in_contact(mpc_settings_->num_contact_locations);

            if (GetState() == Mpc) {
                double time_into_traj = 0;
                // Get the traj mutex to protect it
                {
                    std::lock_guard<std::mutex> lock(traj_out_mut_);
                    
                    if (traj_start_time_ >= 0) {
                        double time = this->get_clock()->now().seconds();
                        time_into_traj = time - traj_start_time_;
                    }
                    
                    traj_out_.GetConfigInterp(time_into_traj, q);
                    traj_out_.GetVelocityInterp(time_into_traj, v);
                    traj_out_.GetTorqueInterp(time_into_traj, tau);

                    // TODO: Remove after debugging
                    // tau = traj_out_.GetTau(0);

                    for (int i = 0; i < mpc_settings_->num_contact_locations; i++) {
                        vector3_t f_temp;
                        traj_out_.GetForceInterp(time_into_traj, mpc_settings_->contact_frames[i], f_temp);
                        F.segment<3>(3*i) = f_temp;

                        // in_contact[i] = traj_out_.GetInContactInterp(time_into_traj, mpc_settings_->contact_frames[i]);
                    }
                    
                    // This is a safety in case the interpolation time is too large
                    if (q.size() == 0) {
                        // throw std::runtime_error("After traj!"); // TODO: remove
                        RCLCPP_WARN_STREAM(this->get_logger(), "Trajectory interpolation time is after the trajectory ends!");
                        q = traj_out_.GetConfiguration(traj_out_.GetNumNodes()-1);
                        RCLCPP_WARN_STREAM(this->get_logger(), "q is: " << q.transpose());
                    }

                    if (v.size() == 0) {
                        v = traj_out_.GetVelocity(traj_out_.GetNumNodes()-1);
                    }

                    if (tau.size() == 0) {
                        tau = traj_out_.GetTau(traj_out_.GetNumNodes()-1);

                        for (int i = 0; i < mpc_settings_->num_contact_locations; i++) {
                            vector3_t f_temp = traj_out_.GetForce(traj_out_.GetNumNodes() - 1, mpc_settings_->contact_frames[i]);
                            F.segment<3>(3*i) = f_temp;
                        }
                    }
                }

                // // Get contact status
                // {
                //     std::lock_guard<std::mutex> lock(polytope_mutex_);
                //     for (int i = 0; i < mpc_settings_->num_contact_locations; i++) {
                //         in_contact[i] = contact_schedule_.InContact(mpc_settings_->contact_frames[i], time_into_traj);
                //     }
                // }
            } else if (ctrl_state_ == SeekInitialCond) {
                q = q_ic_;
                v = v_ic_;
                tau = vectorx_t::Zero(mpc_model_->GetNumInputs());
                F = vectorx_t::Zero(mpc_settings_->contact_frames.size()*3);
                F(2) = 80;
                F(5) = 80;
                F(8) = 80;
                F(11) = 80;

                for (int i = 0; i < in_contact.size(); i++) {
                    in_contact[i] = true;
                }
            } else if (ctrl_state_ == HoldInitialCond) {
                q = q_ic_;
                v = v_ic_;
                tau = vectorx_t::Zero(mpc_model_->GetNumInputs());
                F = vectorx_t::Zero(mpc_settings_->contact_frames.size()*3);

                for (int i = 0; i < in_contact.size(); i++) {
                    in_contact[i] = true;
                }
            } else {
                throw std::runtime_error("Not a valid state!");
            }

            // TODO: Remove!
            for (int i = 0; i < in_contact.size(); i++) {
                in_contact[i] = true;
            }

            // TODO: Move the log to after the publish call
            // Log
            static unsigned long log_counter = 0;
            if (log_counter % 5 == 0) {
                LogCurrentControl(q, v, tau, F);
            }
            log_counter++;

            // TODO: Put back in when using the WBC
            if (recieved_first_state_ && GetState() == Mpc) {
                // Use WBC
                // Get current state
                vectorx_t q_est, v_est;
                {
                    std::lock_guard<std::mutex> lock(est_state_mut_);
                    q_est = q_;
                    v_est = v_;
                }

                // // TODO: Remove
                q = q_ic_; //q_target_.value()[0];
                v = v_ic_; //v_target_.value()[0];
                F.setZero();
                tau.setZero();

                for (int i = 0; i < in_contact.size(); i++) {
                    in_contact[i] = true;
                } 
                tau = wbc_controller_->ComputeControl(q_est, v_est, q, v, tau, F, in_contact);     
            }

            // if (GetState() != Mpc) {
            //     tau.setZero();
            // }
            
            // // TODO: Remove
            // q = q_ic_;
            // v = v_ic_;
            // tau = vectorx_t::Zero(mpc_model_->GetNumInputs());
            // tau = vectorx_t::Ones(mpc_model_->GetNumInputs());

            // Only actuate legs
            // q.tail(8) = q_ic_.tail(8);
            // v.tail(8) = v_ic_.tail(8);
            // tau.tail(8).setZero();

            // TODO: Only use this when the mujoco model does not match!
            // Check if we need to insert other elements into the targets
            // TODO: Make this work for both MPC and WBC sizes!
            std::vector<double> q_vec(q.data(), q.data() + q.size());
            std::vector<double> v_vec(v.data(), v.data() + v.size());
            std::vector<double> tau_vec(tau.data(), tau.data() + tau.size());

            for (int i = 0; i < mpc_skipped_joint_indexes_.size(); i++) {
                // TODO: Compute angle so that ankle is parallel to the ground
                q_vec.insert(q_vec.begin() + FLOATING_POS_SIZE + mpc_skipped_joint_indexes_[i], mpc_skipped_joint_vals_[i]);
                v_vec.insert(v_vec.begin() + FLOATING_VEL_SIZE + mpc_skipped_joint_indexes_[i], 0);
                tau_vec.insert(tau_vec.begin() + mpc_skipped_joint_indexes_[i], 0);
            }

            Eigen::Map<Eigen::VectorXd> q_map(q_vec.data(), q_vec.size());
            Eigen::Map<Eigen::VectorXd> v_map(v_vec.data(), v_vec.size());
            Eigen::Map<Eigen::VectorXd> tau_map(tau_vec.data(), tau_vec.size());

            q = q_map;
            v = v_map;
            tau = tau_map;
            
            // Make the message
            vectorx_t u_mujoco = ConvertControlToMujocoU(q.tail(mpc_model_->GetNumInputs() + mpc_skipped_joint_indexes_.size()),
                v.tail(mpc_model_->GetNumInputs() + mpc_skipped_joint_indexes_.size()), tau);

            ConvertEigenToStd(u_mujoco, msg.u_mujoco);
            ConvertEigenToStd(q.tail(mpc_model_->GetNumInputs() + mpc_skipped_joint_indexes_.size()), msg.pos_target);
            ConvertEigenToStd(v.tail(mpc_model_->GetNumInputs() + mpc_skipped_joint_indexes_.size()), msg.vel_target);
            ConvertEigenToStd(tau, msg.feed_forward);
            
            if (msg.u_mujoco.size() != 3*(mpc_model_->GetNumInputs() + mpc_skipped_joint_indexes_.size())) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Message's u_mujoco is incorrectly sized. Size: " << msg.u_mujoco.size());
                RCLCPP_ERROR_STREAM(this->get_logger(), "Expected size: " << 3*(mpc_model_->GetNumInputs() + mpc_skipped_joint_indexes_.size()));
            }

            if (msg.feed_forward.size() != control_joint_names_.size()) {
                throw std::runtime_error("Invalid feedforward size/control joint name size!");
            }
            if (msg.pos_target.size() != kp_.size()) {
                throw std::runtime_error("Position target size does not match kp size!");
            }
            if (msg.pos_target.size() != kd_.size()) {
                throw std::runtime_error("Velocity target size does not match kd size!");
            }

            msg.kp = kp_;
            msg.kd = kd_;

            if (ctrl_state_ == SeekInitialCond) {
                // When going to the initial condition, use smaller gains
                for (int i = 0; i < msg.kp.size(); i++) {
                    msg.kp[i] *= 0.1;
                    msg.kd[i] *= 0.1;
                }
            } else if (ctrl_state_ == Mpc) {
                // NOTE: This is only for use with the WBC
                // When going to the initial condition, use smaller gains
                // for (int i = 0; i < msg.kp.size(); i++) {
                //     msg.kp[i] *= 0;
                //     msg.kd[i] *= 0;
                // }
            }

            msg.joint_names = control_joint_names_;

            // TODO: Use only when the mujoco model doesn't match the MPC
            // // Make the message
            // vectorx_t u_mujoco = ConvertControlToMujocoU(q.tail(model_->GetNumInputs()),
            //     v.tail(model_->GetNumInputs()), tau);

            // ConvertEigenToStd(u_mujoco, msg.u_mujoco);
            // ConvertEigenToStd(q.tail(model_->GetNumInputs()), msg.pos_target);
            // ConvertEigenToStd(v.tail(model_->GetNumInputs()), msg.vel_target);
            // ConvertEigenToStd(tau, msg.feed_forward);
            
            // if (msg.u_mujoco.size() != 3*model_->GetNumInputs()) {
            //     RCLCPP_ERROR_STREAM(this->get_logger(), "Message's u_mujoco is incorrectly sized. Size: " << msg.u_mujoco.size());
            // }

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

    void MpcController::LogCurrentControl(const vectorx_t& q_control, const vectorx_t& v_control, const vectorx_t& tau, const vectorx_t& force) {
        log_file_ << this->now().seconds() - time_offset_ << ",";
        LogEigenVec(q_control);
        LogEigenVec(v_control);
        LogEigenVec(tau);
        LogEigenVec(force);
        log_file_ << std::endl;
    }

    void MpcController::LogEigenVec(const vectorx_t& x) {
        for (int i = 0; i < x.size(); i++) {
            log_file_ << x(i) << ",";
        }
    }

    void MpcController::TransitionState(const ControllerState& new_state) {
        std::lock_guard<std::mutex> lock(ctrl_state_mut_);

        ctrl_state_ = new_state;
        std::string new_state_str = GetStateString(new_state);

        RCLCPP_INFO_STREAM(this->get_logger(), "[MPC] Transitioning to state: " << new_state_str);
    }

    MpcController::ControllerState MpcController::GetState() {
        std::lock_guard<std::mutex> lock(ctrl_state_mut_);
        return ctrl_state_;
    }

    void MpcController::UpdateMpcTargets(const vectorx_t& q) {
        std::lock_guard<std::mutex> lock(target_state_mut_);

        const std::vector<double>& dt_vec = mpc_settings_->dt;

        static torc::mpc::vector3_t q_base_target = q.head<3>();
        static torc::mpc::vector4_t q_base_quat_target = q.segment<4>(3);

        q_base_target(0) = q(0);

        if (v_target_.value()[0].head<2>().norm() > 0.01) { // I don't love it but its here for the drift
            q_base_target(1) = q(1);
            q_base_quat_target = q.segment<4>(3);   // TODO: Play with this a bit
        }

        q_base_target(2) = z_target_;

        q_target_.value()[0].head<3>() = q_base_target;
        q_target_.value()[0].segment<4>(3) = q_base_quat_target;

        const quat_t q_quat(q.segment<QUAT_VARS>(POS_VARS));
        vector3_t euler_angles = q_quat.toRotationMatrix().eulerAngles(2, 1, 0);

        // quat_t yaw_quaternion(Eigen::AngleAxisd(euler_angles[2], Eigen::Vector3d::UnitZ()));
        quat_t yaw_quaternion(q_quat.w(), 0, 0, q_quat.z()); 

        q_target_.value()[0](3) = yaw_quaternion.x();
        q_target_.value()[0](4) = yaw_quaternion.y();
        q_target_.value()[0](5) = yaw_quaternion.z();
        q_target_.value()[0](6) = yaw_quaternion.w();

        for (int i = 1; i < q_target_->GetNumNodes(); i++) {
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
        case HoldInitialCond:
            return "Hold Initial Condition";
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
        int num_nom_footholds = 0;
        int num_proj_footholds = 0;
        for (const auto& [frame, points] : nom_footholds_) {
            num_nom_footholds += points.size();
        }
        for (const auto& [frame, points] : projected_footholds_) {
            num_proj_footholds += points.size();
        }

        visualization_msgs::msg::MarkerArray msg;
        msg.markers.resize(viz_frames.size() + force_frames_.size() + num_nom_footholds + num_proj_footholds);
        std::cerr << "marker size: " << msg.markers.size() << std::endl;
        std::cerr << "num nom footholds: " << num_nom_footholds << std::endl;
        std::cerr << "num proj footholds: " << num_proj_footholds << std::endl;
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
                    std::lock_guard<std::mutex> lock(traj_out_mut_);
                    double time = this->get_clock()->now().seconds();
                    double time_into_traj = time - traj_start_time_;
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

        int i = viz_frames.size() + force_frames_.size();

        // Display nominal and projected footholds
        for (const auto& [frame, points] : nom_footholds_) {
            for (int j = 0; j < points.size(); j++) {
                msg.markers[i].type = visualization_msgs::msg::Marker::SPHERE;
                                msg.markers[i].header.frame_id = "world";
                msg.markers[i].header.stamp = this->now();
                msg.markers[i].ns = "nom_foothold";
                msg.markers[i].id = i;
                msg.markers[i].action = visualization_msgs::msg::Marker::MODIFY;

                msg.markers[i].scale.x = 0.04;
                msg.markers[i].scale.y = 0.04;
                msg.markers[i].scale.z = 0.04;

                msg.markers[i].pose.position.x = points[j][0];
                msg.markers[i].pose.position.y = points[j][1];
                msg.markers[i].pose.position.z = 0;

                msg.markers[i].color.a = 1.0;
                msg.markers[i].color.r = 0.431;
                msg.markers[i].color.g = 0.498;
                msg.markers[i].color.b = 0.50196;

                i++;
            }
        }

        for (const auto& [frame, points] : projected_footholds_) {
            for (int j = 0; j < points.size(); j++) {

                msg.markers[i].type = visualization_msgs::msg::Marker::SPHERE;
                                msg.markers[i].header.frame_id = "world";
                msg.markers[i].header.stamp = this->now();
                msg.markers[i].ns = "nom_foothold";
                msg.markers[i].id = i;
                msg.markers[i].action = visualization_msgs::msg::Marker::MODIFY;

                msg.markers[i].scale.x = 0.04;
                msg.markers[i].scale.y = 0.04;
                msg.markers[i].scale.z = 0.04;

                msg.markers[i].pose.position.x = points[j][0];
                msg.markers[i].pose.position.y = points[j][1];
                msg.markers[i].pose.position.z = 0;

                msg.markers[i].color.a = 1.0;
                msg.markers[i].color.r = 1.0;
                msg.markers[i].color.g = 0.75;
                msg.markers[i].color.b = 0;

                i++;
            }
        }


        std::lock_guard<std::mutex> lock(polytope_mutex_);

        int num_polytope_markers = 0;
        for (const auto& frame : viz_polytope_frames_) {
            num_polytope_markers += contact_schedule_.GetNumContacts(frame);
        }
        
        msg.markers.resize(viz_frames.size() + force_frames_.size() + num_nom_footholds + num_proj_footholds + num_polytope_markers);

        // TODO: There is a bug in this block of code for the quad
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
        static int robot_type = this->get_parameter("robot_type").as_int();
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

        if (robot_type == 0) {
            // ---------- Go2 ---------- //
            // TODO: Consider putting back
            // if (traj_start_time_ < 0) {
            //     traj_start_time_ = this->get_clock()->now().seconds();
            // }
            obelisk_estimator_msgs::msg::EstimatedState msg;

            vectorx_t q = vectorx_t::Zero(mpc_model_->GetConfigDim());
            vectorx_t v = vectorx_t::Zero(mpc_model_->GetVelDim());
            double time = this->get_clock()->now().seconds();
            {
                std::lock_guard<std::mutex> lock(traj_out_mut_);
                double time_into_traj = time - traj_start_time_;

                // RCLCPP_INFO_STREAM(this->get_logger(), "Time into traj: " << time_into_traj);
                q = vectorx_t::Zero(model_->GetConfigDim());
                v = vectorx_t::Zero(model_->GetVelDim());
                if (GetState() == Mpc) {
                    traj_out_.GetConfigInterp(time_into_traj, q);
                    traj_out_.GetVelocityInterp(time_into_traj, v);
                } else {
                    q = q_ic_;
                    v = v_ic_;
                }
            }

            // traj_out_.GetConfigInterp(0.01, q);
            msg.base_link_name = "torso";
            vectorx_t q_head = q.head<FLOATING_POS_SIZE>();
            vectorx_t q_tail = q.tail(model_->GetNumInputs());
            msg.q_base = torc::utils::EigenToStdVector(q_head);
            msg.q_joints.resize(model_->GetNumInputs());
            msg.v_joints.resize(model_->GetNumInputs());

            msg.joint_names.resize(q_tail.size());
            msg.joint_names[0] = "FL_hip_joint";
            msg.joint_names[1] = "FR_hip_joint";
            msg.joint_names[2] = "RL_hip_joint";
            msg.joint_names[3] = "RR_hip_joint";
            msg.joint_names[4] = "FL_thigh_joint";
            msg.joint_names[5] = "FR_thigh_joint";
            msg.joint_names[6] = "RL_thigh_joint";
            msg.joint_names[7] = "RR_thigh_joint";
            msg.joint_names[8] = "FL_calf_joint";
            msg.joint_names[9] = "FR_calf_joint";
            msg.joint_names[10] = "RL_calf_joint";
            msg.joint_names[11] = "RR_calf_joint";

            for (int i = 0; i < msg.joint_names.size(); i++) {
                const auto idx = model_->GetJointID(msg.joint_names[i]);
                if (idx.has_value()) {
                    msg.q_joints[i] = q(5 + idx.value());
                    if (4 + idx.value() > v.size()) {
                        RCLCPP_ERROR_STREAM(this->get_logger(), "v idx out of bounds!");
                    } else {
                        msg.v_joints[i] = v(4 + idx.value());
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

            // RCLCPP_ERROR_STREAM(this->get_logger(), "Publishing state viz");
            // std::cerr << "Here " << std::endl;

            // if (!sim_ready_) {
            this->GetPublisher<obelisk_estimator_msgs::msg::EstimatedState>("state_viz_pub")->publish(msg);
            // }
        } else if (robot_type == 1) {
        // ---------- G1 ---------- //
        // static int msg_counter = 0;
        // if (msg_counter < 10) {
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
                // TODO: Put back
                traj_out_.GetConfigInterp(time_into_traj, q);
                traj_out_.GetVelocityInterp(time_into_traj, v);

                // TODO: Remove after debugging
                // q = traj_out_.GetConfiguration(1);
                // v = traj_out_.GetVelocity(1);

                q.segment<4>(3).normalize();
                if (std::abs(q.segment<4>(3).norm() - 1) > 1e-8) {
                    // RCLCPP_ERROR_STREAM(this->get_logger(), "q: " << q.transpose());
                    // RCLCPP_ERROR_STREAM(this->get_logger(), "Setting the quaternion because it has 0 norm!");
                    // throw std::runtime_error("[debug viz] quat has zero norm!");
                }
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
        // }
        // msg_counter++;
        } else {
            throw std::runtime_error("Invalid robot type for PublishTrajStateViz!");
        }
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

        // TODO: Put back
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

     void MpcController::ContactPolytopeCallback(const sample_contact_msgs::msg::ContactPolytopeArray& msg) {
        // Extract the polytopes
        std::vector<torc::mpc::ContactInfo> polytopes;

        // Update the step planner
        for (int i = 0; i < msg.polytopes.size(); i++) {
            std::vector<double> a_mat = msg.polytopes[i].a_mat;

            Eigen::Map<matrixx_t> A(a_mat.data(), 2, 2);
            Eigen::Vector4d b(msg.polytopes[i].b_vec.data());

            polytopes.emplace_back(A, b);
        }

        std::lock_guard<std::mutex> lock(polytope_mutex_);
        step_planner_->UpdateContactPolytopes(polytopes);

        recieved_polytope_ = true;
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

        if (msg.buttons[X] && (this->now() - last_X_press).seconds() > 9e-1) {
            if (GetState() == SeekInitialCond) {
                TransitionState(HoldInitialCond);
            } else if (GetState() == HoldInitialCond) {
                TransitionState(Mpc);
            } else if (GetState() == NoOutput) {
                TransitionState(SeekInitialCond);
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
                v_target_.value()[i](1) = msg.axes[LEFT_JOY_HORZ] * 0; //0.1;
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
