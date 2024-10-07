#include <vector>
#include <pthread.h>

#include "obelisk_node.h"
#include "obelisk_ros_utils.h"

// TODO: remove
#include "obelisk_estimator.h"

// ------ Mujoco Debug ----- //
#include <GLFW/glfw3.h>
// ------ Mujoco Debug ----- //

#include "sample_contact_walking/achilles_mpc.h"

// TODO:
//  - Allow user to set the initial condition in the yaml
//  - Check for paths first relative to $SAMPLE_WALKING_ROOT then as a global path
//  - Add ROS diagonstic messages: https://docs.foxglove.dev/docs/visualization/panels/diagnostics#diagnosticarray
//  - Try plotting just the first MPC solve and see what the trajectory is. Why does it swing its leg really far out
//  - Add whole stack pausing
//  - Sample planner causes an error eventually
//  - Try tracking a single MPC traj with the floating base
//  - Try getting it to stand with just mpc
//  - Try smoothing the velocities with the cost funciton
//  - Continue trying the MPC like a WBC. Determine why it stands up with torques only.
//  - Try without a torque feedforward with the whole MPC problem
//  - Why does it not even look like its trying to catch itself with torques only?
//  - Fix the holonomic constraints


namespace achilles
{
    MpcController::MpcController(const std::string& name) 
    : obelisk::ObeliskController<obelisk_control_msgs::msg::PDFeedForward, obelisk_estimator_msgs::msg::EstimatedState>(name), 
        recieved_first_state_(false), first_mpc_computed_(false), ctrl_state_(NoOutput), traj_start_time_(0), sim_ready_(false) {

        mujoco_sim_instance_ = this;

        // For now model the feet as point contacts
        contact_state_.contacts.emplace("left_ankle_pitch", torc::models::Contact(torc::models::PointContact, false));
        contact_state_.contacts.emplace("right_ankle_pitch", torc::models::Contact(torc::models::PointContact, false));

        this->declare_parameter<double>("mpc_loop_period_sec", 0.01);
        this->declare_parameter<long>("max_mpc_solves", -1);

        // --- Debug Publisher and Timer --- //
        this->RegisterObkTimer("state_viz_timer_setting", "state_viz_timer", std::bind(&MpcController::PublishTrajStateViz, this));
        this->RegisterObkPublisher<obelisk_estimator_msgs::msg::EstimatedState>("state_viz_pub_setting", "state_viz_pub");

        // --- Debug Force Publisher --- //
        this->RegisterObkPublisher<obelisk_sensor_msgs::msg::ObkForceSensor>("force_pub_setting", "force_pub");

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
        mpc_ = std::make_shared<torc::mpc::FullOrderMpc>("achilles_mpc_obk", this->get_parameter("params_path").as_string(), urdf_path);
        RCLCPP_INFO_STREAM(this->get_logger(), "MPC Created...");

        mpc_->Configure();
        RCLCPP_INFO_STREAM(this->get_logger(), "MPC Configured...");

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


        contact_schedule_.SetFrames(mpc_->GetContactFrames());

        if (right_foot_first_) {
            contact_schedule_.InsertSwingByDuration(right_frames_[0], first_swing_time_, swing_time_);
            contact_schedule_.InsertSwingByDuration(right_frames_[1], first_swing_time_, swing_time_);
            next_right_insertion_time_ = first_swing_time_ + 2*swing_time_;
            next_left_insertion_time_ = first_swing_time_ + swing_time_;
        } else {
            contact_schedule_.InsertSwingByDuration(left_frames_[0], first_swing_time_, swing_time_);
            contact_schedule_.InsertSwingByDuration(left_frames_[1], first_swing_time_, swing_time_);
            next_left_insertion_time_ = first_swing_time_ + 2*swing_time_;
            next_right_insertion_time_ = first_swing_time_ + swing_time_;
        }

        AddPeriodicContacts();

        this->declare_parameter<double>("default_swing_height", 0.1);
        this->declare_parameter<double>("default_stand_foot_height", 0.0);
        this->declare_parameter<double>("apex_time", 0.5);
        mpc_->UpdateContactScheduleAndSwingTraj(contact_schedule_,
            this->get_parameter("default_swing_height").as_double(),
            this->get_parameter("default_stand_foot_height").as_double(),
            this->get_parameter("apex_time").as_double());

        mpc_->PrintContactSchedule();
        for (const auto& frame : mpc_->GetContactFrames()) {
            mpc_->PrintSwingTraj(frame);
        }

        // Setup q and v targets
        this->declare_parameter<std::vector<double>>("target_config");
        this->declare_parameter<std::vector<double>>("target_vel");
        std::vector<double> q_targ_temp, v_targ_temp;

        this->get_parameter("target_config", q_targ_temp);
        this->get_parameter("target_vel", v_targ_temp);

        vectorx_t q_target_eig = torc::utils::StdToEigenVector(q_targ_temp);
        vectorx_t v_target_eig = torc::utils::StdToEigenVector(v_targ_temp);
        q_target_ = torc::mpc::SimpleTrajectory(model_->GetConfigDim(), mpc_->GetNumNodes());
        v_target_ = torc::mpc::SimpleTrajectory(model_->GetVelDim(), mpc_->GetNumNodes());
        q_target_->SetAllData(q_target_eig);
        v_target_->SetAllData(v_target_eig);

        mpc_->SetConfigTarget(q_target_.value());
        mpc_->SetVelTarget(v_target_.value());

        // Set initial conditions
        this->declare_parameter<std::vector<double>>("mpc_ic_config");
        this->declare_parameter<std::vector<double>>("mpc_ic_vel");

        std::vector<double> q_ic_temp, v_ic_temp;
        q_ic_temp = this->get_parameter("mpc_ic_config").as_double_array();
        v_ic_temp = this->get_parameter("mpc_ic_vel").as_double_array();

        q_ic_ = torc::utils::StdToEigenVector(q_ic_temp);
        v_ic_ = torc::utils::StdToEigenVector(v_ic_temp);

        std::cout << "q target: " << q_target_.value()[0].transpose() << std::endl;
        std::cout << "v target: " << v_target_.value()[0].transpose() << std::endl;
        std::cout << "q ic: " << q_ic_.transpose() << std::endl;
        std::cout << "v ic: " << v_ic_.transpose() << std::endl;


        this->declare_parameter<bool>("fixed_target", true);
        this->get_parameter("fixed_target", fixed_target_);

        // Create default trajectory
        traj_out_.UpdateSizes(model_->GetConfigDim(), model_->GetVelDim(), model_->GetNumInputs(), mpc_->GetContactFrames(), mpc_->GetNumNodes());
        traj_out_.SetDefault(q_ic_);
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

        mpc_->ComputeNLP(q_ic_, v_ic_, traj_mpc_);
        mpc_->PrintStatistics();
        traj_out_ = traj_mpc_;
        traj_start_time_ = -1;

        // Compute an initial MPC at the initial condition

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

        // Spin up MPC thread
        mpc_thread_ = std::thread(&MpcController::MpcThread, this);
        // sched_param sch;
        // int policy;
        // pthread_getschedparam(mpc_thread_.native_handle(), &policy, &sch);
        // RCLCPP_WARN_STREAM(this->get_logger(), "Sched prio: " << sch.sched_priority);
        // sch.sched_priority = 20;
        // if (int error = pthread_setschedparam(mpc_thread_.native_handle(), SCHED_FIFO, &sch)) {
        //     RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to set MPC thread prio!");
        //     if (error == ESRCH) {
        //         RCLCPP_ERROR_STREAM(this->get_logger(), "Error: ESRCH");
        //     } else if (error == EINVAL) {
        //         RCLCPP_ERROR_STREAM(this->get_logger(), "Error: EINVAL");
        //     } else if (error == EPERM) {
        //         RCLCPP_ERROR_STREAM(this->get_logger(), "Error: EPERM");
        //     } else {
        //         RCLCPP_ERROR_STREAM(this->get_logger(), "Error: not found!");
        //     }
        // }

        // Sample planning
        this->declare_parameter("simulation_samples", 50);
        this->declare_parameter<std::string>("xml_path");

        std::string xml_path_str = this->get_parameter("xml_path").as_string();
        std::filesystem::path xml_path(xml_path_str);

        cem_ = std::make_unique<torc::sample::CrossEntropy>(xml_path, this->get_parameter("simulation_samples").as_int(),
            this->get_parameter("params_path").as_string(), mpc_);

        this->declare_parameter("sample_loop_period_sec", 0.05);
    }

    void MpcController::UpdateXHat(const obelisk_estimator_msgs::msg::EstimatedState& msg) {
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


        if (q_.size() != model_->GetConfigDim()) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "received q does not match the size of the model");
        }

        if (v_.size() != model_->GetVelDim()) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "received v does not match the size of the model");
        }

        if (!recieved_first_state_ && q_.size() == model_->GetConfigDim() && v_.size() == model_->GetVelDim() && q_.segment<QUAT_VARS>(POS_VARS).norm() > 0.99) {
            recieved_first_state_ = true;
        }
    }

    // ------ Mujoco Debug ------ //
    // -------------------------------------- //
    // ----------- GLFW Callbacks ----------- //
    // -------------------------------------- //
    void MpcController::KeyboardCallback(GLFWwindow* window, int key, int scancode, int act, int mods) {
        if (mujoco_sim_instance_) {
            mujoco_sim_instance_->HandleKeyboard(window, key, scancode, act, mods);
        } else {
            throw std::runtime_error("No active mujoco sim instance while the GLFW window is running!");
        }
    }

    void MpcController::HandleKeyboard(__attribute__((unused)) GLFWwindow* window, int key, __attribute__((unused)) int scancode,
                        int act, __attribute__((unused)) int mods) {
        // backspace: reset simulation
        if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
            mj_resetData(mj_model_, data_);
            mj_forward(mj_model_, data_);
        }

        if (act == GLFW_PRESS && key == GLFW_KEY_SPACE) {
            pause = !pause;
        }
    }

    void MpcController::MouseButtonCallback(GLFWwindow* window, int button, int act, int mods) {
        if (mujoco_sim_instance_) {
                mujoco_sim_instance_->HandleMouseButton(window, button, act, mods);
        } else {
            throw std::runtime_error("No active mujoco sim instance while the GLFW window is running!");
        }
    }

    void MpcController::HandleMouseButton(GLFWwindow* window, __attribute__((unused)) int button, __attribute__((unused)) int act,
                            __attribute__((unused)) int mods) {
        // update button state
        button_left   = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
        button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
        button_right  = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

        // update mouse position
        glfwGetCursorPos(window, &lastx, &lasty);
    }

    void MpcController::MouseMoveCallback(GLFWwindow* window, double xpos, double ypos) {
        if (mujoco_sim_instance_) {
            mujoco_sim_instance_->HandleMouseMove(window, xpos, ypos);
        } else {
            throw std::runtime_error("No active mujoco sim instance while the GLFW window is running!");
        }
    }
    void MpcController::HandleMouseMove(GLFWwindow* window, double xpos, double ypos) {
        // no buttons down: nothing to do
        if (!button_left && !button_middle && !button_right) {
            return;
        }

        // compute mouse displacement, save
        double dx = xpos - lastx;
        double dy = ypos - lasty;
        lastx     = xpos;
        lasty     = ypos;

        // get current window size
        int width, height;
        glfwGetWindowSize(window, &width, &height);

        // get shift key state
        bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                            glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

        // determine action based on mouse button
        mjtMouse action;
        if (button_right) {
            action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
        } else if (button_left) {
            action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
        } else {
            action = mjMOUSE_ZOOM;
        }

        // move camera
        mjv_moveCamera(mj_model_, action, dx / height, dy / height, &scn, &cam);
    }

    void MpcController::ScrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
        if (mujoco_sim_instance_) {
            mujoco_sim_instance_->HandleScroll(window, xoffset, yoffset);
        } else {
            throw std::runtime_error("No active mujoco sim instance while the GLFW window is running!");
        }
    }

    void MpcController::HandleScroll(__attribute__((unused)) GLFWwindow* window, __attribute__((unused)) double xoffset,
                        double yoffset) {
        // emulate vertical mouse motion = 5% of window height
        mjv_moveCamera(mj_model_, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
    }
    // ------ Mujoco Debug ------ //

    // Running the MPC in its own the thread seems to make the timing more consistent and overall faster
    // If I still need more, I can try to adjust the thread prio
    // Experimentally, note that the faster I run it, the more consistent (and faster, up to a limit) it is
    void MpcController::MpcThread() {
        const long mpc_loop_rate_ns = this->get_parameter("mpc_loop_period_sec").as_double()*1e9;
        RCLCPP_INFO_STREAM(this->get_logger(), "MPC loop period set to: " << mpc_loop_rate_ns << "ns.");

        const long max_mpc_solves = this->get_parameter("max_mpc_solves").as_int();
        
        static bool first_loop = true;
        auto prev_time = this->now();

        // ------ Mujoco Debug ----- //
        // Init simulator
        // load the mujoco model
        char error[1000] = "Could not load binary model";
        std::string xml_path_str = this->get_parameter("xml_path").as_string();
        std::filesystem::path xml_path(xml_path_str);
        if (!std::filesystem::exists(std::filesystem::path(xml_path))) {
            throw std::invalid_argument("[Mujoco Debug] Mujoco XML path does not exist!");
        }

        mj_model_ = mj_loadXML(xml_path.c_str(), 0, error, 1000);
        if (!mj_model_) {
            throw std::runtime_error("[Mujoco Debug] Could not create Mujoco model!");
        }

        // Make the data for each sample
        data_ = mj_makeData(mj_model_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Found " << this->mj_model_->nkey << " Mujoco keyframes.");
        this->declare_parameter<std::string>("ic_keyframe", "ic");
        for (int i = 0; i < this->mj_model_->nkey; i++) {
            std::string potential_keyframe(this->mj_model_->names + this->mj_model_->name_keyadr[i]);
            if (potential_keyframe == this->get_parameter("ic_keyframe").as_string()) {
                RCLCPP_INFO_STREAM(this->get_logger(),
                                    "Setting initial condition to keyframe: " << potential_keyframe);
                mju_copy(this->data_->qpos, &this->mj_model_->key_qpos[i * this->mj_model_->nq], this->mj_model_->nq);
                mju_copy(this->data_->qvel, &this->mj_model_->key_qvel[i * this->mj_model_->nv], this->mj_model_->nv);
                this->data_->time = this->mj_model_->key_time[i];

                mju_copy(this->data_->ctrl, &this->mj_model_->key_ctrl[i * this->mj_model_->nu], this->mj_model_->nu);
            }
        }
        // Init rendering
        mujoco_sim_instance_ = this;
        // Create the window
        if (!glfwInit()) {
            throw std::runtime_error("Could not initialize GLFW");
        }

        window =
            glfwCreateWindow(WINDOW_WIDTH_DEFAULT, WINDOW_LENGTH_DEFAULT, "DEBUG Mujoco Simulation", NULL, NULL);

        if (!window) {
            throw std::runtime_error("Could not create the GLFW window!");
        }

        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);

        // initialize visualization data structures
        mjv_defaultFreeCamera(mj_model_, &cam);
        mjv_defaultOption(&opt);
        mjv_defaultScene(&scn);
        mjr_defaultContext(&con);

        // create scene and context
        mjv_makeScene(mj_model_, &scn, 2000);
        mjr_makeContext(mj_model_, &con, mjFONTSCALE_150);

        // install GLFW mouse and keyboard callbacks
        glfwSetKeyCallback(window, MpcController::KeyboardCallback);
        glfwSetCursorPosCallback(window, MpcController::MouseMoveCallback);
        glfwSetMouseButtonCallback(window, MpcController::MouseButtonCallback);
        glfwSetScrollCallback(window, MpcController::ScrollCallback);

        // ------ Mujoco Debug ----- //

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

                // TODO: Remove!
                // q = q_ic_;
                // q(2) += 0.02;
                // q.head<FLOATING_POS_SIZE>() = q_ic_.head<FLOATING_POS_SIZE>();
                // v.setZero();

                // RCLCPP_ERROR_STREAM(this->get_logger(), "q: " << q.transpose());
                // RCLCPP_ERROR_STREAM(this->get_logger(), "v: " << v.transpose());

                // TODO: remove
                // v.setZero();

                // If in NLP mode, then pause the timer and compute the NLP
                // Then restart the timer after the NLP is solved.
                // Change state to normal MPC
                // This only works if the robot can hold its position for the enough time for the computation

                // Shift the contact schedule
                auto current_time = this->now();
                double time_shift_sec = (current_time - prev_time).nanoseconds()/1e9;
                contact_schedule_.ShiftSwings(-time_shift_sec);    // TODO: Do I need a mutex on this later?
                next_left_insertion_time_ -= time_shift_sec;
                next_right_insertion_time_ -= time_shift_sec;
                prev_time = this->now();
                mpc_->UpdateContactScheduleAndSwingTraj(contact_schedule_,
                    this->get_parameter("default_swing_height").as_double(),
                    this->get_parameter("default_stand_foot_height").as_double(),
                    this->get_parameter("apex_time").as_double());

                AddPeriodicContacts();

                    // ----- Mujoco debug ----- //
                    // traj_out_.GetConfigInterp(mpc_loop_rate_ns*1e-9, q);
                    // traj_out_.GetVelocityInterp(mpc_loop_rate_ns*1e-9, v);
                    mju_copy(q.data(), data_->qpos, q.size());
                    // Change the quaternion convention
                    q(3) = data_->qpos[4];
                    q(4) = data_->qpos[5];
                    q(5) = data_->qpos[6];
                    q(6) = data_->qpos[3];
                    // RCLCPP_INFO_STREAM(this->get_logger(), "q: " << q.transpose());

                    mju_copy(v.data(), data_->qvel, v.size());

                    // TODO: What is the correct frame?
                    quat_t quat(q.segment<QUAT_VARS>(POS_VARS));
                    v.segment<3>(POS_VARS) = quat.toRotationMatrix()*v.segment<3>(POS_VARS);
                    // ----- Mujoco debug ----- //


                if (!fixed_target_) {
                    UpdateMpcTargets(q);
                    mpc_->SetConfigTarget(q_target_.value());
                    mpc_->SetVelTarget(v_target_.value());
                }

                if (max_mpc_solves < 0 || mpc_->GetTotalSolves() < max_mpc_solves) {
                    double time = this->get_clock()->now().seconds();
                    double delay_start_time = this->now().seconds() - traj_start_time_;
                    mpc_->Compute(q, v, traj_mpc_, delay_start_time);
                    {
                        // Get the traj mutex to protect it
                        std::lock_guard<std::mutex> lock(traj_out_mut_);
                        traj_out_ = traj_mpc_;

                        // Assign time time too
                        traj_start_time_ = time;
                    }

                    // ------ Mujoco Debug ----- //
                    sim_ready_ = true;
                    double traj_start_time_mj = data_->time;
                    long step_num = -1;
                    while (data_->time < traj_start_time_mj + mpc_loop_rate_ns*1e-9) {
                        std::lock_guard<std::mutex> lock(mj_data_mut_);
                        if (!pause) {
                            double time_into_traj = data_->time - traj_start_time_mj;
                            // Copy controls over
                            vectorx_t q, v, tau;
                            traj_out_.GetConfigInterp(time_into_traj, q);
                            traj_out_.GetVelocityInterp(time_into_traj, v);
                            traj_out_.GetTorqueInterp(time_into_traj, tau);

                            vectorx_t u_mujoco = ConvertControlToMujocoU(q.tail(model_->GetNumInputs()),
                                            v.tail(model_->GetNumInputs()), tau);
                            mju_copy(data_->ctrl, u_mujoco.data(), u_mujoco.size());

                            while (pause) {
                                std::this_thread::sleep_for(std::chrono::milliseconds(15));
                            }

                            // Step
                            mj_step(mj_model_, data_);
                            // RCLCPP_WARN_STREAM(this->get_logger(), "Stepping");


                            obelisk_control_msgs::msg::PDFeedForward msg;
                            ConvertEigenToStd(u_mujoco, msg.u_mujoco);
                            ConvertEigenToStd(q.tail(model_->GetNumInputs()), msg.pos_target);
                            ConvertEigenToStd(v.tail(model_->GetNumInputs()), msg.vel_target);
                            ConvertEigenToStd(tau, msg.feed_forward);
                            
                            if (msg.u_mujoco.size() != 54) {
                                RCLCPP_ERROR_STREAM(this->get_logger(), "Message's u_mujoco is incorrectly sized. Size: " << msg.u_mujoco.size());
                            }

                            // msg.header.stamp.sec = data_->time;
                            msg.header.stamp = this->now();

                            this->GetPublisher<obelisk_control_msgs::msg::PDFeedForward>(this->ctrl_key_)->publish(msg);

                            step_num++;

                            // TODO: Print velocities to debug holonomic constraint issues. Why do the feet want to slide forward?

                            // Publish current state
                            obelisk_estimator_msgs::msg::EstimatedState msg_est_state;
                            vectorx_t q_state = vectorx_t::Zero(q.size());
                            vectorx_t v_state = vectorx_t::Zero(v.size());

                            mju_copy(q_state.data(), data_->qpos, q.size());
                            // Change the quaternion convention
                            q_state(3) = data_->qpos[4];
                            q_state(4) = data_->qpos[5];
                            q_state(5) = data_->qpos[6];
                            q_state(6) = data_->qpos[3];

                            mju_copy(v_state.data(), data_->qvel, v.size());

                            // q_state = q;
                            // q_state(2) += 0.01;
                            // v_state = v;
                            // msg.header.stamp.sec = data_->time;
                
                            // traj_out_.GetConfigInterp(0.01, q);
                            msg_est_state.base_link_name = "torso";
                            vectorx_t q_head = q_state.head<FLOATING_POS_SIZE>();
                            vectorx_t q_tail = q_state.tail(model_->GetNumInputs());
                            msg_est_state.q_base = torc::utils::EigenToStdVector(q_head);
                            msg_est_state.q_joints = torc::utils::EigenToStdVector(q_tail);

                            msg_est_state.joint_names.resize(q_tail.size());
                            msg_est_state.joint_names[0] = "left_hip_yaw_joint";
                            msg_est_state.joint_names[1] = "left_hip_roll_joint";
                            msg_est_state.joint_names[2] = "left_hip_pitch_joint";
                            msg_est_state.joint_names[3] = "left_knee_pitch_joint";
                            msg_est_state.joint_names[4] = "left_ankle_pitch_joint";
                            msg_est_state.joint_names[5] = "left_shoulder_pitch_joint";
                            msg_est_state.joint_names[6] = "left_shoulder_roll_joint";
                            msg_est_state.joint_names[7] = "left_shoulder_yaw_joint";
                            msg_est_state.joint_names[8] = "left_elbow_pitch_joint";
                            msg_est_state.joint_names[9] = "right_hip_yaw_joint";
                            msg_est_state.joint_names[10] = "right_hip_roll_joint";
                            msg_est_state.joint_names[11] = "right_hip_pitch_joint";
                            msg_est_state.joint_names[12] = "right_knee_pitch_joint";
                            msg_est_state.joint_names[13] = "right_ankle_pitch_joint";
                            msg_est_state.joint_names[14] = "right_shoulder_pitch_joint";
                            msg_est_state.joint_names[15] = "right_shoulder_roll_joint";
                            msg_est_state.joint_names[16] = "right_shoulder_yaw_joint";
                            msg_est_state.joint_names[17] = "right_elbow_pitch_joint";

                            // traj_out_.GetVelocityInterp(0.01, v);
                            vectorx_t v_head = v_state.head<FLOATING_VEL_SIZE>();
                            vectorx_t v_tail = v_state.tail(model_->GetNumInputs());

                            // vectorx_t temp = vectorx_t::Zero(6);
                            msg_est_state.v_base = torc::utils::EigenToStdVector(v_head);
                            msg_est_state.v_joints = torc::utils::EigenToStdVector(v_tail);

                            // TODO: Put back!
                            msg_est_state.header.stamp = this->now();

                            this->GetPublisher<obelisk_estimator_msgs::msg::EstimatedState>("state_viz_pub")->publish(msg_est_state);
                        }

                        // Render
                        // This logic will only work with certain time step sizes
                        if (step_num % 40 == 0 || pause) {
                            if (!glfwWindowShouldClose(window)) {
                                // get framebuffer viewport
                                mjrRect viewport = {0, 0, 0, 0};
                                glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

                                float time = 0;

                                // update scene and render
                                time = data_->time;
                                mjv_updateScene(mj_model_, data_, &opt, NULL, &cam, mjCAT_ALL, &scn);
                                
                                mjr_render(viewport, &scn, &con);

                                // add time stamp in upper-left corner
                                char stamp[50];
                                sprintf_arr(stamp, "Time = %.3f", time);
                                mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, stamp, NULL, &con);

                                // swap OpenGL buffers (blocking call due to v-sync)
                                glfwSwapBuffers(window);

                                // process pending GUI events, call GLFW callbacks
                                glfwPollEvents();
                            }
                        }
                    }
                    
                    // ------ Mujoco Debug ----- //

                    // std::cout << "Left ankle pitch ic error: " << traj_mpc_.GetConfiguration(0)(11) - q(11) << std::endl;
                    // std::cout << "Norm IC config error: " << (traj_mpc_.GetConfiguration(0) - q).norm() << std::endl;
                    // std::cout << "Norm IC vel error: " << (traj_mpc_.GetVelocity(0) - v).norm() << std::endl;

                    // RCLCPP_INFO_STREAM(this->get_logger(), "Config IC Error: " << (traj_out_.GetConfiguration(0) - q).norm());
                    // RCLCPP_INFO_STREAM(this->get_logger(), "Vel IC Error: " << (traj_out_.GetVelocity(0) - v).norm());

                    // TODO: Put back
                    // if (GetState() != Mpc) {
                    //     TransitionState(Mpc);
                    // }
                } else {
                    static bool printed = false;
                    if (!printed) {
                        mpc_->PrintStatistics();
                        mpc_->PrintContactSchedule();
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
                while ((-(this->now() - start_time).nanoseconds() + mpc_loop_rate_ns) > 0) {}
            } else {
                RCLCPP_WARN_STREAM(this->get_logger(), "MPC computation took longer than loop rate allowed for. " << std::abs(time_left)*1e-6 << "ms over time.");
            }
        }
    }

    obelisk_control_msgs::msg::PDFeedForward MpcController::ComputeControl() {
        // This callback does not need any access to the state as it outputs PD
        //  setpoints from the trajectory.

        obelisk_control_msgs::msg::PDFeedForward msg;

        // For force debugging
        obelisk_sensor_msgs::msg::ObkForceSensor force_msg;

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

                    // TODO: Remove
                    // q = traj_out_.GetConfiguration(0);
                    // v = traj_out_.GetVelocity(0);
                    // q.head(v.size()) = traj_out_.GetVelocity(1);
                    // std::cout << "traj node 1 vel: " << traj_out_.GetVelocity(1).transpose() << std::endl;
                    // RCLCPP_INFO_STREAM(this->get_logger(), "traj v l knee diff: " << v(3) - traj_out_.GetVelocity(1)(3));
                    // tau = traj_out_.GetTau(1);

                    // TODO: Put back
                    traj_out_.GetConfigInterp(time_into_traj, q);
                    traj_out_.GetVelocityInterp(time_into_traj, v);
                    traj_out_.GetTorqueInterp(time_into_traj, tau);

                    // For force debugging
                    std::array<std::string, 4> frames = {"foot_front_right", "foot_rear_right", "foot_front_left", "foot_rear_left"};
                    for (const auto& frame : frames) {
                        geometry_msgs::msg::Vector3 force;
                        vector3_t force_traj;
                        traj_out_.GetForceInterp(time_into_traj, frame, force_traj);
                        force.x = force_traj(0);
                        force.y = force_traj(1);
                        force.z = force_traj(2);

                        force_msg.forces.emplace_back(force);
                    }

                    // TODO: Remove
                    // tau.head<5>().setZero();
                    // tau.tail(9).setZero();
                    // tau(5) = 0;
                    // tau(6) = 0;
                    // tau(7) = 0;
                    // tau(8) = 0;

                    // tau.setZero();

                    // std::vector<torc::models::ExternalForce<double>> f_ext;
                    // for (const auto& frame : mpc_->GetContactFrames()) {
                    //     vector3_t force;
                    //     traj_out_.GetForceInterp(time_into_traj, frame, force);
                    //     f_ext.emplace_back(frame, force);
                    // }

                    // const double fd_delta = 1e-8;
                    // vectorx_t v2;
                    // traj_out_.GetVelocityInterp(time_into_traj + fd_delta, v2);
                    // vectorx_t a = (v2 - v)/fd_delta;
                    // tau = model_->InverseDynamics(q, v, a, f_ext);
                    // tau = tau.tail(18);

                    // std::cout << "a: " << a.transpose() << std::endl;
                    // std::cout << "new id: " << tau.transpose() << std::endl;

                    // vectorx_t tau2;
                    // traj_out_.GetTorqueInterp(time_into_traj, tau2);
                    // std::cout << "traj: " << tau2.transpose() << std::endl;
                    // std::cout << "normed difference: " << (tau2 - tau).norm() << std::endl;

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
                    // ---
                }
            } else if (ctrl_state_ == SeekInitialCond) {
                // RCLCPP_WARN_STREAM(this->get_logger(), "Seeking IC");
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
            // TODO: Put back
            // this->GetPublisher<obelisk_control_msgs::msg::PDFeedForward>(this->ctrl_key_)->publish(msg);

            // Publish desired forces for debugging         
            force_msg.header.stamp = this->now();   
            this->GetPublisher<obelisk_sensor_msgs::msg::ObkForceSensor>("force_pub")->publish(force_msg);
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

    MpcController::ControllerState MpcController::GetState() {
        std::lock_guard<std::mutex> lock(ctrl_state_mut_);
        return ctrl_state_;
    }

    void MpcController::UpdateMpcTargets(const vectorx_t& q) {
        // For now, only update the desired x and y positions based on the x, y target vel

        const std::vector<double>& dt_vec = mpc_->GetDtVector();

        q_target_.value()[0](0) = q(0);
        q_target_.value()[0](1) = q(1);

        for (int i = 1; i < q_target_->GetNumNodes(); i++) {
            const quat_t quat(q_target_.value()[i-1].segment<QUAT_VARS>(POS_VARS));
            const matrix3_t R = quat.toRotationMatrix();
            const vector3_t v = R*v_target_.value()[i].head<POS_VARS>();

            q_target_.value()[i](0) = q_target_.value()[i - 1](0) + dt_vec[i-1]*v(0);
            q_target_.value()[i](1) = q_target_.value()[i - 1](1) + dt_vec[i-1]*v(1);
        }
        

        // In the future, we will want to be able to update the z height, and orientation
        // Also in the future we will get inputs from the joystick
    }

    void MpcController::PublishTrajViz(const torc::mpc::Trajectory& traj, const std::vector<std::string>& viz_frames) {
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

    void MpcController::PublishTrajStateViz() {
        std::lock_guard<std::mutex> lock(traj_out_mut_);

        if (traj_start_time_ < 0) {
            traj_start_time_ = this->get_clock()->now().seconds();
        }
        obelisk_estimator_msgs::msg::EstimatedState msg;

        double time = this->get_clock()->now().seconds();
        // TODO: Do I need to use nanoseconds?
        double time_into_traj = time - traj_start_time_;
        // double time_into_traj = 0;

        // RCLCPP_INFO_STREAM(this->get_logger(), "Time into traj: " << time_into_traj);
        vectorx_t q = vectorx_t::Zero(model_->GetConfigDim());
        vectorx_t v = vectorx_t::Zero(model_->GetVelDim());
        traj_out_.GetConfigInterp(time_into_traj, q);
        traj_out_.GetVelocityInterp(time_into_traj, v);

        // traj_out_.GetConfigInterp(0.01, q);
        msg.base_link_name = "torso";
        vectorx_t q_head = q.head<FLOATING_POS_SIZE>();
        vectorx_t q_tail = q.tail(model_->GetNumInputs());
        msg.q_base = torc::utils::EigenToStdVector(q_head);
        msg.q_joints = torc::utils::EigenToStdVector(q_tail);

        msg.joint_names.resize(q_tail.size());
        msg.joint_names[0] = "left_hip_yaw_joint";
        msg.joint_names[1] = "left_hip_roll_joint";
        msg.joint_names[2] = "left_hip_pitch_joint";
        msg.joint_names[3] = "left_knee_pitch_joint";
        msg.joint_names[4] = "left_ankle_pitch_joint";
        msg.joint_names[5] = "left_shoulder_pitch_joint";
        msg.joint_names[6] = "left_shoulder_roll_joint";
        msg.joint_names[7] = "left_shoulder_yaw_joint";
        msg.joint_names[8] = "left_elbow_pitch_joint";
        msg.joint_names[9] = "right_hip_yaw_joint";
        msg.joint_names[10] = "right_hip_roll_joint";
        msg.joint_names[11] = "right_hip_pitch_joint";
        msg.joint_names[12] = "right_knee_pitch_joint";
        msg.joint_names[13] = "right_ankle_pitch_joint";
        msg.joint_names[14] = "right_shoulder_pitch_joint";
        msg.joint_names[15] = "right_shoulder_roll_joint";
        msg.joint_names[16] = "right_shoulder_yaw_joint";
        msg.joint_names[17] = "right_elbow_pitch_joint";

        // traj_out_.GetVelocityInterp(0.01, v);
        vectorx_t v_head = v.head<FLOATING_VEL_SIZE>();
        vectorx_t v_tail = v.tail(model_->GetNumInputs());

        // vectorx_t temp = vectorx_t::Zero(6);
        msg.v_base = torc::utils::EigenToStdVector(v_head);
        msg.v_joints = torc::utils::EigenToStdVector(v_tail);

        msg.header.stamp = this->now();

        if (!sim_ready_) {
            this->GetPublisher<obelisk_estimator_msgs::msg::EstimatedState>("state_viz_pub")->publish(msg);
        }
    }

    void MpcController::AddPeriodicContacts() {

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

        // For each contact determine last contact
        // If the last contact is within 1 second, then add another contact swing_time_ seconds later that lasts for swing_time_ + double_stance_time_
        // for (const auto& frame : mpc_->GetContactFrames()) {
        //     const double last_swing_time = contact_schedule_.GetLastSwingTime(frame);
        //     // RCLCPP_INFO_STREAM(this->get_logger(), "Last contact time: " << last_contact_time);
        //     if (last_swing_time < 1) {
        //         // RCLCPP_INFO_STREAM(this->get_logger(), "Adding a contact for " << frame << " at time: " << last_contact_time + swing_time_ << " until time: " << last_contact_time + 2*swing_time_ + double_stance_time_);
        //         contact_schedule_.InsertSwingByDuration(frame, last_swing_time + swing_time_ + double_stance_time_,  swing_time_);
        //     }
        // } 
    }

} // namespace achilles


int main(int argc, char* argv[]) {
    obelisk::utils::SpinObelisk<achilles::MpcController, rclcpp::executors::MultiThreadedExecutor>(
        argc, argv, "whole_body_controller");
}
