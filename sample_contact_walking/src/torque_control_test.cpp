#include "sample_contact_walking/torque_control_test.h"
#include "obelisk_ros_utils.h"

namespace robot {
    BasicTorqueControl::BasicTorqueControl(const std::string& name) 
        : obelisk::ObeliskController<obelisk_control_msgs::msg::PDFeedForward, obelisk_estimator_msgs::msg::EstimatedState>(name),
            received_first_state(false), ctrl_count(0) {
            // ----- Joystick Subscriber ----- //
            this->RegisterObkSubscription<sensor_msgs::msg::Joy>(
                "joystick_sub_setting", "joystick_sub",
                std::bind(&BasicTorqueControl::JoystickCallback, this, std::placeholders::_1));


            // Get the log file name
            this->declare_parameter<std::string>("log_file_name", "mpc_logs/basic_torque_control_log.csv");
            log_file.open(this->get_parameter("log_file_name").as_string());

            // Get the gains

            // Get the sizes and names
            this->declare_parameter<std::vector<std::string>>("ctrl_joint_names");
            this->declare_parameter<int>("num_ctrl_joints");
            this->declare_parameter<std::vector<double>>("kp");
            this->declare_parameter<std::vector<double>>("kd");
            this->declare_parameter<double>("torque_scalar");

            kp = this->get_parameter("kp").as_double_array();
            kd = this->get_parameter("kd").as_double_array();
            this->get_parameter("num_ctrl_joints", num_ctrl_joints);
            joint_names = this->get_parameter("ctrl_joint_names").as_string_array();
            torque_scalar = this->get_parameter("torque_scalar").as_double();

            // Choose the controller type
            control_type = 0;

            control_idx = 0;

            if (num_ctrl_joints != joint_names.size()) {
                throw std::runtime_error("um_ctrl_joints != joint_names.size()");
            }

            if (num_ctrl_joints != kp.size()) {
                throw std::runtime_error("um_ctrl_joints != kp.size()");
            }

            time_offset = this->now().seconds();
    }

    BasicTorqueControl::~BasicTorqueControl() {
        // log_file.close();
    }

    obelisk_control_msgs::msg::PDFeedForward BasicTorqueControl::ComputeControl() {
        obelisk_control_msgs::msg::PDFeedForward msg;
        
        msg.feed_forward.resize(num_ctrl_joints);
        msg.kp.resize(num_ctrl_joints);
        msg.kd.resize(num_ctrl_joints);
        msg.pos_target.resize(num_ctrl_joints);
        msg.vel_target.resize(num_ctrl_joints);
        msg.u_mujoco.resize(3*num_ctrl_joints);
        msg.joint_names = joint_names;

        if (received_first_state) {
            double current_time = this->now().seconds() - time_offset;

            // Sinusoidal torques
            double tau = torque_scalar*sin(current_time);


            if (control_type == 0) {
                // Torque only
                for (int i = 0; i < num_ctrl_joints; i++) {
                    if (i == control_idx) {
                        msg.feed_forward[i] = tau;
                        msg.u_mujoco[i + 2*(num_ctrl_joints)] = tau;
                    } else {
                        msg.feed_forward[i] = 0;
                        msg.u_mujoco[i + 2*(num_ctrl_joints)] = 0;
                    }
                    msg.kp[i] = 0;
                    msg.kd[i] = 0;
                    msg.pos_target[i] = 0;
                    msg.vel_target[i] = 0;

                    msg.u_mujoco[i] = 0;
                    msg.u_mujoco[i + num_ctrl_joints] = 0;
                }
            } else {
                throw std::runtime_error("Not supported control type!");
            }

            if (ctrl_count % 2 == 0) {
                log_file << current_time << ",";
                for (int i = 0; i < num_ctrl_joints; i++) {
                    log_file << msg.feed_forward[i] << ",";
                }
                for (int i = 0; i < q_est.size(); i++) {
                    log_file << q_est[i] << ",";
                }
                for (int i = 0; i < v_est.size(); i++) {
                    log_file << v_est[i] << ",";
                }
                log_file << std::endl;
            }

            msg.header.stamp = this->now();
            this->GetPublisher<obelisk_control_msgs::msg::PDFeedForward>(this->ctrl_key_)->publish(msg);        
        } else {
            for (int i = 0; i < num_ctrl_joints; i++) {
                msg.feed_forward[i] = 0;
                msg.u_mujoco[i + 2*(num_ctrl_joints)] = 0;
                msg.kp[i] = 0;
                msg.kd[i] = 0;
                msg.pos_target[i] = 0;
                msg.vel_target[i] = 0;

                msg.u_mujoco[i] = 0;
                msg.u_mujoco[i + num_ctrl_joints] = 0;
            }

            msg.header.stamp = this->now();
            this->GetPublisher<obelisk_control_msgs::msg::PDFeedForward>(this->ctrl_key_)->publish(msg);       
        }

        ctrl_count++;
        return msg;
    }

    void BasicTorqueControl::UpdateXHat(const obelisk_estimator_msgs::msg::EstimatedState& msg) {
        received_first_state = true;
        q_est = msg.q_joints;
        v_est = msg.v_joints;
    }

    void BasicTorqueControl::JoystickCallback(const sensor_msgs::msg::Joy& msg) {
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

        if (msg.buttons[X] && (this->now() - last_X_press).seconds() > 9e-1) {
            control_idx++;

            RCLCPP_INFO_STREAM(this->get_logger(), "Updating the control idx!");

            last_X_press = this->now();
        }
    }
}


int main(int argc, char* argv[]) {
    obelisk::utils::SpinObelisk<robot::BasicTorqueControl, rclcpp::executors::MultiThreadedExecutor>(
        argc, argv, "BasicTorqueControl");
}