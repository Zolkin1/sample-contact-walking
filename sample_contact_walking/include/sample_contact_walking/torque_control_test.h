#include <atomic>
#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "obelisk_controller.h"

#include "sensor_msgs/msg/joy.hpp"


namespace robot {
    class BasicTorqueControl : public obelisk::ObeliskController<obelisk_control_msgs::msg::PDFeedForward, obelisk_estimator_msgs::msg::EstimatedState> {
    public:
        BasicTorqueControl(const std::string& name);
        ~BasicTorqueControl();
    protected:
        // Control functions
        obelisk_control_msgs::msg::PDFeedForward ComputeControl() override;

        // State functions
        void UpdateXHat(const obelisk_estimator_msgs::msg::EstimatedState& msg) override;

        // Joystick interface
        void JoystickCallback(const sensor_msgs::msg::Joy& msg);

        std::vector<double> q_est;
        std::vector<double> v_est;

        std::vector<double> kp;
        std::vector<double> kd;

        double torque_scalar;

        int control_type;

        double time_offset;

        bool received_first_state;

        std::vector<std::string> joint_names;
        int num_ctrl_joints;

        std::ofstream log_file;

        int control_idx;

        unsigned long ctrl_count;
    };
}