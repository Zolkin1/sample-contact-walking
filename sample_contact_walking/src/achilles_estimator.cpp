#include "sample_contact_walking/achilles_estimator.h"
#include "obelisk_ros_utils.h"
// #include "position_setpoint_controller.h"

namespace achilles {
    AchillesEstimator::AchillesEstimator(const std::string& name) 
        : obelisk::ObelsikEstimator<obelisk_estimator_msgs::msg::EstimatedState>(name),
        recieved_first_encoders_(false), recieved_first_mocap_(false), recieved_first_imu_(false)  {
        // ---------- Joint Encoder Subscription ---------- /, /
        this->RegisterObkSubscription<obelisk_sensor_msgs::msg::ObkJointEncoders>(
            "joint_encoders_setting", "jnt_sensor",
            std::bind(&AchillesEstimator::JointEncoderCallback, this, std::placeholders::_1));
        
        // ---------- Mocap Subscription ---------- //
        this->RegisterObkSubscription<obelisk_sensor_msgs::msg::ObkFramePose>(
            "mocap_setting", "mocap_sensor",
            std::bind(&AchillesEstimator::MocapCallback, this, std::placeholders::_1));

        // Create broadcasters to map Mujoco Sites to frames

        // Reset values
        joint_names_.clear();
        joint_pos_.clear();
        joint_vels_.clear();
    }

    void AchillesEstimator::JointEncoderCallback(const obelisk_sensor_msgs::msg::ObkJointEncoders& msg) {
        recieved_first_encoders_ = true;
    }

    void AchillesEstimator::MocapCallback(const obelisk_sensor_msgs::msg::ObkFramePose& msg) {
        recieved_first_mocap_ = true;
    }

    void AchillesEstimator::ImuCallback(const obelisk_sensor_msgs::msg::ObkImu& msg) {
        recieved_first_imu_ = true;
    }

} // namespace achilles

int main(int argc, char* argv[]) {
    obelisk::utils::SpinObelisk<achilles::AchillesEstimator, rclcpp::executors::MultiThreadedExecutor>(
        argc, argv, "state_estimator");
}