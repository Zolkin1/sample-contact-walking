#include "obelisk_estimator.h"

namespace achilles {
    class AchillesEstimator : public obelisk::ObeliskEstimator<obelisk_estimator_msgs::msg::EstimatedState> {
        public:
            AchillesEstimator();
        protected:

        private:
            // TODO: What should be the callback groups
            void JointEncoderCallback(const obelisk_sensor_msgs::msg::ObkJointEncoders& msg);
            void MocapCallback(const obelisk_sensor_msgs::msg::ObkFramePose& msg);
            void ImuCallback(const obelisk_sensor_msgs::msg::ObkImu& msg)

            obelisk_estimator_msgs::msg::EstimatedState ComputeStateEstimate() {
                obelisk_estimator_msgs::msg::EstimatedState msg;
                if (recieved_first_imu_ && recieved_first_encoders_ && recieved_first_mocap_) {
                    RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Starting to publish estimated states.");

                    msg.q_joints = joint_pos_;
                    msg.q_base = base_pos_;
                    
                    msg.joint_names = joint_names_;
                    msg.base_link_name = base_link_name_;

                    msg.v_joints = joint_vels_;
                    msg.v_base = base_vel_;

                    this->GetPublisher<obelisk_estimator_msgs::msg::EstimatedState>(this->est_pub_key_)->publish(msg);
                } else {
                    RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Waiting on sensor measurements to publish estimated state.");
                }

                return msg;
            }

            // ---------- Member Variables ---------- //
            static constexpr FLOATING_POS_SIZE = 7;
            static constexpr FLOATING_VEL_SIZE = 6;

            bool recieved_first_encoders_;
            bool recieved_first_mocap_;
            bool recieved_first_imu_;

            std::string base_link_name_;
            std::vector<std::string> joint_names_;
            std::vector<double> joint_pos_;
            std::vector<double> joint_vels_;
            std::array<double, FLOATING_POS_SIZE> base_pos_;
            std::array<double, FLOATING_VEL_SIZE> base_vel_;
    };
} // namespace achilles