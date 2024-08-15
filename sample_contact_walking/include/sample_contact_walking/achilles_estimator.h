#include <Eigen/Core>

#include "obelisk_estimator.h"

namespace achilles {
    class AchillesEstimator : public obelisk::ObeliskEstimator<obelisk_estimator_msgs::msg::EstimatedState> {
        public:
            AchillesEstimator(const std::string& name);
        protected:

        private:
            // TODO: What should be the callback groups
            void JointEncoderCallback(const obelisk_sensor_msgs::msg::ObkJointEncoders& msg);
            void MocapCallback(const obelisk_sensor_msgs::msg::ObkFramePose& msg);
            void TorsoImuCallback(const obelisk_sensor_msgs::msg::ObkImu& msg);

            obelisk_estimator_msgs::msg::EstimatedState ComputeStateEstimate() override;

            void MakeTorsoMocapTransform();

            // ---------- Member Variables ---------- //
            static constexpr int FLOATING_POS_SIZE = 7;
            static constexpr int FLOATING_VEL_SIZE = 6;
            static constexpr int POS_VARS = 3;
            static constexpr int QUAT_VARS = 4;

            // State flags
            bool recieved_first_encoders_;
            bool recieved_first_mocap_;
            bool recieved_first_imu_;

            // Intermediate estimate variables
            std::array<double, POS_VARS> prev_base_pos_;
            Eigen::Quaterniond prev_base_quat_;

            double prev_base_sec_;
            double prev_base_nanosec_;

            double base_sec_;
            double base_nanosec_;

            // Estimated state vectors
            std::string base_link_name_;
            std::vector<std::string> joint_names_;
            std::vector<double> joint_pos_;
            std::vector<double> joint_vels_;
            std::array<double, POS_VARS> base_pos_;
            Eigen::Quaterniond base_quat_;
            std::array<double, FLOATING_VEL_SIZE> base_vel_;

            // Messages
            obelisk_estimator_msgs::msg::EstimatedState est_state_msg_;

            // broadcasters
            std::shared_ptr<tf2_ros::StaticTransformBroadcaster> torso_mocap_broadcaster_;
    };
} // namespace achilles