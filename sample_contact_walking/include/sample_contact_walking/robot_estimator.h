#include <Eigen/Core>
#include "rclcpp/time.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "sensor_msgs/msg/joy.hpp"

#include "obelisk_estimator.h"
#include "low_pass_filter.h"
#include "full_order_rigid_body.h"
#include "MpcSettings.h"

// TODO: Remove after debug!
// #include "obelisk_controller.h"

namespace robot {
    using vector3_t = Eigen::Vector3d;
    using vector6_t = Eigen::Vector<double, 6>;

    class RobotEstimator : public obelisk::ObeliskEstimator<obelisk_estimator_msgs::msg::EstimatedState> {
        public:
            RobotEstimator(const std::string& name);
        protected:

        private:
            // TODO: What should be the callback groups
            void JointEncoderCallback(const obelisk_sensor_msgs::msg::ObkJointEncoders& msg);
            void PelvisMocapCallback(const geometry_msgs::msg::PoseStamped& msg);
            void TorsoMocapCallback(const geometry_msgs::msg::PoseStamped& msg);
            void TorsoImuCallback(const obelisk_sensor_msgs::msg::ObkImu& msg);
            void PelvisImuCallback(const obelisk_sensor_msgs::msg::ObkImu& msg);
            void TorsoOdomCallback(const nav_msgs::msg::Odometry& msg);
            void JoystickCallback(const sensor_msgs::msg::Joy& msg);

            obelisk_estimator_msgs::msg::EstimatedState ComputeStateEstimate() override;

            void MakeTorsoMocapTransform();

            // True sim state for debugging
            void TrueStateCallback(const obelisk_sensor_msgs::msg::TrueSimState& msg);

            // TODO: Remove after testing
            // void ReceiveControlDebug(const obelisk_control_msgs::msg::PDFeedForward& msg);

            // ---------- Member Variables ---------- //
            static constexpr int FLOATING_POS_SIZE = 7;
            static constexpr int FLOATING_VEL_SIZE = 6;
            static constexpr int POS_VARS = 3;
            static constexpr int QUAT_VARS = 4;

            // State flags
            bool recieved_first_encoders_;
            bool recieved_first_mocap_;
            bool recieved_first_pelvis_imu_;
            bool recieved_first_torso_imu_;
            bool received_first_camera_;

            bool use_sim_state_;    // Determine if TrueSimState should be read
            bool use_torso_mocap_;
            int base_pose_sensor_;

            // Intermediate estimate variables
            std::array<double, POS_VARS> prev_base_pos_;
            Eigen::Quaterniond prev_base_quat_;

            double prev_base_sec_;
            double prev_base_nanosec_;
            rclcpp::Time prev_mocap_time_;

            double base_sec_;
            double base_nanosec_;
            rclcpp::Time mocap_time_;

            // Names
            std::string base_link_name_;
            std::vector<std::string> joint_names_;

            // Joint States
            std::vector<double> joint_pos_;
            std::vector<double> joint_vels_;

            // Base States
            std::array<double, POS_VARS> base_pos_;
            Eigen::Quaterniond base_quat_;
            std::array<double, POS_VARS> base_vel_world_;
            std::array<double, POS_VARS> base_vel_local_;
            std::array<double, POS_VARS> base_ang_vel_local_;

            // Messages
            obelisk_estimator_msgs::msg::EstimatedState est_state_msg_;

            // broadcasters
            std::shared_ptr<tf2_ros::StaticTransformBroadcaster> torso_mocap_broadcaster_;

            // Filters
            std::unique_ptr<torc::state_est::LowPassFilter> pelvis_ang_vel_lpf_;
            std::unique_ptr<torc::state_est::LowPassFilter> camera_pos_lpf_;

            // Robot model
            std::unique_ptr<torc::models::FullOrderRigidBody> mpc_model_; // Reduced model
            std::shared_ptr<torc::mpc::MpcSettings> mpc_settings_;

            // Simulation
            double jnt_vel_var_;
            double base_vel_var_;

            // Logging
            double time_offset_;
            std::ofstream log_file_;

            // Default pose
            pinocchio::SE3 default_pose_;

            // Name
            std::string robot_name_;
    };
} // namespace robot