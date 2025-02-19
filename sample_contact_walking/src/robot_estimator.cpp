
#include "pinocchio/math/quaternion.hpp"
#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/fwd.hpp"
#include "pinocchio/spatial/explog.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "sample_contact_walking/robot_estimator.h"
#include "obelisk_ros_utils.h"

namespace robot {
    RobotEstimator::RobotEstimator(const std::string& name) 
        : obelisk::ObeliskEstimator<obelisk_estimator_msgs::msg::EstimatedState>(name),
        recieved_first_encoders_(false), recieved_first_mocap_(false), recieved_first_pelvis_imu_(false), recieved_first_torso_imu_(false),
            use_sim_state_(false), use_torso_mocap_(true), jnt_vel_var_(0), base_vel_var_(0)  {
        // ---------- Joint Encoder Subscription ---------- /, /
        this->RegisterObkSubscription<obelisk_sensor_msgs::msg::ObkJointEncoders>(
            "joint_encoders_setting", "jnt_sensor",
            std::bind(&RobotEstimator::JointEncoderCallback, this, std::placeholders::_1));
        
        this->declare_parameter<bool>("use_torso_mocap", true);
        use_torso_mocap_ = this->get_parameter("use_torso_mocap").as_bool();

        if (!use_torso_mocap_) {
            // ---------- Pelvis Mocap Subscription ---------- //
            this->RegisterObkSubscription<geometry_msgs::msg::PoseStamped>(
                "mocap_setting", "pelvis_mocap_sensor",
                std::bind(&RobotEstimator::PelvisMocapCallback, this, std::placeholders::_1));
            RCLCPP_INFO_STREAM(this->get_logger(), "Using the pelvis mocap!");
        } else {
            // ---------- Torso Mocap Subscription ---------- //
            this->RegisterObkSubscription<geometry_msgs::msg::PoseStamped>(
                "torso_mocap_setting", "torso_mocap_sensor",
                std::bind(&RobotEstimator::TorsoMocapCallback, this, std::placeholders::_1));
            RCLCPP_INFO_STREAM(this->get_logger(), "Using the torso mocap!");
        }

        // ---------- IMU Subscriptions ---------- //
        this->RegisterObkSubscription<obelisk_sensor_msgs::msg::ObkImu>(
            "torso_imu_setting", "torso_imu_sensor",
            std::bind(&RobotEstimator::TorsoImuCallback, this, std::placeholders::_1));
        
        this->RegisterObkSubscription<obelisk_sensor_msgs::msg::ObkImu>(
            "pelvis_imu_setting", "pelvis_imu_sensor",
            std::bind(&RobotEstimator::PelvisImuCallback, this, std::placeholders::_1));


        this->declare_parameter<bool>("use_sim_state", false);
        use_sim_state_ = this->get_parameter("use_sim_state").as_bool();

        if (use_sim_state_) {
            RCLCPP_WARN_STREAM(this->get_logger(), "Using the true sim state for state estimation!");
            // ---------- True Sim State Subscription ---------- //
            this->RegisterObkSubscription<obelisk_sensor_msgs::msg::TrueSimState>(
                "true_sim_sub_setting", "true_sim_state",
                std::bind(&RobotEstimator::TrueStateCallback, this, std::placeholders::_1));
        }

        // URDF
        this->declare_parameter<std::string>("urdf_path", "");
        std::filesystem::path urdf_path(this->get_parameter("urdf_path").as_string());

        // MPC Settings
        this->declare_parameter<std::string>("mpc_settings_path", "");
        mpc_settings_ = std::make_shared<torc::mpc::MpcSettings>(this->get_parameter("mpc_settings_path").as_string());

        // Create model
        this->declare_parameter<std::string>("robot_name", "");
        std::string robot_name = this->get_parameter("robot_name").as_string();
        RCLCPP_INFO_STREAM(this->get_logger(), "Config yaml robot name: " << robot_name);
        std::string model_name = name + robot_name + "_model";
        mpc_model_ = std::make_unique<torc::models::FullOrderRigidBody>(model_name, urdf_path,  mpc_settings_->joint_skip_names, mpc_settings_->joint_skip_values);

        this->declare_parameter<std::string>("base_link_name");
        base_link_name_ = this->get_parameter("base_link_name").as_string();

        this->declare_parameter<std::vector<double>>("pelvis_ang_vel_lpf_coefs");
        pelvis_ang_vel_lpf_ = std::make_unique<torc::state_est::LowPassFilter>(this->get_parameter("pelvis_ang_vel_lpf_coefs").as_double_array());

        this->declare_parameter<double>("joint_vel_variance", 0);
        this->get_parameter("joint_vel_variance", jnt_vel_var_);

        this->declare_parameter<double>("base_vel_variance", 0);
        this->get_parameter("base_vel_variance", base_vel_var_);

        // Reset values
        joint_names_.clear();
        joint_pos_.clear();
        joint_vels_.clear();

        std::fill(std::begin(prev_base_pos_), std::end(prev_base_pos_), 0);
        std::fill(std::begin(base_pos_), std::end(base_pos_), 0);


        time_offset_ = this->now().seconds();
        this->declare_parameter<std::string>("log_file_name", "mpc_logs/estimator_log.csv");
        std::string log_name = this->get_parameter("log_file_name").as_string();
        RCLCPP_INFO_STREAM(this->get_logger(), "Estimator is logging to: " << log_name);
        log_file_.open(log_name);

        // Make broadcasters
        torso_mocap_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        this->MakeTorsoMocapTransform();
    }

    void RobotEstimator::JointEncoderCallback(const obelisk_sensor_msgs::msg::ObkJointEncoders& msg) {
        // TODO: Assign joints by name to match the model
        // joint_pos_ = msg.joint_pos;
        // joint_vels_ = msg.joint_vel;
        // joint_names_ = msg.joint_names;        

        static std::random_device rd;  // Seed generator
        static std::mt19937 gen(rd()); // Mersenne Twister engine
        double stddev = std::sqrt(jnt_vel_var_);  // Standard deviation
        static std::normal_distribution<double> dist(0, stddev); // Normal distribution

        joint_pos_.resize(mpc_model_->GetConfigDim() - 7);
        joint_vels_.resize(mpc_model_->GetVelDim() - 6);
        joint_names_.resize(mpc_model_->GetConfigDim() - 7);

        if (msg.joint_names.size() != msg.joint_pos.size()) {
            throw std::runtime_error("[JointEncoderCallback] msg is not self consistent!");
        }

        // Configuration
        // Only receive the joints that we use in the MPC
        for (size_t i = 0; i < msg.joint_pos.size(); i++) {
            const auto joint_idx = mpc_model_->GetJointID(msg.joint_names[i]);
            if (joint_idx.has_value()) {
                if (joint_idx.value() < 2) {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid joint name!");
                }
                joint_pos_[joint_idx.value() - 2] = msg.joint_pos.at(i);     // Offset for the root and base joints

                joint_names_[joint_idx.value() - 2] = msg.joint_names[i];   // Assing joint names
            }
        }

        // Velocity
        for (size_t i = 0; i < msg.joint_vel.size(); i++) {
            const auto joint_idx = mpc_model_->GetJointID(msg.joint_names[i]);
            if (joint_idx.has_value()) {
                if (joint_idx.value() < 2) {
                    RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid joint name!");
                }
                joint_vels_[joint_idx.value() - 2] = msg.joint_vel.at(i);     // Offset for the root and base joints
                if (jnt_vel_var_ > 0) {
                    joint_vels_[joint_idx.value() - 2] += dist(gen);
                }
            }
        }

        recieved_first_encoders_ = true;
    }

    void RobotEstimator::PelvisMocapCallback(const geometry_msgs::msg::PoseStamped& msg) {
        prev_base_pos_ = base_pos_;
        prev_base_quat_ = base_quat_;
        prev_base_sec_ = base_sec_;
        prev_base_nanosec_ = base_nanosec_;
        prev_mocap_time_ = mocap_time_;

        base_pos_.at(0) = msg.pose.position.x;
        base_pos_.at(1) = msg.pose.position.y;
        base_pos_.at(2) = msg.pose.position.z;

        base_quat_.x() = msg.pose.orientation.x;
        base_quat_.y() = msg.pose.orientation.y;
        base_quat_.z() = msg.pose.orientation.z;
        base_quat_.w() = msg.pose.orientation.w;

        // Normalize
        double norm = std::sqrt(base_quat_.x()*base_quat_.x() + base_quat_.y()*base_quat_.y() + base_quat_.z()*base_quat_.z() + base_quat_.w()*base_quat_.w());
        base_quat_.x() = base_quat_.x()/norm;
        base_quat_.y() = base_quat_.y()/norm;
        base_quat_.z() = base_quat_.z()/norm;
        base_quat_.w() = base_quat_.w()/norm;

        base_sec_ = msg.header.stamp.sec;
        base_nanosec_ = msg.header.stamp.nanosec;
        mocap_time_ = msg.header.stamp;

        if (!use_sim_state_ && recieved_first_mocap_) {
            double elapsed_time = (mocap_time_ - prev_mocap_time_).seconds();
            if (elapsed_time > 0) {
                // Finite difference the mocap data for velocities
                base_vel_world_[0] = (base_pos_[0] - prev_base_pos_[0])/elapsed_time;
                base_vel_world_[1] = (base_pos_[1] - prev_base_pos_[1])/elapsed_time;
                base_vel_world_[2] = (base_pos_[2] - prev_base_pos_[2])/elapsed_time;
            }
        }

        if (msg.header.frame_id != "world") {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Mocap sensor is with respect to the wrong frame! Expecting `world`, got: " << msg.header.frame_id);
        }

        recieved_first_mocap_ = true;
    }

        void RobotEstimator::TorsoMocapCallback(const geometry_msgs::msg::PoseStamped& msg) {
        prev_base_pos_ = base_pos_;
        prev_base_quat_ = base_quat_;
        prev_base_sec_ = base_sec_;
        prev_base_nanosec_ = base_nanosec_;
        prev_mocap_time_ = mocap_time_;

        // TODO: If I go to async then I may want to do this in a the compute estimated state function
        // Convert the sensor data into the pelvis (base) frame
        torc::models::vector3_t pos;
        pos << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
        torc::models::quat_t orientation(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);
        pinocchio::SE3 frame_pose(orientation, pos);

        torc::models::vectorx_t config = mpc_model_->GetNeutralConfig();
        for (int i = 0; i < joint_pos_.size(); i++) {
            config[7 + i] = joint_pos_[i];
        }

        pinocchio::SE3 base_pose = mpc_model_->TransformPose(frame_pose, "torso_mocap", "pelvis", config);

        base_pos_.at(0) = base_pose.translation()[0];
        base_pos_.at(1) = base_pose.translation()[1];
        base_pos_.at(2) = base_pose.translation()[2];

        torc::models::quat_t fb_quat(base_pose.rotation());

        base_quat_.x() = fb_quat.x();
        base_quat_.y() = fb_quat.y();
        base_quat_.z() = fb_quat.z();
        base_quat_.w() = fb_quat.w();

        base_sec_ = msg.header.stamp.sec;
        base_nanosec_ = msg.header.stamp.nanosec;
        mocap_time_ = msg.header.stamp;

        if (!use_sim_state_ && recieved_first_mocap_) {
            double elapsed_time = (mocap_time_ - prev_mocap_time_).seconds();
            if (elapsed_time > 0) {
                // Finite difference the mocap data for velocities
                base_vel_world_[0] = (base_pos_[0] - prev_base_pos_[0])/elapsed_time;
                base_vel_world_[1] = (base_pos_[1] - prev_base_pos_[1])/elapsed_time;
                base_vel_world_[2] = (base_pos_[2] - prev_base_pos_[2])/elapsed_time;
            }
        }

        if (msg.header.frame_id != "world") {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Mocap sensor is with respect to the wrong frame! Expecting `world`, got: " << msg.header.frame_id);
        }

        recieved_first_mocap_ = true;
    }

    // TODO: Fill out callback
    void RobotEstimator::TorsoImuCallback(__attribute__((__unused__)) const obelisk_sensor_msgs::msg::ObkImu& msg) {
        recieved_first_torso_imu_ = true;
    }

    void RobotEstimator::PelvisImuCallback(__attribute__((__unused__)) const obelisk_sensor_msgs::msg::ObkImu& msg) {
        static std::random_device rd;  // Seed generator
        static std::mt19937 gen(rd()); // Mersenne Twister engine
        double stddev = std::sqrt(base_vel_var_);  // Standard deviation
        static std::normal_distribution<double> dist(0, stddev); // Normal distribution

        if (!use_sim_state_) {
            // Low pass filter this
            Eigen::Vector3d new_ang_vel;
            new_ang_vel << msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z;
            Eigen::Vector3d filtered_ang_vel = pelvis_ang_vel_lpf_->Filter(new_ang_vel);

            // TODO: Rotate into the correct frame. For the G1 this sould be an identity rotation

            base_ang_vel_local_[0] = filtered_ang_vel(0);
            base_ang_vel_local_[1] = filtered_ang_vel(1);
            base_ang_vel_local_[2] = filtered_ang_vel(2);

            if (base_vel_var_ > 0) {
                base_ang_vel_local_[0] += dist(gen);
                base_ang_vel_local_[1] += dist(gen);
                base_ang_vel_local_[2] += dist(gen);
            }
        }

        recieved_first_pelvis_imu_ = true;
    }

    obelisk_estimator_msgs::msg::EstimatedState RobotEstimator::ComputeStateEstimate() {
        
        if (recieved_first_pelvis_imu_ && recieved_first_encoders_ && recieved_first_mocap_) {
            RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Starting to publish estimated states.");

            double d_sec = base_sec_ - prev_base_sec_;
            double d_nano_sec = base_nanosec_ - prev_base_nanosec_;
            double dt = ((d_sec * 1e9) + d_nano_sec)/1e9;

            est_state_msg_.q_joints = joint_pos_;

            est_state_msg_.q_base.clear();
            for (int i = 0; i < POS_VARS; i++) {
                est_state_msg_.q_base.emplace_back(base_pos_[i]);
            }

            est_state_msg_.q_base.emplace_back(base_quat_.x());
            est_state_msg_.q_base.emplace_back(base_quat_.y());
            est_state_msg_.q_base.emplace_back(base_quat_.z());
            est_state_msg_.q_base.emplace_back(base_quat_.w());

            est_state_msg_.joint_names = joint_names_;
            est_state_msg_.base_link_name = base_link_name_;

            est_state_msg_.v_joints = joint_vels_;

            // Assign v_base from the true sim state
            // Need to rotate in to the local frame
            Eigen::Map<vector3_t> v_world_linear(base_vel_world_.data());
            Eigen::Map<vector3_t> v_local_angular(base_ang_vel_local_.data());

            vector6_t local_vel;

            Eigen::Quaterniond base_quat(base_quat_.w(), base_quat_.x(), base_quat_.y(), base_quat_.z());
            // TODO: Double check this conversion
            local_vel.head<POS_VARS>() = base_quat.toRotationMatrix().transpose() * v_world_linear;
            local_vel.tail<POS_VARS>() = v_local_angular;

            // local_vel.head<POS_VARS>() = v_world_linear;
            // local_vel.tail<POS_VARS>() = v_world_angular;

            est_state_msg_.v_base.clear();
            for (int i = 0; i < FLOATING_VEL_SIZE; i++) {
                est_state_msg_.v_base.emplace_back(local_vel(i));
            }

            est_state_msg_.header.stamp = this->now();

            log_file_ << this->now().seconds() - time_offset_ << ",";
            for (int i = 0; i < 7; i++) {
                log_file_ << est_state_msg_.q_base[i] << ",";
            }
            for (int i = 0; i < joint_names_.size(); i++) {
                log_file_ << est_state_msg_.q_joints[i] << ",";
            }
            for (int i = 0; i < 6; i++) {
                log_file_ << est_state_msg_.v_base[i] << ",";
            }
            for (int i = 0; i < joint_names_.size(); i++) {
                log_file_ << est_state_msg_.v_joints[i] << ",";
            }
            log_file_ << std::endl;
            // Regardless of state of incoming data (i.e. even if dt <= 0)
            // TODO: Put back!
            this->GetPublisher<obelisk_estimator_msgs::msg::EstimatedState>(this->est_pub_key_)->publish(est_state_msg_);
        } else {
            RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Waiting on sensor measurements to publish estimated state.");
        }

        return est_state_msg_;
    }

    void RobotEstimator::MakeTorsoMocapTransform() {
        // TODO: Investigate if this is needed
        // geometry_msgs::msg::TransformStamped t;

        // t.header.stamp = this->get_clock()->now();
        // t.header.frame_id = "world"; //"pelvis"//"torso_mocap_site";     // This must match the base link in the estimated state
        // t.child_frame_id = this->get_parameter("base_link_name").as_string();             // Must match the the base link in the urdf

        // t.transform.translation.x = 0.0;
        // t.transform.translation.y = 0.0;
        // t.transform.translation.z = 0.0;
        // tf2::Quaternion q;
        // q.setRPY(
        // 0.0,
        // 0.0, //3.14,
        // 0.0); //3.14);
        // t.transform.rotation.x = q.x();
        // t.transform.rotation.y = q.y();
        // t.transform.rotation.z = q.z();
        // t.transform.rotation.w = q.w();

        // torso_mocap_broadcaster_->sendTransform(t);
    } 

    void RobotEstimator::TrueStateCallback(const obelisk_sensor_msgs::msg::TrueSimState& msg) {
        for (size_t i = 0; i < POS_VARS; i++) {
            base_vel_world_[i] = msg.v_base[i];
        }
        for (size_t i = 0; i < POS_VARS; i++) {
            base_ang_vel_local_[i] = msg.v_base[i + POS_VARS];
        }
    }
} // namespace robot

int main(int argc, char* argv[]) {
    obelisk::utils::SpinObelisk<robot::RobotEstimator, rclcpp::executors::MultiThreadedExecutor>(
        argc, argv, "state_estimator");
}