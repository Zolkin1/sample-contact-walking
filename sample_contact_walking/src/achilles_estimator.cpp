
#include "pinocchio/math/quaternion.hpp"
#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/fwd.hpp"
#include "pinocchio/spatial/explog.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "sample_contact_walking/achilles_estimator.h"
#include "obelisk_ros_utils.h"

namespace achilles {
    AchillesEstimator::AchillesEstimator(const std::string& name) 
        : obelisk::ObeliskEstimator<obelisk_estimator_msgs::msg::EstimatedState>(name),
        recieved_first_encoders_(false), recieved_first_mocap_(false), recieved_first_imu_(false)  {
        // ---------- Joint Encoder Subscription ---------- /, /
        this->RegisterObkSubscription<obelisk_sensor_msgs::msg::ObkJointEncoders>(
            "joint_encoders_setting", "jnt_sensor",
            std::bind(&AchillesEstimator::JointEncoderCallback, this, std::placeholders::_1));
        
        // ---------- Mocap Subscription ---------- //
        this->RegisterObkSubscription<obelisk_sensor_msgs::msg::ObkFramePose>(
            "mocap_setting", "mocap_sensor",
            std::bind(&AchillesEstimator::MocapCallback, this, std::placeholders::_1));

        // ---------- IMU Subscriptions ---------- //
        this->RegisterObkSubscription<obelisk_sensor_msgs::msg::ObkImu>(
            "torso_imu_setting", "torso_imu_sensor",
            std::bind(&AchillesEstimator::TorsoImuCallback, this, std::placeholders::_1));

        // Create broadcasters to map Mujoco Sites to frames

        // Reset values
        joint_names_.clear();
        joint_pos_.clear();
        joint_vels_.clear();

        std::fill(std::begin(prev_base_pos_), std::end(prev_base_pos_), 0);
        std::fill(std::begin(base_pos_), std::end(base_pos_), 0);

        // TODO: remove
        recieved_first_imu_ = true;

        // Make broadcasters
        torso_mocap_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        this->MakeTorsoMocapTransform();
    }

    void AchillesEstimator::JointEncoderCallback(const obelisk_sensor_msgs::msg::ObkJointEncoders& msg) {
        recieved_first_encoders_ = true;

        joint_pos_ = msg.joint_pos;
        joint_vels_ = msg.joint_vel;
        joint_names_ = msg.joint_names;        
    }

    void AchillesEstimator::MocapCallback(const obelisk_sensor_msgs::msg::ObkFramePose& msg) {
        recieved_first_mocap_ = true;

        prev_base_pos_ = base_pos_;
        prev_base_quat_ = base_quat_;
        prev_base_sec_ = base_sec_;
        prev_base_nanosec_ = base_nanosec_;

        base_pos_.at(0) = msg.position.x;
        base_pos_.at(1) = msg.position.y;
        base_pos_.at(2) = msg.position.z;

        base_quat_.x() = msg.orientation.x;
        base_quat_.y() = msg.orientation.y;
        base_quat_.z() = msg.orientation.z;
        base_quat_.w() = msg.orientation.w;

        base_sec_ = msg.header.stamp.sec;
        base_nanosec_ = msg.header.stamp.nanosec;

        base_link_name_ = msg.frame_name;

        if (msg.header.frame_id != "world") {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Mocap sensor is with respect to the wrong frame! Expecting `world`, got: " << msg.header.frame_id);
        }
    }

    void AchillesEstimator::TorsoImuCallback(__attribute__((__unused__)) const obelisk_sensor_msgs::msg::ObkImu& msg) {
        recieved_first_imu_ = true;
    }

    obelisk_estimator_msgs::msg::EstimatedState AchillesEstimator::ComputeStateEstimate() {
        obelisk_estimator_msgs::msg::EstimatedState msg;
        if (recieved_first_imu_ && recieved_first_encoders_ && recieved_first_mocap_) {
            double d_sec = base_sec_ - prev_base_sec_;
            double d_nano_sec = base_nanosec_ - prev_base_nanosec_;
            double dt = ((d_sec * 1e9) + d_nano_sec)/1e9;

            if (dt > 0) {
                RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Starting to publish estimated states.");

                msg.q_joints = joint_pos_;
                for (int i = 0; i < POS_VARS; i++) {
                    msg.q_base.emplace_back(base_pos_.at(i));
                }

                msg.q_base.emplace_back(base_quat_.x());
                msg.q_base.emplace_back(base_quat_.y());
                msg.q_base.emplace_back(base_quat_.z());
                msg.q_base.emplace_back(base_quat_.w());

                
                msg.joint_names = joint_names_;
                msg.base_link_name = base_link_name_;

                msg.v_joints = joint_vels_;

                // Calculate base velocity via simple euler
                double d_sec = base_sec_ - prev_base_sec_;
                double d_nano_sec = base_nanosec_ - prev_base_nanosec_;
                double dt = ((d_sec * 1e9) + d_nano_sec)/1e9;

                for (size_t i = 0; i < POS_VARS; i++) {
                    base_vel_.at(i) = (base_pos_.at(i) - prev_base_pos_.at(i))/dt;
                }

                base_quat_.normalize();
                prev_base_quat_.normalize();

                Eigen::Vector3d tangent_vec = pinocchio::quaternion::log3(base_quat_); // prev_quat.inverse()*

                for (size_t i = 0; i < 3; i++) {
                    base_vel_.at(i + POS_VARS) = tangent_vec(i)/dt;
                }

                for (int i = 0; i < FLOATING_VEL_SIZE; i++) {
                    msg.v_base.emplace_back(base_vel_.at(i));
                }

                this->GetPublisher<obelisk_estimator_msgs::msg::EstimatedState>(this->est_pub_key_)->publish(msg);
            }
        } else {
            RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Waiting on sensor measurements to publish estimated state.");
        }

        return msg;
    }

    void AchillesEstimator::MakeTorsoMocapTransform() {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "torso_mocap_site";
        t.child_frame_id = "base_link";

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

} // namespace achilles

int main(int argc, char* argv[]) {
    obelisk::utils::SpinObelisk<achilles::AchillesEstimator, rclcpp::executors::MultiThreadedExecutor>(
        argc, argv, "state_estimator");
}