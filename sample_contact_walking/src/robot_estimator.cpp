
#include "pinocchio/math/quaternion.hpp"
#include "pinocchio/spatial/fwd.hpp"
#include "pinocchio/fwd.hpp"
#include "pinocchio/spatial/explog.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "sample_contact_walking/robot_estimator.h"
#include "obelisk_ros_utils.h"

namespace robot {
    RobotEstimator::RobotEstimator(const std::string& name) 
        : obelisk::ObeliskEstimator<obelisk_estimator_msgs::msg::EstimatedState>(name),
        recieved_first_encoders_(false), recieved_first_mocap_(false), recieved_first_imu_(false)  {
        // ---------- Joint Encoder Subscription ---------- /, /
        this->RegisterObkSubscription<obelisk_sensor_msgs::msg::ObkJointEncoders>(
            "joint_encoders_setting", "jnt_sensor",
            std::bind(&RobotEstimator::JointEncoderCallback, this, std::placeholders::_1));
        
        // ---------- Mocap Subscription ---------- //
        this->RegisterObkSubscription<obelisk_sensor_msgs::msg::ObkFramePose>(
            "mocap_setting", "mocap_sensor",
            std::bind(&RobotEstimator::MocapCallback, this, std::placeholders::_1));

        // ---------- IMU Subscriptions ---------- //
        this->RegisterObkSubscription<obelisk_sensor_msgs::msg::ObkImu>(
            "torso_imu_setting", "torso_imu_sensor",
            std::bind(&RobotEstimator::TorsoImuCallback, this, std::placeholders::_1));

        // ---------- True Sim State Subscription ---------- //
        this->RegisterObkSubscription<obelisk_sensor_msgs::msg::TrueSimState>(
            "true_sim_sub_setting", "true_sim_state",
            std::bind(&RobotEstimator::TrueStateCallback, this, std::placeholders::_1));

        // ------ DEBUG ------ //
        // this->RegisterObkSubscription<obelisk_control_msgs::msg::PDFeedForward>("debug_print_setting",
        //     "debug_print", std::bind(&RobotEstimator::ReceiveControlDebug, this, std::placeholders::_1));
        // this->RegisterObkPublisher<obelisk_control_msgs::msg::PDFeedForward>("debug_pub_setting", "debug_pub");
        // ------ DEBUG ------ //
        
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

    void RobotEstimator::JointEncoderCallback(const obelisk_sensor_msgs::msg::ObkJointEncoders& msg) {
        recieved_first_encoders_ = true;

        joint_pos_ = msg.joint_pos;
        joint_vels_ = msg.joint_vel;
        joint_names_ = msg.joint_names;        
    }

    void RobotEstimator::MocapCallback(const obelisk_sensor_msgs::msg::ObkFramePose& msg) {
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

    void RobotEstimator::TorsoImuCallback(__attribute__((__unused__)) const obelisk_sensor_msgs::msg::ObkImu& msg) {
        recieved_first_imu_ = true;
    }

    obelisk_estimator_msgs::msg::EstimatedState RobotEstimator::ComputeStateEstimate() {
        
        if (recieved_first_imu_ && recieved_first_encoders_ && recieved_first_mocap_) {
            RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Starting to publish estimated states.");

            double d_sec = base_sec_ - prev_base_sec_;
            double d_nano_sec = base_nanosec_ - prev_base_nanosec_;
            double dt = ((d_sec * 1e9) + d_nano_sec)/1e9;

            if (dt > 0) {
                // TODO: Base velocity is in world frame from mujoco, but the MPC needs it in local frame
                // TODO: This is quite a bad estimate, need to clean it up
                // For now, I will just grab the true velocity

                // Calculate base velocity via simple euler
                // for (size_t i = 0; i < POS_VARS; i++) {
                //     base_vel_.at(i) = (base_pos_.at(i) - prev_base_pos_.at(i))/dt;
                // }

                // base_quat_.normalize();
                // prev_base_quat_.normalize();

                // Eigen::Vector3d tangent_vec = pinocchio::quaternion::log3(base_quat_); // prev_quat.inverse()*

                // for (size_t i = 0; i < 3; i++) {
                //     base_vel_.at(i + POS_VARS) = tangent_vec(i)/dt;
                // }

                // est_state_msg_.v_base.clear();
                // for (int i = 0; i < FLOATING_VEL_SIZE; i++) {
                //     est_state_msg_.v_base.emplace_back(base_vel_.at(i));
                // }
            }
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
            Eigen::Map<vector3_t> v_world_linear(base_vel_.data());
            Eigen::Map<vector3_t> v_world_angular(base_vel_.data() + 3);

            vector6_t local_vel;

            Eigen::Quaterniond base_quat(base_quat_.w(), base_quat_.x(), base_quat_.y(), base_quat_.z());
            local_vel.head<POS_VARS>() = base_quat.toRotationMatrix().transpose() * v_world_linear;
            local_vel.tail<POS_VARS>() = base_quat.toRotationMatrix().transpose() * v_world_angular;

            est_state_msg_.v_base.clear();
            for (int i = 0; i < FLOATING_VEL_SIZE; i++) {
                est_state_msg_.v_base.emplace_back(local_vel(i));
            }

            est_state_msg_.header.stamp = this->now();

            // Regardless of state of incoming data (i.e. even if dt <= 0)
            // TODO: Put back!
            // this->GetPublisher<obelisk_estimator_msgs::msg::EstimatedState>(this->est_pub_key_)->publish(est_state_msg_);
        } else {
            RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Waiting on sensor measurements to publish estimated state.");
        }

        return est_state_msg_;
    }

    void RobotEstimator::MakeTorsoMocapTransform() {
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

    void RobotEstimator::TrueStateCallback(const obelisk_sensor_msgs::msg::TrueSimState& msg) {
        for (size_t i = 0; i < FLOATING_VEL_SIZE; i++) {
            base_vel_[i] = msg.v_base[i];
        }
    }

    // TODO: Remove after debug
    // void RobotEstimator::ReceiveControlDebug(const obelisk_control_msgs::msg::PDFeedForward& msg) {
    //     auto time = this->now();
    //     obelisk_control_msgs::msg::PDFeedForward msg2;
    //     msg2 = msg;
    //     msg2.header.stamp = time;
    //     this->GetPublisher<obelisk_control_msgs::msg::PDFeedForward>("debug_pub")->publish(msg2);
    // }

} // namespace robot

int main(int argc, char* argv[]) {
    obelisk::utils::SpinObelisk<robot::RobotEstimator, rclcpp::executors::MultiThreadedExecutor>(
        argc, argv, "state_estimator");
}