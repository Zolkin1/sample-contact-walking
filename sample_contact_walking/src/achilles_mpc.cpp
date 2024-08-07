#include "obelisk_ros_utils.h"

#include "sample_contact_walking/achilles_mpc.h"

namespace achilles
{
    AchillesController::AchillesController(const std::string& name) 
    : obelisk::ObeliskController<obelisk_control_msgs::msg::PDFeedForward, obelisk_estimator_msgs::msg::EstimatedState>(name), recieved_first_state_(false) {
        // For now model the feet as point contacts
        contact_state_.contacts.emplace("left_ankle_pitch", torc::models::Contact(torc::models::PointContact, false));
        contact_state_.contacts.emplace("right_ankle_pitch", torc::models::Contact(torc::models::PointContact, false));

        //  Update model
        this->declare_parameter<std::string>("urdf_path", "");
        std::filesystem::path urdf_path(this->get_parameter("urdf_path").as_string());
        // wbc_.UpdateModelPath(urdf_path);

        std::string model_name = name + "_model";
        model_ = std::make_unique<torc::models::FullOrderRigidBody>(model_name, urdf_path);

        mpc_ = std::make_unique<torc::mpc::FullOrderMpc>(this->get_parameter("params_path").as_string(), urdf_path);
    }

    void AchillesController::UpdateXHat(const obelisk_estimator_msgs::msg::EstimatedState& msg) {
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

        // TODO: Update contact states properly
        contact_state_.contacts.at("left_ankle_pitch").state = true;
        contact_state_.contacts.at("right_ankle_pitch").state = true;

        if (q_.size() != model_->GetConfigDim()) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "recieved q does not match the size of the model");
        }

        if (v_.size() != model_->GetVelDim()) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "recieved v does not match the size of the model");
        }

        recieved_first_state_ = true;
    }

    obelisk_control_msgs::msg::PDFeedForward AchillesController::ComputeControl() {
        obelisk_control_msgs::msg::PDFeedForward msg;

        if (recieved_first_state_) {
            RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Computing first control.");
            // Create target state
            vectorx_t q_target = vectorx_t::Zero(model_->GetConfigDim());
            q_target << 0, 0, 0.97, 0, 0, 0, 0, 0, 0, -0.26, 0.65, -0.43, 0, 0, 0, 0, 0, 0, -0.26, 0.65, -0.43, 0, 0, 0, 0;
            vectorx_t v_target = vectorx_t::Zero(model_->GetVelDim());
            vectorx_t target_state = torc::models::FullOrderRigidBody::BuildState(q_target, v_target);
            
            vectorx_t feed_forward = vectorx_t::Zero(model_->GetNumInputs());

            // Create force_target
            vectorx_t force_target = vectorx_t::Constant(2, model_->GetMass()/2);

            // Create current state
            vectorx_t state = torc::models::FullOrderRigidBody::BuildState(q_, v_);

            // Call QP WBC controller
            // vectorx_t control = wbc_.ComputeControl(target_state, force_target, state, contact_state_);

            // Make the message
            vectorx_t u_mujoco = ConvertControlToMujocoU(q_target.tail(model_->GetNumInputs()), v_target.tail(model_->GetNumInputs()), feed_forward);
            ConvertEigenToStd(u_mujoco, msg.u_mujoco);
            ConvertEigenToStd(q_target.tail(model_->GetNumInputs()), msg.pos_target);
            ConvertEigenToStd(v_target.tail(model_->GetNumInputs()), msg.vel_target);
            ConvertEigenToStd(feed_forward, msg.feed_forward);

            if (msg.u_mujoco.size() != 54) {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Message's u_mujoco is incorrectly sized. Size: " << msg.u_mujoco.size());
            }

            this->GetPublisher<obelisk_control_msgs::msg::PDFeedForward>(this->ctrl_key_)->publish(msg);
        }

        return msg;
    }

    void AchillesController::ConvertEigenToStd(const vectorx_t& eig_vec, std::vector<double>& std_vec) {
        std_vec.clear();
        for (int i = 0; i < eig_vec.size(); i++) {
            std_vec.emplace_back(eig_vec(i));
        }
    }

    vectorx_t AchillesController::ConvertControlToMujocoU(const vectorx_t& pos_target, const vectorx_t& vel_target, const vectorx_t& feed_forward) {
        vectorx_t u(pos_target.size() + vel_target.size() + feed_forward.size());
        u << pos_target, vel_target, feed_forward;
        return u;
    }

} // namespace achilles


int main(int argc, char* argv[]) {
    obelisk::utils::SpinObelisk<achilles::AchillesController, rclcpp::executors::MultiThreadedExecutor>(
        argc, argv, "whole_body_controller");
}
