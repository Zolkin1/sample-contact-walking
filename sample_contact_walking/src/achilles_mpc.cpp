#include "obelisk_ros_utils.h"

#include "sample_contact_walking/achilles_mpc.h"

namespace achilles
{
    AchillesController::AchillesController(const std::string& name) 
    : obelisk::ObeliskController<obelisk_control_msgs::msg::PDFeedForward, obelisk_estimator_msgs::msg::EstimatedState>(name), 
        recieved_first_state_(false), first_mpc_computed_(false), traj_start_time_(0) {
        // For now model the feet as point contacts
        contact_state_.contacts.emplace("left_ankle_pitch", torc::models::Contact(torc::models::PointContact, false));
        contact_state_.contacts.emplace("right_ankle_pitch", torc::models::Contact(torc::models::PointContact, false));

        this->RegisterObkTimer("timer_mpc_setting", "mpc_timer", std::bind(&AchillesController::ComputeMpc, this));

        //  Update model
        this->declare_parameter<std::string>("urdf_path", "");
        std::filesystem::path urdf_path(this->get_parameter("urdf_path").as_string());

        std::string model_name = name + "_model";
        model_ = std::make_unique<torc::models::FullOrderRigidBody>(model_name, urdf_path);

        mpc_ = std::make_unique<torc::mpc::FullOrderMpc>("achilles_mpc_obk", this->get_parameter("params_path").as_string(), urdf_path);
        RCLCPP_INFO_STREAM(this->get_logger(), "MPC Created...");

        mpc_->Configure();
        RCLCPP_INFO_STREAM(this->get_logger(), "MPC Configured...");

        traj_out_.UpdateSizes(model_->GetConfigDim(), model_->GetVelDim(), model_->GetNumInputs(), mpc_->GetContactFrames(), mpc_->GetNumNodes());

        vectorx_t q_neutral = model_->GetNeutralConfig();
        q_neutral(2) += 0.321;
        traj_out_.SetDefault(q_neutral);
        traj_mpc_ = traj_out_;

        mpc_->SetWarmStartTrajectory(traj_out_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Warm start trajectory created...");

        // TODO: Remove -- add a dummy contact schedule
        torc::mpc::ContactSchedule cs(mpc_->GetContactFrames());
        const double contact_time = 0.3;
        double time = 0;
        for (int i = 0; i < 3; i++) {
            if (i % 2 != 0) {
                cs.InsertContact("right_foot", time, time + contact_time);
                cs.InsertContact("right_hand", time, time + contact_time);
            } else {
                cs.InsertContact("left_foot", time, time + contact_time);
                cs.InsertContact("left_hand", time, time + contact_time);
            }
            time += contact_time;
        }
        mpc_->UpdateContactSchedule(cs);
    }

    void AchillesController::UpdateXHat(const obelisk_estimator_msgs::msg::EstimatedState& msg) {
        // Get the mutex to protect the states
        std::lock_guard<std::mutex> lock(est_state_mut_);
        
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

        // TODO: Update contact in MPC with SetContactSchedule
        // TODO: Update contact states properly
        contact_state_.contacts.at("left_ankle_pitch").state = true;
        contact_state_.contacts.at("right_ankle_pitch").state = true;

        if (q_.size() != model_->GetConfigDim()) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "received q does not match the size of the model");
        }

        if (v_.size() != model_->GetVelDim()) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "received v does not match the size of the model");
        }

        recieved_first_state_ = true;
    }

    void AchillesController::ComputeMpc() {
        if (recieved_first_state_) {
            RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Computing first trajectory.");
            vectorx_t q, v;
            {
                // Get the mutex to protect the states
                std::lock_guard<std::mutex> lock(est_state_mut_);

                // Create current state
                q = q_;
                v = v_;
            }

            // RCLCPP_INFO_STREAM(this->get_logger(), "q: " << q.transpose());
            // RCLCPP_INFO_STREAM(this->get_logger(), "v: " << v.transpose());

            // Get the current time
            double time = this->get_clock()->now().seconds();
            RCLCPP_INFO_STREAM(this->get_logger(), "Time: " << time);

            mpc_->Compute(q, v, traj_mpc_);
            {
                // Get the traj mutex to protect it
                std::lock_guard<std::mutex> lock(traj_out_mut_);
                traj_out_ = traj_mpc_;

                // Assign time time too
                traj_start_time_ = time;
            }
            RCLCPP_INFO_STREAM(this->get_logger(), "MPC Computation Completed!");
            first_mpc_computed_ = true;
        }
    }

    obelisk_control_msgs::msg::PDFeedForward AchillesController::ComputeControl() {
        // This callback does not need any access to the state as it outputs PD
        //  setpoints from the trajectory.

        obelisk_control_msgs::msg::PDFeedForward msg;

        if (first_mpc_computed_) {
            RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Publishing first control.");
            {
                // Get the traj mutex to protect it
                std::lock_guard<std::mutex> lock(traj_out_mut_);

                double time = this->get_clock()->now().seconds();
                // TODO: Do I need to use nanoseconds?
                double time_into_traj = time - traj_start_time_;
                // RCLCPP_INFO_STREAM(this->get_logger(), "Time into traj: " << time_into_traj);

                vectorx_t q, v, tau;
                traj_out_.GetConfigInterp(time_into_traj, q);
                traj_out_.GetVelocityInterp(time_into_traj, v);
                traj_out_.GetTorqueInterp(time_into_traj, tau);

                // --- TODO: Remove! -- This is a safety in case the interpolation time is too large
                if (q.size() == 0) {
                    q = model_->GetNeutralConfig();
                }

                if (v.size() == 0) {
                    v = torc::mpc::vectorx_t::Zero(model_->GetVelDim());
                }

                if (tau.size() == 0) {
                    tau = torc::mpc::vectorx_t::Zero(model_->GetNumInputs());
                }
                // ---

                // Make the message
                vectorx_t u_mujoco = ConvertControlToMujocoU(q.tail(model_->GetNumInputs()),
                    v.tail(model_->GetNumInputs()), tau);
                ConvertEigenToStd(u_mujoco, msg.u_mujoco);
                ConvertEigenToStd(q.tail(model_->GetNumInputs()), msg.pos_target);
                ConvertEigenToStd(v.tail(model_->GetNumInputs()), msg.vel_target);
                ConvertEigenToStd(tau, msg.feed_forward);
            }
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
