#include <atomic>

#include <visualization_msgs/msg/marker_array.hpp>

#include "rclcpp/rclcpp.hpp"
#include "obelisk_controller.h"

#include "sensor_msgs/msg/joy.hpp"

#include "full_order_mpc.h"
// #include "cross_entropy.h"

namespace robot {
    using torc::mpc::vectorx_t;
    using torc::mpc::vector3_t;
    using torc::mpc::quat_t;
    using torc::mpc::matrix3_t;
    using torc::mpc::matrixx_t;

    class MpcController : public obelisk::ObeliskController<obelisk_control_msgs::msg::PDFeedForward, obelisk_estimator_msgs::msg::EstimatedState> {
        public:
            MpcController(const std::string& name);

            // ------ Mujoco Debug ----- //
            static MpcController* mujoco_sim_instance_;
            // ------ Mujoco Debug ----- //
        protected:

        private:
            // Control functions
            obelisk_control_msgs::msg::PDFeedForward ComputeControl() override;
            // void ComputeMpc();

            // MPC Thread function
            void MpcThread();

            // MPC Targets
            // TODO: Update when I have joystick access
            void UpdateMpcTargets(const vectorx_t& q);

            // Updating contact info
            void UpdateContactPolytopes();

            // Joystick interface
            void JoystickCallback(const sensor_msgs::msg::Joy& msg);

            // Contact schedule
            void AddPeriodicContacts();

            // Parse contact into
            void ParseContactParameters();

            // State functions
            void UpdateXHat(const obelisk_estimator_msgs::msg::EstimatedState& msg) override;

            // Helpers
            void ConvertEigenToStd(const vectorx_t& eig_vec, std::vector<double>& std_vec);
            vectorx_t ConvertControlToMujocoU(const vectorx_t& pos_target, const vectorx_t& vel_target, const vectorx_t& feed_forward);

            // Viz
            void PublishTrajViz(const torc::mpc::Trajectory& traj, const std::vector<std::string>& viz_frames);
            void PublishTrajStateViz();
            void MakeTargetTorsoMocapTransform();

            // States
            enum ControllerState {
                SeekInitialCond,
                Mpc,
                NoOutput
                // TODO: Add a state for changing contact schedule
            };
            void TransitionState(const ControllerState& new_state);
            ControllerState GetState();
            std::string GetStateString(const ControllerState& state);

            // ---------- Member Variables ---------- //
            static constexpr int FLOATING_POS_SIZE = 7;
            static constexpr int FLOATING_VEL_SIZE = 6;
            static constexpr int POS_VARS = 3;
            static constexpr int QUAT_VARS = 4;

            // Viz information
            std::vector<std::string> viz_frames_;

            // State flags
            bool recieved_first_state_;
            bool first_mpc_computed_;

            bool viz_forces_;
            double scale_forces_;
            std::vector<std::string> force_frames_;

            ControllerState ctrl_state_;

            std::atomic<bool> print_timings_;

            // Mutexes
            std::mutex est_state_mut_;
            std::mutex traj_out_mut_;
            std::mutex ctrl_state_mut_;
            std::mutex target_state_mut_;

            // Estimated state vectors
            vectorx_t q_;        // position, quat (x, y, z, w), joints
            vectorx_t v_;

            // Target vectors
            std::optional<torc::mpc::SimpleTrajectory> q_target_;
            std::optional<torc::mpc::SimpleTrajectory> v_target_;
            double z_target_;
            // vectorx_t q_target_;
            // vectorx_t v_target_;
            bool fixed_target_;
            bool controller_target_;

            vectorx_t q_ic_;
            vectorx_t v_ic_;

            // Trajectories
            torc::mpc::Trajectory traj_out_;
            torc::mpc::Trajectory traj_mpc_;

            double traj_start_time_;

            // Contact schedule
            double swing_time_;
            double first_swing_time_;
            double double_stance_time_;
            bool right_foot_first_;
            std::vector<std::string> right_frames_;
            std::vector<std::string> left_frames_;
            double next_right_insertion_time_;
            double next_left_insertion_time_;

            std::shared_ptr<torc::mpc::FullOrderMpc> mpc_;
            std::unique_ptr<torc::models::FullOrderRigidBody> model_;               // Full model
            std::unique_ptr<torc::models::FullOrderRigidBody> mpc_model_;           // Potentially reduced model for the MPC
            torc::mpc::ContactSchedule contact_schedule_;

            // MPC Skipped joint indexes
            // TODO: Find a better way to do this
            std::vector<long int> skipped_joint_indexes_;

            // Threads
            std::thread mpc_thread_;
            std::thread sample_thread_;

            // broadcasters
            std::shared_ptr<tf2_ros::StaticTransformBroadcaster> torso_mocap_broadcaster_;
    };

    MpcController* MpcController::mujoco_sim_instance_ = nullptr;
} // namespace robot