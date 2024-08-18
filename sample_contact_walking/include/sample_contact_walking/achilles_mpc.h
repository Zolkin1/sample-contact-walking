#include <visualization_msgs/msg/marker_array.hpp>

#include "rclcpp/rclcpp.hpp"
#include "obelisk_controller.h"

#include "full_order_mpc.h"

namespace achilles {
    using torc::mpc::vectorx_t;
    using torc::mpc::vector3_t;

    class AchillesController : public obelisk::ObeliskController<obelisk_control_msgs::msg::PDFeedForward, obelisk_estimator_msgs::msg::EstimatedState> {
        public:
            AchillesController(const std::string& name);
        protected:

        private:
            // Control functions
            obelisk_control_msgs::msg::PDFeedForward ComputeControl() override;
            // void ComputeMpc();

            // MPC Thread function
            void MpcThread();

            // State functions
            void UpdateXHat(const obelisk_estimator_msgs::msg::EstimatedState& msg) override;

            // Helpers
            void ConvertEigenToStd(const vectorx_t& eig_vec, std::vector<double>& std_vec);
            vectorx_t ConvertControlToMujocoU(const vectorx_t& pos_target, const vectorx_t& vel_target, const vectorx_t& feed_forward);

            // Viz
            void PublishTrajViz(const torc::mpc::Trajectory& traj, const std::vector<std::string>& viz_frames);
            void PublishTrajStateViz();

            // States
            enum ControllerState {
                SeekInitialCond,
                Mpc,
                NoOutput
                // TODO: Add a state for changing contact schedule
            };
            void TransitionState(const ControllerState& new_state);
            ControllerState GetState();

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

            ControllerState ctrl_state_;

            // Mutexes
            std::mutex est_state_mut_;
            std::mutex traj_out_mut_;
            std::mutex ctrl_state_mut_;

            // Estimated state vectors
            vectorx_t q_;        // position, quat (x, y, z, w), joints
            vectorx_t v_;

            // Target vectors
            vectorx_t q_target_;
            vectorx_t v_target_;

            vectorx_t q_ic_;
            vectorx_t v_ic_;

            // Trajectories
            torc::mpc::Trajectory traj_out_;
            torc::mpc::Trajectory traj_mpc_;

            double traj_start_time_;

            torc::models::RobotContactInfo contact_state_;
            std::unique_ptr<torc::mpc::FullOrderMpc> mpc_;
            std::unique_ptr<torc::models::FullOrderRigidBody> model_;
            torc::mpc::ContactSchedule contact_schedule_;

            // Threads
            std::thread mpc_thread_;
    };
} // namespace achilles