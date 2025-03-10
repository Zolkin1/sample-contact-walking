#include <atomic>

#include <visualization_msgs/msg/marker_array.hpp>

#include "rclcpp/rclcpp.hpp"
#include "obelisk_controller.h"

#include "obelisk_sensor_msgs/msg/obk_force_sensor.hpp"
#include "sensor_msgs/msg/joy.hpp"

// #include "sample_contact_msgs/msg/contact_schedule.hpp"
#include "sample_contact_msgs/msg/contact_polytope_array.hpp"

// #include "full_order_mpc.h"
#include "hpipm_mpc.h"
#include "reference_generator.h"
#include "step_planner.h"
#include "wbc_controller.h"
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
            ~MpcController();

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
            // void ContactScheduleCallback(const sample_contact_msgs::msg::ContactSchedule& msg);
            void AddPeriodicContacts();
            void ContactPolytopeCallback(const sample_contact_msgs::msg::ContactPolytopeArray& msg);

            // Feet force sensor callback
            void ForceSensorCallback(const obelisk_sensor_msgs::msg::ObkForceSensor& msg);

            // Parse contact into
            void ParseContactParameters();

            // State functions
            void UpdateXHat(const obelisk_estimator_msgs::msg::EstimatedState& msg) override;

            // Helpers
            void ConvertEigenToStd(const vectorx_t& eig_vec, std::vector<double>& std_vec);
            vectorx_t ConvertControlToMujocoU(const vectorx_t& pos_target, const vectorx_t& vel_target, const vectorx_t& feed_forward);
            // std::pair<vectorx_t, vectorx_t> ReduceState(const vectorx_t& q, const vectorx_t& v, torc::models::FullOrderRigidBody model);
            void LogCurrentControl(const vectorx_t& q_control, const vectorx_t& v_control, const vectorx_t& tau, const vectorx_t& force, const vectorx_t& tau_mpc);
            void LogEigenVec(const vectorx_t& x);

            // Viz
            void PublishTrajViz(const torc::mpc::Trajectory& traj, const std::vector<std::string>& viz_frames);
            void PublishTrajStateViz();
            void MakeTargetTorsoMocapTransform();

            double PreperationPhase();
            std::pair<double, double> FeedbackPhase();

            // States
            enum ControllerState {
                SeekInitialCond,
                HoldInitialCond,
                Mpc,
                NoOutput
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
            std::vector<std::string> viz_polytope_frames_;

            // State flags
            bool recieved_first_state_;
            std::atomic<bool> first_mpc_computed_;
            std::atomic<bool> constant_vel_mode_;

            // Contact vel mode info
            double constant_vel_;

            // Viz info
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

            // Foot step polytopes
            std::mutex polytope_mutex_;
            // std::map<std::string, std::vector<torc::mpc::ContactInfo>> contact_polytopes_;
            bool recieved_polytope_;

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

            // std::shared_ptr<torc::mpc::FullOrderMpc> mpc_;
            std::unique_ptr<torc::models::FullOrderRigidBody> model_;               // Full model
            std::unique_ptr<torc::models::FullOrderRigidBody> mpc_model_;           // Potentially reduced model for the MPC
            std::unique_ptr<torc::models::FullOrderRigidBody> wbc_model_;           // Potentially reduced model for the MPC
            torc::mpc::ContactSchedule contact_schedule_;

            std::unique_ptr<torc::controller::WbcController> wbc_controller_;
            matrixx_t K_;

            // Reference Generator
            std::unique_ptr<torc::mpc::ReferenceGenerator> ref_gen_;
            std::unique_ptr<torc::mpc::SimpleTrajectory> q_base_target_;
            std::unique_ptr<torc::mpc::SimpleTrajectory> v_base_target_;

            // Step Planner
            std::unique_ptr<torc::step_planning::StepPlanner> step_planner_;
            std::map<std::string, std::vector<torc::step_planning::vector2_t>> nom_footholds_, projected_footholds_;   // For visualization

            std::shared_ptr<torc::mpc::MpcSettings> mpc_settings_;
            std::shared_ptr<torc::controller::WbcSettings> wbc_settings_;
            std::shared_ptr<torc::mpc::HpipmMpc> mpc_;

            // MPC Skipped joint indexes
            // TODO: Find a better way to do this
            std::vector<long int> mpc_skipped_joint_indexes_;
            std::vector<double> mpc_skipped_joint_vals_;
            std::vector<long int> wbc_skipped_joint_indexes_;

            // Threads
            std::thread mpc_thread_;
            std::thread sample_thread_;

            // broadcasters
            std::shared_ptr<tf2_ros::StaticTransformBroadcaster> torso_mocap_broadcaster_;

            // For hardware
            std::vector<std::string> control_joint_names_;
            std::vector<double> kp_;
            std::vector<double> kd_;

            // No stepping settings
            double state_no_step_threshold_;
            double command_no_step_threshold_;

            double time_offset_;
            std::ofstream log_file_;
            std::ofstream timing_log_file_;
            std::ofstream contact_schedule_log_file_;
            std::ofstream force_sensor_log_file_;
    };

    MpcController* MpcController::mujoco_sim_instance_ = nullptr;
} // namespace robot