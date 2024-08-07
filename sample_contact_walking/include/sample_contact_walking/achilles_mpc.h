#include "rclcpp/rclcpp.hpp"
#include "obelisk_controller.h"

// #include "whole_body_qp_controller.h"
#include "full_order_mpc.h"

namespace achilles {
    using torc::mpc::vectorx_t;

    class AchillesController : public obelisk::ObeliskController<obelisk_control_msgs::msg::PDFeedForward, obelisk_estimator_msgs::msg::EstimatedState> {
        public:
            AchillesController(const std::string& name);
        protected:

        private:
            obelisk_control_msgs::msg::PDFeedForward ComputeControl() override;
            void UpdateXHat(const obelisk_estimator_msgs::msg::EstimatedState& msg) override;

            void ConvertEigenToStd(const vectorx_t& eig_vec, std::vector<double>& std_vec);

            vectorx_t ConvertControlToMujocoU(const vectorx_t& pos_target, const vectorx_t& vel_target, const vectorx_t& feed_forward);
            // ---------- Member Variables ---------- //
            static constexpr int FLOATING_POS_SIZE = 7;
            static constexpr int FLOATING_VEL_SIZE = 6;
            static constexpr int POS_VARS = 3;
            static constexpr int QUAT_VARS = 4;

            // State flags
            bool recieved_first_state_;

            // Estimated state vectors
            vectorx_t q_;        // position, quat (x, y, z, w), joints
            vectorx_t v_;

            torc::models::RobotContactInfo contact_state_;
            // torc::mpc::
            // torc::controllers::WholeBodyQPController wbc_;
            std::unique_ptr<torc::models::FullOrderRigidBody> model_;
    };
} // namespace achilles