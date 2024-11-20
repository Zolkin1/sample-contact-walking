import numpy as np
from typing import List, Optional

from rclpy.executors import SingleThreadedExecutor

from obelisk_py.core.utils.ros import spin_obelisk

from obelisk_control_msgs.msg import PositionSetpoint
from rclpy.lifecycle import LifecycleState, TransitionCallbackReturn

from obelisk_py.core.control import ObeliskController
from obelisk_py.core.obelisk_typing import ObeliskControlMsg, ObeliskEstimatorMsg
from sample_contact_msgs.msg import ContactInfo, ContactPolytope, ContactSchedule

class ContactPlanner(ObeliskController):
    """Example position setpoint controller."""

    def __init__(self, node_name: str = "example_position_setpoint_controller") -> None:
        """Initialize the example position setpoint controller."""
        super().__init__(node_name)
        self.get_logger().info("INITIALIZING")
        # self.declare_parameter("test_param", "default_value")
        # self.get_logger().info(f"test_param: {self.get_parameter('test_param').get_parameter_value().string_value}")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the controller."""
        super().on_configure(state)
        self.get_logger().info("CONFIGURING")
        return TransitionCallbackReturn.SUCCESS

    def update_x_hat(self, x_hat_msg: ObeliskEstimatorMsg) -> None:
        """Update the state estimate.

        Parameters:
            x_hat_msg: The Obelisk message containing the state estimate.
        """
        self.q_est = np.concatenate((x_hat_msg.q_base, x_hat_msg.q_joints))
        # self.get_logger().info("Estiamted state: " + np.array2string(self.q_est))

    def compute_control(self):
        """
        Returns:
            obelisk_control_msg: The control message.
        """

        # setting the message
        cs_msg = ContactSchedule()
        cs_msg.header.stamp = self.get_clock().now().to_msg()

        contact_info = ContactInfo()

        # For now I will hardcode all of these settings
        contact_frames = ["left_toe", "left_heel", "right_toe", "right_heel"]

        num_contacts = 3

        for frame in contact_frames:
            # Assign the frame
            contact_info.robot_contact_frame = frame

            # Assign the swing times
            # TODO: Fill this in

            # Assign the contact polytopes
            # TODO: Change this
            for i in range(num_contacts):
                polytope = ContactPolytope()
                polytope.a_mat = [1., 0., 0., 1.]
                polytope.b_vec = [1., 1., -1., -1.]

                contact_info.polytopes.append(polytope)

            cs_msg.contact_info.append(contact_info)

        # TODO: Need to update obelisk

        # self.obk_publishers["pub_ctrl"].publish(cs_msg)
        # self.get_logger().info("Publishing a contact schedule")
        return cs_msg
    
def main(args: Optional[List] = None):
    spin_obelisk(args, ContactPlanner, SingleThreadedExecutor)


if __name__ == '__main__':
    main()
