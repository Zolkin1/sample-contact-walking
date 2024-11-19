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
        self.declare_parameter("test_param", "default_value")
        self.get_logger().info(f"test_param: {self.get_parameter('test_param').get_parameter_value().string_value}")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the controller."""
        super().on_configure(state)
        self.joint_pos = None
        return TransitionCallbackReturn.SUCCESS

    def update_x_hat(self, x_hat_msg: ObeliskEstimatorMsg) -> None:
        """Update the state estimate.

        Parameters:
            x_hat_msg: The Obelisk message containing the state estimate.
        """
        pass  # do nothing

    def compute_control(self) -> ObeliskControlMsg:
        """
        Returns:
            obelisk_control_msg: The control message.
        """

        # setting the message
        cs_msg = ContactSchedule()
        cs_msg.header.stamp = self.get_clock().now()

        # TODO: Change
        cs_msg.contact_info.resize(2)

        contact_info = ContactInfo()

        # For now I will hardcode all of these settings
        contact_frames = ["left_toe", "left_heel", "right_toe", "right_heel"]

        for frame in contact_frames:
            contact_info.contact_frame = frame

        self.obk_publishers["pub_ctrl"].publish(cs_msg)
        return cs_msg  # type: ignore
    
def main(args: Optional[List] = None):
    print('Hi from contact_planning.')
    spin_obelisk(args, ContactPlanner, SingleThreadedExecutor)


if __name__ == '__main__':
    main()
