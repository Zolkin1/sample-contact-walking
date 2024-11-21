import numpy as np
import math
from scipy.spatial.transform import Rotation
from typing import List, Optional

from rclpy.executors import SingleThreadedExecutor

from obelisk_py.core.utils.ros import spin_obelisk

from obelisk_control_msgs.msg import PositionSetpoint
from rclpy.lifecycle import LifecycleState, TransitionCallbackReturn

from obelisk_py.core.control import ObeliskController
from obelisk_py.core.obelisk_typing import ObeliskControlMsg, ObeliskEstimatorMsg
from sample_contact_msgs.msg import ContactInfo, ContactPolytope, ContactSchedule, CommandedTarget

class ContactPlanner(ObeliskController):
    """Example position setpoint controller."""

    def __init__(self, node_name: str = "example_position_setpoint_controller") -> None:
        """Initialize the example position setpoint controller."""
        super().__init__(node_name)
        self.get_logger().info("INITIALIZING")

        # Contact schedule publisher
        # TODO: Remove when obelisk is updated
        self.contact_schedule_pub_ = self.create_publisher(ContactSchedule, 'obelisk/g1/contact_schedule', 10, non_obelisk=True)

        # Joystick Subcriber
        self.joystick_sub_ = self.create_subscription(CommandedTarget, 'obelisk/g1/commanded_target', self.command_target_callback, 10, non_obelisk=True)

        self.declare_parameter("num_nodes", 32)
        self.num_nodes = self.get_parameter("num_nodes").get_parameter_value().integer_value

        self.q_target_base = np.zeros((7, self.num_nodes))
        self.q_target_base_global = self.q_target_base

        self.declare_parameter("default_polytope_size", 0.7)
        self.default_size = self.get_parameter("default_polytope_size").value

        self.received_state = False

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
        # self.get_logger().info(f"q est size: {self.q_est.shape}")
        # self.get_logger().info(f"q est base size: {len(x_hat_msg.q_base)}")
        # self.get_logger().info(f"q est joints size: {len(x_hat_msg.q_joints)}")

        if not self.received_state:
            # self.q_target = np.repeat(self.q_est, 32)
            for i in range(self.q_target_base.shape[1]):
                self.q_target_base[:,i] = x_hat_msg.q_base
            # TODO: remove
            self.get_logger().info("q_target_base: " + np.array2string(self.q_target_base))

        self.received_state = True
        # self.get_logger().info("Estiamted state: " + np.array2string(self.q_est))

    def compute_control(self):
        """
        Returns:
            obelisk_control_msg: The control message.
        """

        if self.received_state:
            # setting the message
            cs_msg = ContactSchedule()
            cs_msg.header.stamp = self.get_clock().now().to_msg()


            # For now I will hardcode all of these settings
            contact_frames = ["left_toe", "right_toe", "left_heel", "right_heel"]

            # TODO: Have this value update correctly
            num_contacts = 3
            frame_idx = 0

            # TODO: Change
            nodes_per_contact = math.floor(self.num_nodes/num_contacts)

            for frame in contact_frames:
                # self.get_logger().info("Frame: " + frame)
                contact_info = ContactInfo()

                # Assign the frame
                contact_info.robot_contact_frame = frame

                # Assign the swing times
                # TODO: Fill this in

                # Assign the contact polytopes
                # TODO: Change this
                # TODO: Make this the raibert heuristic
                for i in range(num_contacts):
                    polytope = ContactPolytope()
                    polytope.a_mat = [1., 0., 0., 1.]

                    # TODO: Add the x-y offset of the feet relative to the base frame
                    polytope.b_vec = [self.default_size + self.q_target_base_global[0, i*nodes_per_contact], 
                                        self.default_size + self.q_target_base_global[1, i*nodes_per_contact],
                                        -self.default_size + self.q_target_base_global[0, i*nodes_per_contact],
                                        -self.default_size + self.q_target_base_global[1, i*nodes_per_contact]]

                    contact_info.polytopes.append(polytope)

                cs_msg.contact_info.append(contact_info)
                frame_idx += 1

            # TODO: Need to update obelisk

            self.contact_schedule_pub_.publish(cs_msg)

            # self.obk_publishers["pub_ctrl"].publish(cs_msg)
            # self.get_logger().info("Publishing a contact schedule")
            # return cs_msg
    
    def command_target_callback(self, msg: CommandedTarget):
        """Parse the user joystick command."""
        # TODO: Make this not hard coded
        first_dt = 0.01
        other_dt = 0.025
        if self.received_state:
            self.q_target_base[:,0] = self.q_est[:7]
            self.q_target_base_global[:,0] = self.q_est[:7]

            v_target = np.array(msg.v_target)

            for i in range(1, self.num_nodes):
                dt = 0
                if i == 1:
                    dt = first_dt
                else:
                    dt = other_dt
                
                # Local frame
                self.q_target_base[:, i] = self.q_target_base[:, i-1] + dt*v_target[:7]

                # Global frame
                # Make the rotation matrix from the body frame to the world frame
                R = Rotation.from_quat([self.q_target_base_global[:, i-1][3], self.q_target_base_global[:, i-1][4], self.q_target_base_global[:, i-1][5], self.q_target_base_global[:, i-1][6]])

                # Linear Velocity integration
                v_linear = R.as_matrix()@v_target[:3]
                self.q_target_base_global[:3, i] = self.q_target_base_global[:3, i-1] + dt*v_linear

                # Angular velocity integration
                v_angular = v_target[3:6]
                angle = np.linalg.norm(v_angular) * dt
                if angle > 0:
                    axis = v_angular / np.linalg.norm(v_angular)
                    delta_rotation = Rotation.from_rotvec(axis * angle)
                else:
                    delta_rotation = Rotation.identity()

                full_rotation  = R * delta_rotation
                self.q_target_base_global[3:7, i] = full_rotation.as_quat()

            # self.get_logger().info("Updating target!")

def main(args: Optional[List] = None):
    spin_obelisk(args, ContactPlanner, SingleThreadedExecutor)


if __name__ == '__main__':
    main()
