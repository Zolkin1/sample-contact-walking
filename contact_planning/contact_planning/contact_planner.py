import numpy as np
import math
from scipy.spatial.transform import Rotation
from scipy import sparse
from typing import List, Optional

from rclpy.executors import SingleThreadedExecutor

from obelisk_py.core.utils.ros import spin_obelisk

from obelisk_control_msgs.msg import PositionSetpoint
from rclpy.lifecycle import LifecycleState, TransitionCallbackReturn

from obelisk_py.core.control import ObeliskController
from obelisk_py.core.obelisk_typing import ObeliskControlMsg, ObeliskEstimatorMsg
from sample_contact_msgs.msg import ContactInfo, ContactPolytope, ContactSchedule, CommandedTarget

import mujoco
import osqp

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


        self.declare_parameter("mujoco_xml_path", "")
        self.mujoco_xml_path = self.get_parameter("mujoco_xml_path").value

        self.mujoco_model = mujoco.MjModel.from_xml_path(self.mujoco_xml_path)
        self.mujoco_data = mujoco.MjData(self.mujoco_model)
        
        # List of geom names to be considered for footholds
        self.declare_parameter("foothold_geoms", [""])
        self.foothold_geoms = self.get_parameter("foothold_geoms").value

        self.parse_geoms_from_mujoco(self.foothold_geoms)

        # OSQP
        self.osqp_prob = osqp.OSQP()
        self.osqp_setup = False

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
                    # polytope.b_vec = [self.default_size + self.q_target_base_global[0, i*nodes_per_contact], 
                    #                     self.default_size + self.q_target_base_global[1, i*nodes_per_contact],
                    #                     -self.default_size + self.q_target_base_global[0, i*nodes_per_contact],
                    #                     -self.default_size + self.q_target_base_global[1, i*nodes_per_contact]]

                    # Compute the closest foothold
                    # Compute this relative to the middle of the foot so that we don't get two seperate polytopes for the toe and heel
                    desired_foothold = self.q_target_base_global[:2, i*nodes_per_contact] # TODO: Add the foot offset
                    idx = self.compute_closest_foothold(desired_foothold)

                    polytope.b_vec = self.b_vecs[idx].tolist()
                    contact_info.polytopes.append(polytope)

                cs_msg.contact_info.append(contact_info)
                frame_idx += 1

            # TODO: Need to update obelisk

            self.contact_schedule_pub_.publish(cs_msg)

            # self.obk_publishers["pub_ctrl"].publish(cs_msg)
            # self.get_logger().info("Publishing a contact schedule")
            # return cs_msg
    
    def command_target_callback(self, msg: CommandedTarget):
        """Parse the commanded target."""
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

    def compute_closest_foothold(self, desired_foothold):
        # Iterate through each of the candidate polytopes already generated
        min_distances = []
        for i in range(len(self.b_vecs)):
            # Compute the distance between the desired point and the polytopes
            # Compute with OSQP (need to time it...)
            min_distances.append(self.compute_min_distance(self.A_mats[i], self.b_vecs[i], desired_foothold))

        # Iterate through the list and choose the geom coressponding to the geom with the smallest cost
        min_dist = 100
        min_idx = 100
        for i in range(len(min_distances)):
            # self.get_logger().info(f"Dist: {min_distances[i]}")
            if min_distances[i] < min_dist:
                min_dist = min_distances[i]
                min_idx = i

        # Return the index
        return min_idx
    
    # For now we are making the assumption that the terrain does not move
    def parse_geoms_from_mujoco(self, foothold_geoms: List[str]):
        # Read in the terrain from Mujoco
        self.b_vecs = []
        self.A_mats = []

        for geom_name in foothold_geoms:
            geom_id = mujoco.mj_name2id(self.mujoco_model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)

            # Check type
            geom_type = self.mujoco_model.geom_type[geom_id]
            if geom_type != mujoco.mjtGeom.mjGEOM_BOX:
                raise ValueError("The specified geom is not a box.")

            # For now this will just be considering the "top" surface
            # Create polytope representations for the relevant surfaces on the geoms
            half_sizes = self.mujoco_model.geom_size[geom_id]
            geom_pos = self.mujoco_model.geom_pos[geom_id]
            # geom_rot = self.mujoco_model.geom_quat[geom_id]

            # TODO: Determine a way to encode the height of the polytope
            # For now assuming it is flush with the ground

            # TODO: Deal with geometry at an angle
            
            # Get the smallest and largest x and y values
            # TODO: Support non-identity values here
            self.A_mats.append(np.array([[1, 0], [0, 1]]))

            self.b_vecs.append(np.array([max(half_sizes[0] + geom_pos[0], -half_sizes[0] + geom_pos[0]),
                                         max(half_sizes[1] + geom_pos[1], -half_sizes[1] + geom_pos[1]),
                                         min(half_sizes[0] + geom_pos[0], -half_sizes[0] + geom_pos[0]),
                                         min(half_sizes[1] + geom_pos[1], -half_sizes[1] + geom_pos[1])]))
            
            # self.get_logger().info(f"b: {self.b_vecs[-1]}")
            self.get_logger().info(f"geom pos: {geom_pos[:2]}")
    
    def compute_min_distance(self, A, b, p):
        """
        Compute min (x - p)^2 s.t. Ax <= b using osqp
        """
        q = -2*p
        l = b[2:]
        u = b[:2]
        # self.get_logger().info(f"l: {l}")
        # self.get_logger().info(f"u: {u}")

        P = sparse.csc_matrix([[1, 0], [0, 1]])
        A_sparse = sparse.csc_matrix(A)
        # self.get_logger().info(f"A: {A}")

        if not self.osqp_setup:
            self.osqp_prob.setup(P, q, A_sparse, l, u, verbose=False)
            self.osqp_setup = True
        else:
            # TODO: Support updating A when needed
            self.osqp_prob.update(q=q, l=l, u=u)

        res = self.osqp_prob.solve()

        # self.get_logger().info(f"result: {res.x}")
        return res.info.obj_val

def main(args: Optional[List] = None):
    spin_obelisk(args, ContactPlanner, SingleThreadedExecutor)


if __name__ == '__main__':
    main()
