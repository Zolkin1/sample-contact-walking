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
from obelisk_estimator_msgs.msg import EstimatedState
from sample_contact_msgs.msg import ContactInfo, ContactPolytope, ContactSchedule, CommandedTarget
from sensor_msgs.msg import Joy

import mujoco
import osqp

class ContactPlanner(ObeliskController):
    """Example position setpoint controller."""

    def __init__(self, node_name: str = "example_position_setpoint_controller") -> None:
        """Initialize the example position setpoint controller."""
        super().__init__(node_name, ContactSchedule, EstimatedState)
        self.get_logger().info("INITIALIZING")

        # # Joystick Subscriber
        self.register_obk_subscription("joystick_sub_setting", self.joystick_callback, Joy)
    
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

        # TODO: Figure out how to get this from the other node
        self.declare_parameter("first_step_time", 0.0)
        self.declare_parameter("swing_time", 0.3)
        self.declare_parameter("right_foot_frames", [""])
        self.declare_parameter("left_foot_frames", [""])

        self.first_step_time = self.get_parameter("first_step_time").value
        self.swing_time = self.get_parameter("swing_time").value
        self.right_foot_frames = self.get_parameter("right_foot_frames").value
        self.left_foot_frames = self.get_parameter("left_foot_frames").value

        self.declare_parameter("foot_offsets", [0.0])
        self.foot_offset = np.array(self.get_parameter("foot_offsets").value)
        self.foot_offset = np.reshape(self.foot_offset, (len(self.right_foot_frames) + len(self.left_foot_frames), 2))
        self.get_logger().error(f"foot offsets: {self.foot_offset}")

        self.declare_parameter("node_group_1_n", 1)
        self.declare_parameter("node_group_2_n", 31)
        self.declare_parameter("node_dt_1", 0.01)
        self.declare_parameter("node_dt_2", 0.025)
        self.node_group_1_n = self.get_parameter("node_group_1_n").value
        self.node_group_2_n = self.get_parameter("node_group_2_n").value
        self.node_dt_1 = self.get_parameter("node_dt_1").value
        self.node_dt_2 = self.get_parameter("node_dt_2").value

        self.dt_vec = []
        for i in range(self.num_nodes):
            if i < self.node_group_1_n:
                self.dt_vec.append(self.node_dt_1)
            else:
                self.dt_vec.append(self.node_dt_2)

        if self.node_group_1_n + self.node_group_2_n != self.num_nodes:
            raise Exception("Invalid node dt groups!")

        self.declare_parameter("polytope_margin", 0.)
        self.polytope_margin = self.get_parameter("polytope_margin").value
        if self.polytope_margin < 0:
            raise Exception(f"Polytope margin should be positive! margin: {self.polytope_margin}")

        # OSQP
        self.osqp_prob = osqp.OSQP()
        self.osqp_setup = False

        self.received_state = False
        self.mpc_start_time = -1

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the controller."""
        super().on_configure(state)
        self.get_logger().info("CONFIGURING")
        return TransitionCallbackReturn.SUCCESS

    def update_x_hat(self, x_hat_msg) -> None:
        """Update the state estimate.

        Parameters:
            x_hat_msg: The Obelisk message containing the state estimate.
        """
        # TODO: DOUBLE CHECK THIS
        self.q_est = np.concatenate((x_hat_msg.q_base, x_hat_msg.q_joints))
        # self.get_logger().info(f"q est size: {self.q_est.shape}")
        # self.get_logger().info(f"q est base size: {len(x_hat_msg.q_base)}")
        # self.get_logger().info(f"q est joints size: {len(x_hat_msg.q_joints)}")

        if not self.received_state:
            # self.q_target = np.repeat(self.q_est, 32)
            for i in range(self.q_target_base.shape[1]):
                self.q_target_base[:,i] = x_hat_msg.q_base

        if np.linalg.norm(self.q_est[3:7]) > 0.9:
            self.received_state = True

        if np.linalg.norm(self.q_est[3:7]) < 0.9:
            self.received_state = False

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
            # contact_frames = ["left_toe", "right_toe", "left_heel", "right_heel"]
            contact_frames = self.right_foot_frames + self.left_foot_frames

            num_contacts = []

            current_time_seconds =  self.get_clock().now().nanoseconds*1e-9
            # self.get_logger().info(f"current time seconds: {current_time_seconds}")
            for i in range(len(contact_frames)):
                frame = contact_frames[i]

                # self.get_logger().info("Frame: " + frame)
                contact_info = ContactInfo()

                # Assign the frame
                contact_info.robot_contact_frame = frame

                # Assign the swing times
                contact_info.swing_times = self.compute_swing_times(frame, current_time_seconds)
                for i in range(len(contact_info.swing_times)):
                    contact_info.swing_times[i] -= current_time_seconds
                num_contacts.append(math.floor(len(contact_info.swing_times)/2) + 1)
                # self.get_logger().info(f"{frame} swing times: {contact_info.swing_times}")

                cs_msg.contact_info.append(contact_info)

            for j in range(len(contact_frames)):
                frame = contact_frames[j]
                # Assign the contact polytopes
                # TODO: Make this the raibert heuristic

                for i in range(num_contacts[j]):
                    polytope = ContactPolytope()
                    polytope.a_mat = [1., 0., 0., 1.]

                    contact_mid_time = self.get_contact_mid_times(current_time_seconds, i, cs_msg.contact_info[j].swing_times)
                    # contact_mid_time_rel = contact_mid_time - current_time_seconds

                    desired_base_pos = self.get_position(0) # TODO: Is this how I want to handle this?
                    if (contact_mid_time > 0):
                        desired_base_pos = self.get_position(contact_mid_time)

                    # self.get_logger().info(f"Frame: {frame}")
                    # self.get_logger().info(f"contact mid time: {contact_mid_time}")
                    # self.get_logger().info(f"desired_base_pos: {desired_base_pos[:2]}")

                    # Compute the closest foothold
                    # Compute this relative to the middle of the foot so that we don't get two seperate polytopes for the toe and heel
                    # print(f"foot offset: {self.foot_offset[j,:]}")
                    # print(f"des base pos: {desired_base_pos}")
                    # TODO: Double check this
                    R = Rotation.from_quat([desired_base_pos[3], desired_base_pos[4], desired_base_pos[5], desired_base_pos[6]])
                    desired_foothold = desired_base_pos[:2] + (R.as_matrix()[:2,:2]@self.foot_offset[j,:].transpose()).transpose()

                    # self.get_logger().info(f"desired foothold: {desired_foothold[:2]}\n")

                    idx = self.compute_closest_foothold(desired_foothold)


                    polytope.b_vec = self.b_vecs[idx].tolist()
                    # print("frame: " + frame)
                    # print(f"b: {polytope.b_vec}")
                    cs_msg.contact_info[j].polytopes.append(polytope)

            self.obk_publishers["pub_ctrl"].publish(cs_msg)
            # self.get_logger().info("Publishing a contact schedule")
            return cs_msg
    
    def command_target_callback(self, msg: CommandedTarget):
        """Parse the commanded target."""
        if self.received_state:
            self.q_target_base[:,0] = self.q_est[:7]
            self.q_target_base_global[:,0] = self.q_est[:7]

            v_target = np.array(msg.v_target)

            for i in range(1, self.num_nodes):
                dt = self.dt_vec[i]
                
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
            # TODO: Implement kinematic cost
            # min_distances[-1] += self.compute_fk_cost(desired_foothold)

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

        # Add in the polytope safety margin
        l = l + [self.polytope_margin, self.polytope_margin]
        u = u - [self.polytope_margin, self.polytope_margin]

        P = sparse.csc_matrix([[2, 0], [0, 2]])
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

    def joystick_callback(self, msg: Joy):
        X_button = 2

        if msg.buttons[X_button]:
            self.mpc_start_time = self.get_clock().now().nanoseconds*1e-9
            self.get_logger().info("JOY STICK RECIEVED") # TODO Remove

        return

    def compute_swing_times(self, frame: str, current_time_seconds: float):
        swing_times = []
        if self.mpc_start_time > 0:
            time_since_start = current_time_seconds - self.mpc_start_time

            if time_since_start < self.first_step_time:
                return swing_times

            time_since_first_swing = time_since_start - self.first_step_time
            # self.get_logger().info(f"time since first swing: {time_since_first_swing}")
            # Swing once every 2*swing_time seconds

            # Get number of full gaits:
            num_gaits = math.floor(time_since_first_swing/(2*self.swing_time))

            time_into_full_gait = time_since_first_swing - (num_gaits * (2*self.swing_time))
            # self.get_logger().info(f"time into full gait: {time_into_full_gait}")

            # self.get_logger().info(f"current_time_seconds: {current_time_seconds}")

            gait_start_time = current_time_seconds - time_into_full_gait

            # if gait_start_time > 10:
                # self.get_logger().info(f"gait_start_time: {gait_start_time}")

            # if time_into_full_gait > 10:
                # self.get_logger().info(f"time_into_full_gait: {time_into_full_gait}")

            if time_into_full_gait < self.swing_time:
                # Assume the right foot is first
                if frame in self.right_foot_frames:
                    swing_times.append(gait_start_time)
                else:
                    swing_times.append(gait_start_time + self.swing_time)
            else:
                # Left foot is in swing
                if frame in self.left_foot_frames:
                    swing_times.append(gait_start_time + self.swing_time)
                else:
                    swing_times.append(gait_start_time)

            while swing_times[-1] < current_time_seconds + sum(self.dt_vec):
                swing_times.append(swing_times[-1] + self.swing_time)

            if len(swing_times) % 2 == 1:
                swing_times.append(swing_times[-1] + self.swing_time)

        return swing_times
    
    def get_node(self, time):
        if time < 0:
            raise Exception(f"[get_node] time is too small! time: {time}")
        
        if time > sum(self.dt_vec):
            raise Exception(f"[get_node] time is too large! time:{time}")
        
        t = 0
        i = 0
        while (t <= time):
            t = t + self.dt_vec[i]
            i += 1

        return i-1

    def get_contact_mid_times(self, current_time_seconds: float, contact_num: int, swing_times):
        # self.get_logger().info(f"contact num: {contact_num}, swing_times len: {len(swing_times)}")
        if len(swing_times) > 0:
            if contact_num == 0:
                # The first contact is always before the first recorded swing
                return swing_times[0] - self.swing_time/2. # TODO: Adjust this
            elif contact_num <= len(swing_times)/2. - 1:
                # We are in an intermediate contact
                # Get the mid point by using the surrounding swing times
                touch_down = swing_times[contact_num*2]
                lift_off = swing_times[contact_num*2 + 1]
                contact_mid_time = (lift_off + touch_down)/2.
                # self.get_logger().info(f"contact mid time {contact_mid_time}")
                return contact_mid_time
            else:
                # After the last recorded swing
                return swing_times[-1] + self.swing_time/2. # TODO: Adjust this number
        else:
            return 0.2 # TODO: Adjust this number

    def get_position(self, time: float):
        node = 0
        if (time < sum(self.dt_vec)):
            node = self.get_node(time)
        else:
            node = self.num_nodes - 1
        
        return self.q_target_base[:, node]

def main(args: Optional[List] = None):
    spin_obelisk(args, ContactPlanner, SingleThreadedExecutor)


if __name__ == '__main__':
    main()
