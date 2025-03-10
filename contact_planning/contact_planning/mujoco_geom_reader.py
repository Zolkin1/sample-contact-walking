import numpy as np
import math
from scipy.spatial.transform import Rotation
from scipy import sparse
from typing import List, Optional

from rclpy.executors import SingleThreadedExecutor

from obelisk_py.core.utils.ros import spin_obelisk

from rclpy.lifecycle import LifecycleState, TransitionCallbackReturn

from obelisk_py.core.control import ObeliskController
from obelisk_estimator_msgs.msg import EstimatedState
from sample_contact_msgs.msg import ContactPolytopeArray, ContactPolytope
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion, Point
import mujoco

# TODO: Can I make this an obelisk sensor??
class MujocoGeomReader(ObeliskController):
    """Example position setpoint controller."""

    def __init__(self, node_name: str = "example_position_setpoint_controller") -> None:
        """Initialize the example position setpoint controller."""
        super().__init__(node_name, ContactPolytopeArray, EstimatedState)
        self.get_logger().info("INITIALIZING")

        # # Viz Publisher
        self.register_obk_publisher("viz_pub_setting", MarkerArray, "viz_pub_key")
        self.register_obk_timer("viz_timer_setting", self.viz_callback, "viz_timer_key")

        self.declare_parameter("mujoco_xml_path", "")
        self.mujoco_xml_path = self.get_parameter("mujoco_xml_path").value

        self.mujoco_model = mujoco.MjModel.from_xml_path(self.mujoco_xml_path)
        self.mujoco_data = mujoco.MjData(self.mujoco_model)
        
        # List of geom names to be considered for footholds
        self.declare_parameter("foothold_geoms", [""])
        self.foothold_geoms = self.get_parameter("foothold_geoms").value

        self.parse_geoms_from_mujoco(self.foothold_geoms)

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the controller."""
        super().on_configure(state)
        self.get_logger().info("CONFIGURING")
        return TransitionCallbackReturn.SUCCESS

    def update_x_hat(self, x_hat_msg) -> None:
        """Does nothing"""

    def compute_control(self):
        """
        Returns:
            obelisk_control_msg: The control message.
        """
        polytopes = ContactPolytopeArray()

        for j in range(len(self.b_vecs)):
            polytope = ContactPolytope()
            polytope.a_mat = [1., 0., 0., 1.]
            polytope.b_vec = self.b_vecs[j].tolist()
            polytope.height = self.heights[j]

            polytopes.polytopes.append(polytope)

        self.obk_publishers["pub_ctrl"].publish(polytopes)

        return polytopes
    
    # For now we are making the assumption that the terrain does not move
    def parse_geoms_from_mujoco(self, foothold_geoms: List[str]):
        # Read in the terrain from Mujoco
        self.b_vecs = []
        self.A_mats = []
        self.heights = []
        self.foothold_geoms = foothold_geoms

        for geom_name in foothold_geoms:
            geom_id = mujoco.mj_name2id(self.mujoco_model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)

            if (geom_id != -1):
                # Check type
                geom_type = self.mujoco_model.geom_type[geom_id]
                if geom_type != mujoco.mjtGeom.mjGEOM_BOX:
                    print(geom_type)
                    print(geom_name)
                    raise ValueError("The specified geom is not a box.")

                # For now this will just be considering the "top" surface
                # Create polytope representations for the relevant surfaces on the geoms
                half_sizes = self.mujoco_model.geom_size[geom_id]
                geom_pos = self.mujoco_model.geom_pos[geom_id]
                # geom_rot = self.mujoco_model.geom_quat[geom_id]


                self.heights.append(geom_pos[2] + half_sizes[2])
                self.get_logger().info(f"height: {self.heights[-1]}")
        
                # TODO: Deal with geometry at an angle
                
                # Get the smallest and largest x and y values
                # TODO: Support non-identity values here
                self.A_mats.append(np.array([[1, 0], [0, 1]]))

                self.b_vecs.append(np.array([max(half_sizes[0] + geom_pos[0], -half_sizes[0] + geom_pos[0]),
                                            max(half_sizes[1] + geom_pos[1], -half_sizes[1] + geom_pos[1]),
                                            min(half_sizes[0] + geom_pos[0], -half_sizes[0] + geom_pos[0]),
                                            min(half_sizes[1] + geom_pos[1], -half_sizes[1] + geom_pos[1])]))
                
                # Log this to a file so it can be parsed with the other logs

                # self.get_logger().info(f"b: {self.b_vecs[-1]}")
                self.get_logger().info(f"geom pos: {geom_pos[:2]}")
    
        # Write to CSV
        with open("mpc_logs/polytope_config.csv", "w") as f:
            for i, (A, b) in enumerate(zip(self.A_mats, self.b_vecs)):
                np.savetxt(f, [A.flatten()], delimiter=",", comments="")
                np.savetxt(f, [b], delimiter=",", comments="")

    def viz_callback(self):
        msg = MarkerArray()
        for geom_name in self.foothold_geoms:
            geom_id = mujoco.mj_name2id(self.mujoco_model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)

            if (geom_id != -1):
                # Check type
                geom_type = self.mujoco_model.geom_type[geom_id]
                if geom_type != mujoco.mjtGeom.mjGEOM_BOX:
                    print(geom_type)
                    print(geom_name)
                    raise ValueError("The specified geom is not a box.")

                # For now this will just be considering the "top" surface
                # Create polytope representations for the relevant surfaces on the geoms
                half_sizes = self.mujoco_model.geom_size[geom_id]
                geom_pos = self.mujoco_model.geom_pos[geom_id]
                # geom_rot = self.mujoco_model.geom_quat[geom_id]

                # TODO: Determine a way to encode the height of the polytope
                # For now assuming it is flush with the ground

                # TODO: Deal with geometry at an angle
            
                marker = Marker()
                marker.header.frame_id = "world"
                marker.header.stamp = self.get_clock().now().to_msg()

                marker.ns = "mujoco_scene_viz"
                marker.id = geom_id
                marker.action = Marker.MODIFY

                marker.pose.position = Point(x=geom_pos[0], 
                                            y=geom_pos[1], 
                                            z=geom_pos[2])

                # For now they are always aligned, TODO: Fix this in the future
                # marker.pose.orientation = self.mat_to_quat(self.rotmats[i])
                marker.pose.orientation.w = 1.
                marker.pose.orientation.x = 0.
                marker.pose.orientation.w = 0.
                marker.pose.orientation.z = 0.


                marker.color.a = 1.0
                marker.color.r = 0.69
                marker.color.g = 0.541
                marker.color.b = 0.094

                marker.scale.x = half_sizes[0] * 2
                marker.scale.y = half_sizes[1] * 2
                marker.scale.z = half_sizes[2] * 2
                marker.type = Marker.CUBE

                msg.markers.append(marker)

        self.obk_publishers["viz_pub_key"].publish(msg)
        # print("here")


def main(args: Optional[List] = None):
    spin_obelisk(args, MujocoGeomReader, SingleThreadedExecutor)


if __name__ == '__main__':
    main()
