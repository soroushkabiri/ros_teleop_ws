#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs


class MapLocalizationNode(Node):
    def __init__(self):
        super().__init__("map_localization_node")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            Odometry,
            '/rtabmap/odom',
            self.odom_callback,
            10
        )

        self.publisher = self.create_publisher(
            Odometry,
            '/map_localization',
            10
        )


    def odom_callback(self, msg: Odometry):
        try:
            # Lookup transform map -> odom
            transform = self.tf_buffer.lookup_transform(
                'map',
                'odom',
                rclpy.time.Time()
            )

            # Extract values
            ox = msg.pose.pose.position.x
            oy = msg.pose.pose.position.y
            oz = msg.pose.pose.position.z
            ow = msg.pose.pose.orientation.w
            ox_q = msg.pose.pose.orientation.x
            oy_q = msg.pose.pose.orientation.y
            oz_q = msg.pose.pose.orientation.z

            # Transform translation
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            tz = transform.transform.translation.z

            # Quaternion of transform
            t_qx = transform.transform.rotation.x
            t_qy = transform.transform.rotation.y
            t_qz = transform.transform.rotation.z
            t_qw = transform.transform.rotation.w

            # Convert tf into numpy
            import numpy as np
            from scipy.spatial.transform import Rotation as R

            # Apply transform: map_pose = T_map_odom * odom_pose
            rot_t = R.from_quat([t_qx, t_qy, t_qz, t_qw])
            rot_o = R.from_quat([ox_q, oy_q, oz_q, ow])

            pos_o = np.array([ox, oy, oz])
            pos_map = rot_t.apply(pos_o) + np.array([tx, ty, tz])
            rot_map = rot_t * rot_o
            q_map = rot_map.as_quat()

            # Build new odometry msg
            new_msg = Odometry()
            new_msg.header.stamp = msg.header.stamp
            new_msg.header.frame_id = 'map'
            new_msg.child_frame_id = msg.child_frame_id

            new_msg.pose.pose.position.x = pos_map[0]
            new_msg.pose.pose.position.y = pos_map[1]
            new_msg.pose.pose.position.z = pos_map[2]

            new_msg.pose.pose.orientation.x = q_map[0]
            new_msg.pose.pose.orientation.y = q_map[1]
            new_msg.pose.pose.orientation.z = q_map[2]
            new_msg.pose.pose.orientation.w = q_map[3]

            new_msg.twist = msg.twist

            self.publisher.publish(new_msg)

        except Exception as e:
            self.get_logger().warn(f"TF Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MapLocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

