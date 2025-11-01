#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy

class RectOdomYaw(Node):
    def __init__(self):
        super().__init__('rect_obj_odom_yaw_node')

        # QoS to match rtabmap best-effort publisher

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        # Subscriber to /rtabmap/odom
        self.create_subscription(Odometry, '/rtabmap/odom', self.odom_callback, qos)

        # Publisher for yaw in degrees
        self.yaw_pub = self.create_publisher(Float32, '/rect_obj_odom_yaw', 10)

        self.get_logger().info('RectOdomYaw Node started, publishing /rect_obj_odom_yaw.')

    
    def odom_callback(self, msg: Odometry):
        self.get_logger().info("Received odom message")
        q = msg.pose.pose.orientation
        yaw_rad = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        yaw_deg = math.degrees(yaw_rad)
        self.yaw_pub.publish(Float32(data=yaw_deg))
    
    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        # Convert quaternion to yaw angle (Z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = RectOdomYaw()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
