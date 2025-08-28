#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

def quaternion_to_yaw_deg(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(yaw)

class ImuYawPublisher(Node):
    def __init__(self):
        super().__init__('imu_yaw_publisher')
        self.declare_parameter('imu_topic', 'imu')
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.sub = self.create_subscription(Imu, imu_topic+'/imu/yaw', self.imu_callback, 10)
        self.pub = self.create_publisher(Float32, imu_topic+ 'imu/yaw', 10)
        #self.get_logger().info(f"Subscribed to IMU topic: {imu_topic+'/imu'}, Publishing {imu_topic} '/yaw_deg' in same namespace.")

    def imu_callback(self, msg):
        q = msg.orientation
        yaw_deg = quaternion_to_yaw_deg(q.x, q.y, q.z, q.w)
        yaw_msg = Float32()
        yaw_msg.data = yaw_deg
        self.pub.publish(yaw_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuYawPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
