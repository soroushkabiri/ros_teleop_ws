#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import requests

ESP32_IP = "192.168.4.1"  # Default IP

class ESP32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        
        # Default PID values
        self.default_kp = 500
        self.default_ki = 0
        self.default_kd = 0.1

        # Subscribe to /cmd_vel
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Subscribe to /set_pid_params
        self.create_subscription(Float32MultiArray, '/set_pid_params', self.pid_callback, 10)

    def cmd_vel_callback(self, msg):
        linear = max(min(msg.linear.x, 0.2), -0.2)
        angular = max(min(msg.angular.z, 0.5), -0.5)

        try:
            params = {
                'desired linear velocity': linear,
                'desired angular velocity': angular
            }
            response = requests.get(f'http://{ESP32_IP}/v_desired', params=params, timeout=1)
            self.get_logger().info(f"Sent to ESP32: v={linear:.2f}, w={angular:.2f}, Response: {response.text}")
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Failed to send velocity to ESP32: {e}")

    def pid_callback(self, msg):
        
        data = list(msg.data)
        kp = self.default_kp
        ki = self.default_ki
        kd = self.default_kd

        try:
            if len(data) >= 1:
                kp = data[0]
            if len(data) >= 2:
                ki = data[1]
            if len(data) >= 3:
                kd = data[2]
            #kp, ki, kd = msg.data
            params = {
                'kp': kp,
                'ki': ki,
                'kd': kd
            }
            response = requests.get(f'http://{ESP32_IP}/set_pid', params=params, timeout=1)
            self.get_logger().info(f"Sent PID to ESP32: kp={kp}, ki={ki}, kd={kd}, Response: {response.text}")
        except Exception as e:
            self.get_logger().error(f"Failed to send PID params to ESP32: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ESP32Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


# the way to set pid parameters:
#ros2 topic pub /set_pid_params std_msgs/Float32MultiArray "{data: [500, 0.01, 0.1]}"

