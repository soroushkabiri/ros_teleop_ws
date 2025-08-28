#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import requests

ESP32_IP_F2 = "192.168.1.119"  # Default IP
ESP32_IP_F1 = "192.168.1.120"  # Default IP
ESP32_IP_L = "192.168.1.121"  # Default IP

class ESP32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        
        # Default PID values
        self.default_pr = 0
        self.default_pl = 0

        # Subscribe to /set_pwm
        self.create_subscription(Float32MultiArray, '/set_pwm', self.pwm_callback, 10)

    def pwm_callback(self, msg):
        
        data = list(msg.data)
        pr = self.default_pr
        pl = self.default_pl

        try:
            if len(data) >= 1:
                pr = data[0]
            if len(data) >= 2:
                pl = data[1]

            #kp, ki, kd = msg.data
            params = {'pr': pr,'pl': pl}
            response = requests.get(f'http://{ESP32_IP_L}/set_pwm', params=params, timeout=1)
            self.get_logger().info(f"Sent PID to ESP32: pr={pr}, pl={pl}, Response: {response.text}")
        except Exception as e:
            self.get_logger().error(f"Failed to send PID params to ESP32: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ESP32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


# the way to set pid parameters:
#ros2 topic pub /set_pwm std_msgs/Float32MultiArray "{data: [0, 0]}"

