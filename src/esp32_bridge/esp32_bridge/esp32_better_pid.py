#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import requests

#ESP32_IP_F2 = "192.168.1.119"  # Default IP
#ESP32_IP_F1 = "192.168.1.120"  # Default IP
#ESP32_IP_L = "192.168.1.121"  # Default IP
ESP32_IP_L = "172.20.10.2"  # Default IP
ESP32_IP_F1 = "172.20.10.3"  # Default IP
ESP32_IP_F2 = "172.20.10.4"  # Default IP

WHEEL_BASE = 0.4  # meters

class ESP32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        
        # Default PID values
        self.default_kp = 0.0
        self.default_ki = 0.0
        self.default_kd = 0.0

        #self.def_initial_pr_L = 0.0
        #self.def_initial_pl_L = 0.0
        self.def_initial_pr_F2 = 0.0
        self.def_initial_pl_F2 = 0.0
        
        # Subscribe to /cmd_vel for leader

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback_F2, 10)
        #self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback_F2, 10)


        # Subscribe to /set_pid_params
        self.create_subscription(Float32MultiArray, '/set_pid_params', self.pid_callback, 10)
        
        # Subscribe to /set_pid_params
        self.create_subscription(Float32MultiArray, '/set_pwm', self.pwm_callback, 10)

    def cmd_vel_callback_F2(self, msg):
        linear = max(min(msg.linear.x, 0.6), -0.6)
        angular = max(min(msg.angular.z, 1.2), -1.2)

        try:
            params = {
                #'desired_linear_velocity_L': linear,
                #'desired_angular_velocity_L': angular
                #'desired_linear_velocity_F2': linear,
                #'desired_angular_velocity_F2': angular
                'desired_linear_velocity_F2': linear,
                'desired_angular_velocity_F2': angular


            }
            #response = requests.get(f'http://{ESP32_IP_F1}/v_desired_F1', params=params, timeout=1)
            response = requests.get(f'http://{ESP32_IP_F2}/v_desired_F2', params=params, timeout=1)

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
            params = {'kp': kp,'ki': ki,'kd': kd}
            #response = requests.get(f'http://{ESP32_IP_F1}/set_pid', params=params, timeout=1)
            response = requests.get(f'http://{ESP32_IP_F2}/set_pid', params=params, timeout=1)

            #self.get_logger().info(f"Sent PID to ESP32: kp={kp}, ki={ki}, kd={kd}, Response: {response.text}")
        except Exception as e:
            self.get_logger().error(f"Failed to send PID params to ESP32: {e}")

    def pwm_callback(self, msg):
        
        data = list(msg.data)
        #initial_pr_L = self.def_initial_pr_L
        #initial_pl_L = self.def_initial_pl_L
        initial_pr_F2 = self.def_initial_pr_F2
        initial_pl_F2 = self.def_initial_pl_F2        
        #initial_pr_F1 = self.def_initial_pr_F1
        #initial_pl_F1 = self.def_initial_pl_F1     
        try:
            if len(data) >= 1:
                #initial_pr_L = data[0]
                initial_pr_F2 = data[0]

            if len(data) >= 2:
                #initial_pl_L = data[1]
                initial_pl_F2 = data[1]

            #params = {'initial_pr_L': initial_pr_L,'initial_pl_L': initial_pl_L,}
            params = {'initial_pr_F2': initial_pr_F2,'initial_pl_F1': initial_pl_F2,}

            #response = requests.get(f'http://{ESP32_IP_L}/set_pwm_L', params=params, timeout=1)
            response = requests.get(f'http://{ESP32_IP_F2}/set_pwm_F2', params=params, timeout=1)

            #self.get_logger().info(f"Sent Pwm to ESP32: initial_pr_F2={initial_pr_F2}, initial_pl_F2={initial_pl_F2}, Response: {response.text}")
        except Exception as e:
            self.get_logger().error(f"Failed to send pwm params to ESP32: {e}")

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
# for leader and F2
#ros2 topic pub /set_pid_params std_msgs/Float32MultiArray "{data: [40, 0.01, 40]}"
#ros2 topic pub /set_pwm std_msgs/Float32MultiArray "{data: [64, 64]}"
# for F1
#ros2 topic pub /set_pid_params std_msgs/Float32MultiArray "{data: [50, 0.01, 60]}"
#ros2 topic pub /set_pwm std_msgs/Float32MultiArray "{data: [72, 72]}"