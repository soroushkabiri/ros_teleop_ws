#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import requests


ESP32_IP_F2 = "192.168.1.119"  # Default IP
ESP32_IP_F1 = "192.168.1.120"  # Default IP
ESP32_IP_L = "192.168.1.121"  # Default IP

WHEEL_BASE = 0.4  # meters

class ESP32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')
        
        # Default PID values
        self.default_kp = 400.0
        self.default_ki = 100.0
        self.default_kd = 50.0


        self.def_initial_pr_L = 0.0
        self.def_initial_pl_L = 0.0
        self.def_initial_pr_F1 = 0.0
        self.def_initial_pl_F1 = 0.0

        # Subscribe to /cmd_vel for leader
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback_L, 10)
        
        # Subscribe to /cmd_vel_des for follower 1
        self.create_subscription(Twist, '/robot0_1/cmd_vel_des', self.cmd_vel_callback_F1, 10)


        # Subscribe to /set_pwm_params
        self.create_subscription(Float32MultiArray, '/set_pwm', self.pwm_callback, 10)


        # Subscribe to /set_pid_params
        self.create_subscription(Float32MultiArray, '/set_pid_params', self.pid_callback, 10)

        # Publisher for actual velocities of leader
        self.actual_vel_pub_L = self.create_publisher(Twist, '/robot0_0/cmd_vel', 10)
        # Publisher for actual velocities of folower 1
        self.actual_vel_pub_F1 = self.create_publisher(Twist, '/robot0_1/cmd_vel', 10)
        # robot0_1=F1

        # Timer to periodically fetch status from ESP32 
        self.create_timer(1/40, self.status_callback)

    def cmd_vel_callback_L(self, msg):
        linear = max(min(msg.linear.x, 0.4), -0.4)
        angular = max(min(msg.angular.z, 0.8), -0.8)

        try:
            params = {
                'desired_linear_velocity_L': linear,
                'desired_angular_velocity_L': angular
            }
            response = requests.get(f'http://{ESP32_IP_L}/v_desired_L', params=params, timeout=1)
            #self.get_logger().info(f"Sent to ESP32: v={linear:.2f}, w={angular:.2f}, Response: {response.text}")
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Failed to send velocity to ESP32: {e}")

    def cmd_vel_callback_F1(self, msg):
        linear = max(min(msg.linear.x, 0.2), -0.2)
        angular = max(min(msg.angular.z, 0.4), -0.4)
        self.send_velocity(linear, angular)

    def send_velocity(self, linear, angular):
        try:
            params = {
                'desired_linear_velocity_F1': linear,
                'desired_angular_velocity_F1': angular
            }
            response = requests.get(f'http://{ESP32_IP_F1}/v_desired_F1', params=params, timeout=1)
            self.get_logger().info(f"Sent to ESP32: v={linear:.2f}, w={angular:.2f}, Response: {response.text}")
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Failed to send velocity to follower 1 ESP32: {e}")


    def destroy_node(self):
        # Send zero velocity before shutting down
        self.get_logger().info("Node shutting down, stopping Follower 1...")
        self.send_velocity(0.0, 0.0)
        super().destroy_node()

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
            response = requests.get(f'http://{ESP32_IP_L}/set_pid', params=params, timeout=1)
            #self.get_logger().info(f"Sent PID to ESP32: kp={kp}, ki={ki}, kd={kd}, Response: {response.text}")
        except Exception as e:
            self.get_logger().error(f"Failed to send PID params to ESP32: {e}")




    def pwm_callback(self, msg):
        
        data = list(msg.data)
        initial_pr_L = self.def_initial_pr_L
        initial_pl_L = self.def_initial_pl_L
        #initial_pr_F2 = self.def_initial_pr_F2
        #initial_pl_F2 = self.def_initial_pl_F2        
        initial_pr_F1 = self.def_initial_pr_F1
        initial_pl_F1 = self.def_initial_pl_F1     
        try:
            if len(data) >= 1:
                initial_pr_L = data[0]
                initial_pr_F1 = data[0]

            if len(data) >= 2:
                initial_pl_L = data[1]
                initial_pl_F1 = data[1]

            params_l = {'initial_pr_L': initial_pr_L,'initial_pl_L': initial_pl_L,}
            params_F1 = {'initial_pr_F1': initial_pr_F1,'initial_pl_F1': initial_pl_F1,}

            response_l = requests.get(f'http://{ESP32_IP_L}/set_pwm_L', params=params_l, timeout=1)
            response_F1 = requests.get(f'http://{ESP32_IP_F1}/set_pwm_F1', params=params_F1, timeout=1)

            #self.get_logger().info(f"Sent Pwm to ESP32: initial_pr_F2={initial_pr_F2}, initial_pl_F2={initial_pl_F2}, Response: {response.text}")
        except Exception as e:
            self.get_logger().error(f"Failed to send pwm params to ESP32: {e}")




    def status_callback(self):
        # This is "periodic callback" to GET /status and handle data
        try:
            response_F1 = requests.get(f"http://{ESP32_IP_F1}/status_F1", timeout=1)
            response_F1.raise_for_status()
            data_F1 = response_F1.json()
            

            response_L = requests.get(f"http://{ESP32_IP_L}/status_L", timeout=1)
            response_L.raise_for_status()
            data_L = response_L.json()


            speed_m1_F1 = float(data_F1.get("actual_speed_m1_F", 0.0))
            speed_m2_F1  = float(data_F1 .get("actual_speed_m2_F", 0.0))

            speed_m1_L = float(data_L.get("actual_speed_m1_L", 0.0))
            speed_m2_L  = float(data_L .get("actual_speed_m2_L", 0.0))

            v_actual_F1  = (speed_m1_F1  + speed_m2_F1 ) / 2.0
            w_actual_F1  = (speed_m1_F1  - speed_m2_F1 ) / WHEEL_BASE

            v_actual_L  = (speed_m1_L  + speed_m2_L ) / 2.0
            w_actual_L  = (speed_m1_L  - speed_m2_L ) / WHEEL_BASE

            speed_m1_d_F1  = float(data_F1 .get("desired_speed_m1_F1", 0.0))
            speed_m2_d_F1  = float(data_F1 .get("desired_speed_m2_F1", 0.0))

            speed_m1_d_L  = float(data_L .get("desired_speed_m1_L", 0.0))
            speed_m2_d_L  = float(data_L .get("desired_speed_m2_L", 0.0))

            v_d_F1  = (speed_m1_d_F1  + speed_m2_d_F1 ) / 2.0
            w_d_F1  = (speed_m1_d_F1  - speed_m2_d_F1 ) / WHEEL_BASE

            v_d_L  = (speed_m1_d_L  + speed_m2_d_L ) / 2.0
            w_d_L  = (speed_m1_d_L  - speed_m2_d_L ) / WHEEL_BASE

            #self.get_logger().info(f"Actual speeds -> M1: {speed_m1:.3f}, M2: {speed_m2:.3f}")
            #self.get_logger().info(f"actual linear velocities -> v_actual_Leader: {v_actual_L:.3f} m/s, v_actual_follower: {v_actual_F1:.3f} rad/s")
            #self.get_logger().info(f"desired linear velocities -> v_desired_Leader: {v_d_L:.3f} m/s, v_desired_follower: {v_d_F1:.3f} rad/s")

            #self.get_logger().info(f"actual angular velocities -> w_actual_leader: {w_actual_L:.3f} m/s, w_actual_follower: {w_actual_F1:.3f} rad/s")
            #self.get_logger().info(f"desired angular velocities -> w_desired_leader: {w_d_L:.3f} m/s, w_desired_follower: {w_d_F1:.3f} rad/s")

            twist_msg_F1 = Twist()
            twist_msg_F1.linear.x = v_actual_F1
            twist_msg_F1.angular.z = w_actual_F1
            self.actual_vel_pub_F1.publish(twist_msg_F1)

            twist_msg_L = Twist()
            twist_msg_L.linear.x = v_actual_L
            twist_msg_L.angular.z = w_actual_L
            self.actual_vel_pub_L.publish(twist_msg_L)

        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Failed to get status from ESP32: {e}")
        except ValueError as e:
            self.get_logger().error(f"Failed to parse status JSON: {e}")



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
#ros2 topic pub /set_pid_params std_msgs/Float32MultiArray "{data: [500, 0.01, 0.1]}"

