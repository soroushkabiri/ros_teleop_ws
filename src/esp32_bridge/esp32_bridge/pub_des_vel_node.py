#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import math
from std_msgs.msg import Float32, Float32MultiArray
import numpy as np
import matplotlib.pyplot as plt
import time
from sensor_msgs.msg import Joy

class XHatToCmdNode(Node):
    def __init__(self):
        super().__init__('x_hat_to_cmd_node')


        # --- Declare parameters with default values for angular velocity controller ---
        self.declare_parameter('kp_angular', 1.0)
        self.declare_parameter('ki_angular', 0.02)
        self.declare_parameter('max_integral_angular', 10.0)

        # List of robot names
        self.robot_names = ['robot0_1',]
        self.num_followers = len(self.robot_names)
        self.leader_current_orientation = 0.0

        # Dictionary to store latest v_hat values and yaw_hat values
        self.v_hat_values = {name: 0.0 for name in self.robot_names}
        self.yaw_hat_values = {name: 0.0 for name in self.robot_names}
        self.current_orientation = {name: 0.0 for name in self.robot_names}

        # Create subscriptions and publishers
        self.cmd_publishers = {}
        for name in self.robot_names:
            self.create_subscription(Float32, f'/{name}/v_hat', self.make_v_hat_callback(name), 10)
            self.create_subscription(Float32, f'/{name}/yaw_hat', self.make_yaw_hat_callback(name), 10)
            self.create_subscription(Float32, f'/{name}/yaw_deg', self.make_current_orientation_callback(name), 10)
            self.create_subscription(Float32, f'/robot0_0/yaw_deg', self.make_leader_current_orientation_callback(), 10)
            self.cmd_publishers[name] = self.create_publisher(Twist, f'/{name}/cmd_vel_des', 10)

        # Plotting setup
#        self.time_history = []
#        self.current_yaw_history_leader = []
#        self.current_yaw_history_followers = [[] for _ in range(self.num_followers)]
        self.start_time = time.time()

#        plt.ion()
#        self.fig, self.ax = plt.subplots(figsize=(10,6))
        
        self.integral_error_F1 = 0.0
        self.last_time = time.time()

        # Timer to publish at fixed rate
        self.create_timer(1/50, self.timer_callback)  # 50 Hz

        self.flip_orientation = False
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)


    def joy_callback(self, msg):
        # use button triangle to rotate while still
        if msg.buttons[2] == 1:
            self.flip_orientation = True
        else:
            self.flip_orientation = False


    def make_v_hat_callback(self, robot_name):
        def callback(msg):
            self.v_hat_values[robot_name] = msg.data
        return callback
    
    def make_yaw_hat_callback(self, robot_name):
        def callback(msg):
            self.yaw_hat_values[robot_name] = msg.data
        return callback
    
    def make_current_orientation_callback(self, robot_name):
        def callback(msg):
            self.current_orientation[robot_name] = msg.data # its on degree (-180 to 180) degree
        return callback
    
    def make_leader_current_orientation_callback(self):
        def callback(msg):
            self.leader_current_orientation = msg.data # its on degree (-180 to 180) degree
        return callback

    def timer_callback(self):
        current_time = time.time() - self.start_time

        for name in self.robot_names:
            cmd_msg = Twist()
            cmd_msg.linear.x = self.v_hat_values[name]
            cmd_msg.angular.z = self.calculate_angular_z(self.yaw_hat_values[name], self.current_orientation[name])
            # All other fields remain zero
            self.cmd_publishers[name].publish(cmd_msg)
        
        # Update plots
#        self.time_history.append(current_time)
#        self.current_yaw_history_leader.append(self.leader_current_orientation)
#        for i,name in enumerate(self.robot_names):
#            self.current_yaw_history_followers[i].append(self.current_orientation[name])

#        self.update_plot()

    def calculate_angular_z(self, desired_orientation, current_orientation):

        current_orientation=(current_orientation/180)*math.pi
        
        # If joystick button pressed, flip target orientation by 180°
        if self.flip_orientation:
            desired_orientation = (desired_orientation + math.pi) % (2 * math.pi)



        # --- Get parameters dynamically ---
        kp = self.get_parameter('kp_angular').get_parameter_value().double_value
        ki = self.get_parameter('ki_angular').get_parameter_value().double_value
        max_integral = self.get_parameter('max_integral_angular').get_parameter_value().double_value



        #kp=1.0 # proportional gain
        #ki = .02  # Integral gain
        
        diff = ((current_orientation - desired_orientation + math.pi) % (2*math.pi)) - math.pi

        # --- Integral term ---
        now = time.time()
        dt = now - self.last_time
        self.last_time = now


        self.integral_error_F1 += diff * dt

        # Optional: prevent integral windup
        #max_integral = 10.0
        self.integral_error_F1 = max(-max_integral, min(max_integral, self.integral_error_F1))

        self.get_logger().info(f"p_error : {kp*diff:.3f} , i-error: {ki*self.integral_error_F1:.3f} ")

        angular_z = kp * diff + ki * self.integral_error_F1
        # Optional: Limit maximum speed to avoid excessive rotation
        max_speed = 2.0
        angular_z = max(-max_speed, min(max_speed, angular_z))

        return -angular_z  # negative because positive diff means need to rotate negative direction

#    def update_plot(self):
#        self.ax.clear()
#        self.ax.plot(self.time_history, self.current_yaw_history_leader, 'k--', label='Leader orientation')
#        for i in range(self.num_followers):
#            self.ax.plot(self.time_history, self.current_yaw_history_followers[i], label=f'Follower {i} orientation')
#        self.ax.set_title('Leader vs Followers orientation')
#        self.ax.set_xlabel('Time (s)')
#        self.ax.set_ylabel('orientation (degree)')
#        self.ax.legend()
#        self.ax.grid(True)
#        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    node = XHatToCmdNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
