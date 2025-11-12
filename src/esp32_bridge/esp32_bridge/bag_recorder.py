#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess
import datetime
import os

class BagRecorderNode(Node):
    def __init__(self):
        super().__init__('bag_recorder')

        # Define topics to record
        self.topics = [
            #"/cmd_vel_fuzzy",
            "/cmd_vel_joy",
            "/robot0_1/v_hat", "/robot1_0/v_hat",
            "/robot0_1/yaw_hat", "/robot1_0/yaw_hat",
             "/robot0_0/yaw_deg",]

        # Create a custom folder name with timestamp
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.bag_name = "map_real"

        # Command for ros2 bag record
        cmd = ["ros2", "bag", "record", "-o", self.bag_name] + self.topics
        self.get_logger().info(f"Starting ros2 bag record → {self.bag_name}")

        # Launch as subprocess
        self.process = subprocess.Popen(cmd)

    def destroy_node(self):
        # When shutting down, stop the recording process
        self.get_logger().info("Stopping ros2 bag record...")
        self.process.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BagRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()



