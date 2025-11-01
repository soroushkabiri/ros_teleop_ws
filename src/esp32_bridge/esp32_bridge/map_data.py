#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

class MapReader(Node):
    def __init__(self):
        super().__init__('map_reader')
        # Subscribe to map topic
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/rtabmap/map',
            self.map_callback,
            10)
        self.subscription  # prevent unused variable warning

    def map_callback(self, msg: OccupancyGrid):
        data = np.array(msg.data)
        print(f"Map frame_id: {msg.header.frame_id}")
        print(f"Map size: width={msg.info.width}, height={msg.info.height}")
        print(f"Map resolution: {msg.info.resolution} m/cell")
        print(f"Number of cells: {data.size}")
        print(f"Unknown cells (-1): {np.sum(data == -1)}")
        print(f"Free cells (0): {np.sum(data == 0)}")
        print(f"Occupied cells (100): {np.sum(data == 100)}")
        print(f"Min value: {np.min(data)}, Max value: {np.max(data)}")
        # Print first 50 cells just as an example
        print("First 50 cells:", data[:50])
        self.get_logger().info("Map data printed, shutting down node.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MapReader()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
