#!/usr/bin/env python3
"""
WaypointPlanner (Dijkstra + safety zone + waypoint follower)
- Inflates obstacles around occupied cells
- Safe zone is treated as occupied during planning
- Publishes /global_path (Path)
- Subscribes to /rtabmap/odom and publishes /current_waypoint (PoseStamped)
"""
import os
import yaml
import cv2
import numpy as np
from math import sqrt, ceil
from heapq import heappush, heappop
from collections import deque
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped

class WaypointPlanner(Node):
    def __init__(self):
        super().__init__('waypoint_planner')
        # Map parameters
        self.map_yaml_path = os.path.expanduser('~/my_map.yaml')
        self.occupied_pixel_threshold = 50
        
        # we can customize this safety distance
        self.safety_distance_m = 0.8

        # Load map
        self.map = self.load_map(self.map_yaml_path)
        if self.map is None:
            self.get_logger().error("Failed to load map.")
            rclpy.shutdown()
            return

        # Inflate obstacles
        self.inflate_obstacles(self.safety_distance_m)

        # Publishers
        self.wp_pub = self.create_publisher(Path, '/global_path_dijkstra', 10)
        self.current_wp_pub = self.create_publisher(PoseStamped, '/current_waypoint', 10)

        # Subscriber
        #self.create_subscription(Odometry, '/rtabmap/odom', self.odom_callback, 10)
        self.create_subscription(Odometry, '/map_localization', self.odom_callback, 10)

        self.create_subscription(PoseStamped, '/dijkstra_goal', self.goal_callback, 10)

        # Internal state
        self.start_received = False
        self.goal_received = False
        self.current_pos = None
        self.goal_pos = None

        self.downsampled_path_world = []
        self.get_logger().info("Waypoint planner ready. Waiting for odom + goal...")

    # Path planning
    def plan_path_and_visualize(self):
        sx, sy = self.world_to_grid(self.current_pos)
        gx, gy = self.world_to_grid(self.goal_pos)

        grid = self.map['inflated_grid']
        if grid[sy, sx] == 1:
            sx, sy = self.find_nearest_free(sx, sy)
            self.current_pos = self.grid_to_world(sx, sy)
        if grid[gy, gx] == 1:
            gx, gy = self.find_nearest_free(gx, gy)
            self.goal_pos = self.grid_to_world(gx, gy)
        path = self.dijkstra((sx, sy), (gx, gy))

        if not path:
            self.get_logger().warn("No path found.")
            self.downsampled_path_world = []
            return

        # Publish full path
        self.publish_path(path)
        self.path_indices = path

        # Downsample for waypoint navigation
        desired_wp = 20
        downsampled_indices = self.downsample_path(path, desired_wp)
        self.downsampled_path_world = [self.grid_to_world(gx, gy) for gx, gy in downsampled_indices]
        self.get_logger().info(f"Path ready with {len(self.downsampled_path_world)} waypoints.")
        #self.visualize_map(sx, sy, gx, gy, self.path_indices)

    # Publish global path
    def publish_path(self, indices):
        msg = Path()
        msg.header.frame_id = 'map'
        for gx, gy in indices:
            x, y = self.grid_to_world(gx, gy)
            ps = PoseStamped()
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.wp_pub.publish(msg)
        self.get_logger().info(f"Published global path with {len(msg.poses)} poses.")

    # Odometry callback (waypoint follower)
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.current_pos = (x, y)

        if not self.start_received:
            self.start_received = True
            self.get_logger().info(f"Received start position from odom: {self.current_pos}")
        # If we have a path, follow it
        if self.downsampled_path_world:
            self.follow_waypoints((x, y))

    def goal_callback(self, msg):
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        self.goal_pos = (gx, gy)
        self.goal_received = True

        self.get_logger().info(f"Received new goal: {self.goal_pos}")                    
        if self.start_received:
            self.plan_path_and_visualize()


    def follow_waypoints(self, robot_pos):
        distances = [np.linalg.norm(np.array(robot_pos) - np.array(wp)) for wp in self.downsampled_path_world]
        idx = int(np.argmin(distances))

        # Choose next
        if idx + 1 < len(self.downsampled_path_world):
            next_wp = self.downsampled_path_world[idx + 1]
        else:
            next_wp = self.downsampled_path_world[idx]

        # Publish current target
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = next_wp[0]
        msg.pose.position.y = next_wp[1]
        msg.pose.orientation.w = 1.0
        self.current_wp_pub.publish(msg)

    # Map loading
    def load_map(self, yaml_file):
        if not os.path.exists(yaml_file):
            self.get_logger().error(f"YAML not found: {yaml_file}")
            return None
        with open(yaml_file, 'r') as f:
            data = yaml.safe_load(f)
        pgm_file = os.path.join(os.path.dirname(yaml_file), data['image'])
        if not os.path.exists(pgm_file):
            self.get_logger().error(f"PGM not found: {pgm_file}")
            return None
        img = cv2.imread(pgm_file, cv2.IMREAD_GRAYSCALE)
        if img is None:
            self.get_logger().error(f"Failed to load image: {pgm_file}")
            return None
        negate = int(data.get('negate', 0))
        if negate:
            img = 255 - img
        grid = (img <= self.occupied_pixel_threshold).astype(np.int8)
        resolution = float(data.get('resolution', 0.05))
        origin = data.get('origin', [0.0, 0.0, 0.0])[:2]
        self.get_logger().info(f"Loaded map: file={pgm_file}, size={img.shape[::-1]}, resolution={resolution}, origin={origin}")
        return {'img': img, 'grid': grid, 'resolution': resolution, 'origin': origin, 'width': grid.shape[1],'height': grid.shape[0],}

    # Inflate obstacles
    def inflate_obstacles(self, safety_distance_m):
        grid = self.map['grid'].astype(np.uint8)
        res = self.map['resolution']
        inflate_cells = ceil(safety_distance_m / res)
        kernel = np.ones((2*inflate_cells+1, 2*inflate_cells+1), dtype=np.uint8)
        inflated_grid = cv2.dilate(grid, kernel, iterations=1)
        self.map['inflated_grid'] = inflated_grid
        self.get_logger().info(f"Inflated obstacles by {safety_distance_m}m ({inflate_cells} cells)")

    # Dijkstra
    def dijkstra(self, start, goal):
        grid = self.map['inflated_grid']
        w, h = self.map['width'], self.map['height']
        start, goal = tuple(start), tuple(goal)
        if start == goal:
            return [start]
        open_set = []
        heappush(open_set, (0.0, start, [start]))
        visited = set()
        while open_set:
            g, current, path = heappop(open_set)
            if current in visited:
                continue
            visited.add(current)
            if current == goal:
                return path
            x, y = current
            for dx, dy in [(-1,-1),(-1,0),(1,0),(0,-1),(0,1),(1,1),(-1,1),(1,-1)]:
                nx, ny = x + dx, y + dy
                if self.in_bounds(nx, ny) and grid[ny, nx] == 0:
                    heappush(open_set, (g + sqrt(dx*dx + dy*dy), (nx, ny), path + [(nx, ny)]))
        return None

    # Coordinate transforms
    def world_to_grid(self, pos):
        x, y = pos
        ox, oy = self.map['origin']
        res = self.map['resolution']
        gx = int((x - ox) / res)
        gy = self.map['height'] - 1 - int((y - oy) / res)
        return gx, gy

    def grid_to_world(self, gx, gy):
        ox, oy = self.map['origin']
        res = self.map['resolution']
        x = gx * res + ox + res/2.0
        y = (self.map['height'] - 1 - gy) * res + oy + res/2.0
        return x, y

    def in_bounds(self, gx, gy):
        return 0 <= gx < self.map['width'] and 0 <= gy < self.map['height']

    # Nearest free cell
    def find_nearest_free(self, gx, gy):
        grid = self.map['inflated_grid']
        q = deque([(gx, gy)])
        visited = np.zeros_like(grid, dtype=np.uint8)
        visited[gy, gx] = 1
        while q:
            x, y = q.popleft()
            if grid[y, x] == 0:
                return x, y
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                nx, ny = x+dx, y+dy
                if self.in_bounds(nx, ny) and not visited[ny, nx]:
                    visited[ny, nx] = 1
                    q.append((nx, ny))
        return gx, gy

    # Downsample path
    def downsample_path(self, path, desired_number):
        if not path:
            return []
        if len(path) <= desired_number:
            return path
        indices = np.linspace(0, len(path)-1, desired_number, dtype=int)
        return [path[i] for i in indices]

    # Visualization
    def visualize_map(self, sx, sy, gx, gy, path):
        grid = self.map['grid']
        inflated = self.map['inflated_grid']
        vis = cv2.cvtColor((grid*255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
        h, w = grid.shape
        vis[np.logical_and(inflated==1, grid==0)] = (0, 165, 255)  # orange
        if path:
            for px, py in path:
                if 0 <= px < w and 0 <= py < h:
                    cv2.circle(vis, (px, py), 1, (255, 0, 0), -1)
        cv2.circle(vis, (sx, sy), 4, (0, 255, 0), -1)
        cv2.circle(vis, (gx, gy), 4, (0, 0, 255), -1)
        scale = 3
        vis_large = cv2.resize(vis, (vis.shape[1]*scale, vis.shape[0]*scale), interpolation=cv2.INTER_NEAREST)
        cv2.imshow("Planner with Safety Zones", vis_large)
        self.get_logger().info("Visualization: press any key in the window to continue.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = WaypointPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

