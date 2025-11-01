# #!/usr/bin/env python3
# """
# WaypointPlanner (Dijkstra + safety zone)
# - Inflates obstacles around occupied cells
# - Safe zone is treated as occupied during planning
# - Visualizes inflated areas in orange
# - Publishes /global_path (avoiding safety zones)
# """
# import os
# import yaml
# import cv2
# import numpy as np
# from math import sqrt, ceil
# from heapq import heappush, heappop
# from collections import deque

# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Path
# from geometry_msgs.msg import PoseStamped


# class WaypointPlanner(Node):
#     def __init__(self):
#         super().__init__('waypoint_planner')

#         self.map_yaml_path = os.path.expanduser('~/my_map.yaml')
#         self.occupied_pixel_threshold = 50
#         self.safety_distance_m = 0.8  # inflate 0.85 meter around obstacles

#         self.map = self.load_map(self.map_yaml_path)
#         if self.map is None:
#             self.get_logger().error("Failed to load map.")
#             rclpy.shutdown()
#             return

#         # Create safety-inflated map
#         self.inflate_obstacles(self.safety_distance_m)

#         self.wp_pub = self.create_publisher(Path, '/global_path', 10)

#         self.current_pos = (0.0, 0.0)
#         self.goal_pos = (5.0, -1.5)
#         self.get_logger().info(f"Testing: start={self.current_pos}, goal={self.goal_pos}")

#         self.plan_path_and_visualize()
#         self.downsampled_path = self.downsample_path(self.path_indices, 10)


#     # -------------------------
#     # Load map
#     # -------------------------
#     def load_map(self, yaml_file):
#         if not os.path.exists(yaml_file):
#             self.get_logger().error(f"YAML not found: {yaml_file}")
#             return None

#         with open(yaml_file, 'r') as f:
#             data = yaml.safe_load(f)

#         pgm_file = os.path.join(os.path.dirname(yaml_file), data['image'])
#         if not os.path.exists(pgm_file):
#             self.get_logger().error(f"PGM not found: {pgm_file}")
#             return None

#         img = cv2.imread(pgm_file, cv2.IMREAD_GRAYSCALE)
#         if img is None:
#             self.get_logger().error(f"Failed to load image: {pgm_file}")
#             return None

#         negate = int(data.get('negate', 0))
#         if negate:
#             img = 255 - img

#         grid = (img <= self.occupied_pixel_threshold).astype(np.int8)

#         resolution = float(data.get('resolution', 0.05))
#         origin = data.get('origin', [0.0, 0.0, 0.0])[:2]

#         self.get_logger().info(f"Loaded map: file={pgm_file}, size={img.shape[::-1]}, resolution={resolution}, origin={origin}")

#         return {
#             'img': img,
#             'grid': grid,
#             'resolution': resolution,
#             'origin': origin,
#             'width': grid.shape[1],
#             'height': grid.shape[0],
#         }

#     # -------------------------
#     # Inflate obstacles
#     # # -------------------------
#     def inflate_obstacles(self, safety_distance_m):
#         grid = self.map['grid'].astype(np.uint8)  # <-- convert to uint8
#         res = self.map['resolution']
#         inflate_cells = ceil(safety_distance_m / res)

#         kernel = np.ones((2*inflate_cells+1, 2*inflate_cells+1), dtype=np.uint8)
#         inflated_grid = cv2.dilate(grid, kernel, iterations=1)
#         self.map['inflated_grid'] = inflated_grid
#         self.get_logger().info(f"Inflated obstacles by {safety_distance_m}m ({inflate_cells} cells)")

#     # -------------------------
#     # Path planning
#     # -------------------------
#     def plan_path_and_visualize(self):
#         sx, sy = self.world_to_grid(self.current_pos)
#         gx, gy = self.world_to_grid(self.goal_pos)
#         self.get_logger().info(f"Start grid=({sx},{sy}), Goal grid=({gx},{gy})")

#         if not self.in_bounds(sx, sy):
#             self.get_logger().error(f"Start outside map bounds: ({sx},{sy})")
#             return
#         if not self.in_bounds(gx, gy):
#             self.get_logger().error(f"Goal outside map bounds: ({gx},{gy})")
#             return

#         grid = self.map['inflated_grid']
#         start_occ = int(grid[sy, sx])
#         goal_occ = int(grid[gy, gx])
#         self.get_logger().info(f"Start occupied={start_occ}, Goal occupied={goal_occ}")

#         if start_occ == 1:
#             self.get_logger().warn("Start occupied -> snapping to nearest free")
#             sx, sy = self.find_nearest_free(sx, sy)
#             self.current_pos = self.grid_to_world(sx, sy)
#             self.get_logger().info(f"New start grid: ({sx},{sy}) world={self.current_pos}")

#         if goal_occ == 1:
#             self.get_logger().warn("Goal occupied -> snapping to nearest free")
#             gx, gy = self.find_nearest_free(gx, gy)
#             self.goal_pos = self.grid_to_world(gx, gy)
#             self.get_logger().info(f"New goal grid: ({gx},{gy}) world={self.goal_pos}")

#         path = self.dijkstra((sx, sy), (gx, gy))
#         if not path:
#             self.get_logger().warn("No path found.")
#             self.path_indices = []
#         else:
#             self.path_indices = path
#             self.publish_path(path)

#         self.visualize_map(sx, sy, gx, gy, self.path_indices)

#     # -------------------------
#     # Coordinate transforms
#     # -------------------------
#     def world_to_grid(self, pos):
#         x, y = float(pos[0]), float(pos[1])
#         ox, oy = self.map['origin']
#         res = self.map['resolution']
#         gx = int((x - ox) / res)
#         gy = self.map['height'] - 1 - int((y - oy) / res)
#         return gx, gy

#     def grid_to_world(self, gx, gy):
#         ox, oy = self.map['origin']
#         res = self.map['resolution']
#         x = gx * res + ox + res / 2.0
#         y = (self.map['height'] - 1 - gy) * res + oy + res / 2.0
#         return x, y

#     def in_bounds(self, gx, gy):
#         return 0 <= gx < self.map['width'] and 0 <= gy < self.map['height']

#     # -------------------------
#     # Nearest free cell
#     # -------------------------
#     def find_nearest_free(self, gx, gy):
#         grid = self.map['inflated_grid']
#         h, w = grid.shape
#         q = deque([(gx, gy)])
#         visited = np.zeros_like(grid, dtype=np.uint8)
#         if not self.in_bounds(gx, gy):
#             return gx, gy
#         visited[gy, gx] = 1
#         while q:
#             x, y = q.popleft()
#             if grid[y, x] == 0:
#                 return x, y
#             for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
#                 nx, ny = x + dx, y + dy
#                 if self.in_bounds(nx, ny) and not visited[ny, nx]:
#                     visited[ny, nx] = 1
#                     q.append((nx, ny))
#         return gx, gy

#     # -------------------------
#     # Dijkstra
#     # -------------------------
#     def dijkstra(self, start, goal):
#         grid = self.map['inflated_grid']
#         w, h = self.map['width'], self.map['height']
#         start, goal = tuple(start), tuple(goal)
#         if start == goal:
#             return [start]

#         open_set = []
#         heappush(open_set, (0.0, start, [start]))
#         visited = set()

#         while open_set:
#             g, current, path = heappop(open_set)
#             if current in visited:
#                 continue
#             visited.add(current)
#             if current == goal:
#                 return path
#             x, y = current
#             for dx, dy in [(-1,-1),(-1,0),(1,0),(0,-1),(0,1),(1,1),(-1,1),(1,-1)]:
#                 nx, ny = x + dx, y + dy
#                 if self.in_bounds(nx, ny) and grid[ny, nx] == 0:
#                     step_cost = sqrt(dx*dx + dy*dy)
#                     heappush(open_set, (g + step_cost, (nx, ny), path + [(nx, ny)]))
#         return None

#     # -------------------------
#     # Publish path
#     # -------------------------
#     def publish_path(self, indices):
#         msg = Path()
#         msg.header.frame_id = 'map'
#         for gx, gy in indices:
#             x, y = self.grid_to_world(gx, gy)
#             ps = PoseStamped()
#             ps.pose.position.x = x
#             ps.pose.position.y = y
#             ps.pose.orientation.w = 1.0
#             msg.poses.append(ps)
#         self.wp_pub.publish(msg)
#         self.get_logger().info(f"Published path with {len(msg.poses)} poses")

#     # -------------------------
#     # Visualization
#     # -------------------------
#     def visualize_map(self, sx, sy, gx, gy, path):
#         grid = self.map['grid']
#         inflated = self.map['inflated_grid']
#         vis = cv2.cvtColor((grid*255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
#         h, w = grid.shape

#         # show inflated zones in orange
#         vis[np.logical_and(inflated==1, grid==0)] = (0, 165, 255)  # BGR: orange

#         # draw path
#         if path:
#             for px, py in path:
#                 if 0 <= px < w and 0 <= py < h:
#                     cv2.circle(vis, (px, py), 1, (255, 0, 0), -1)
#         # draw start/goal
#         if 0 <= sx < w and 0 <= sy < h:
#             cv2.circle(vis, (sx, sy), 4, (0, 255, 0), -1)
#         if 0 <= gx < w and 0 <= gy < h:
#             cv2.circle(vis, (gx, gy), 4, (0, 0, 255), -1)

#         cv2.putText(vis, f"Start grid: ({sx},{sy})", (5,15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200,200,200), 1)
#         cv2.putText(vis, f"Goal grid: ({gx},{gy})", (5,30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200,200,200), 1)
#         cv2.putText(vis, "Inflated zones (orange)", (5,45), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,165,255),1)

#         scale = 3
#         vis_large = cv2.resize(vis, (vis.shape[1]*scale, vis.shape[0]*scale), interpolation=cv2.INTER_NEAREST)
#         cv2.imshow("Planner with Safety Zones", vis_large)
#         self.get_logger().info("Visualization: press any key in the window to continue.")
#         cv2.waitKey(0)
#         cv2.destroyAllWindows()

#     def downsample_path(self, path, desired_number):
#         """
#         path: list of (gx, gy)
#         desired_number: number of waypoints to downsample to
#         """
#         if not path:
#             return []

#         if len(path) <= desired_number:
#             return path

#         indices = np.linspace(0, len(path)-1, desired_number, dtype=int)
#         downsampled = [path[i] for i in indices]
#         return downsampled


# def main(args=None):
#     rclpy.init(args=args)
#     node = WaypointPlanner()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()





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
        self.wp_pub = self.create_publisher(Path, '/global_path', 10)
        self.current_wp_pub = self.create_publisher(PoseStamped, '/current_waypoint', 10)

        # Subscriber
        self.create_subscription(Odometry, '/rtabmap/odom', self.odom_callback, 10)

        # Test start/goal
        self.current_pos = (0.0, 0.0)
        self.goal_pos = (5.0, -1.5)

        # Plan path
        self.plan_path_and_visualize()

        # Downsample path
        self.desired_number_of_waypoints = 10
        self.downsampled_path = self.downsample_path(self.path_indices, self.desired_number_of_waypoints)

        # Convert downsampled path to world coordinates
        self.downsampled_path_world = [self.grid_to_world(gx, gy) for gx, gy in self.downsampled_path]

        self.get_logger().info(f"Waypoint planner initialized with {len(self.downsampled_path_world)} waypoints.")

    # -------------------------
    # Map loading
    # -------------------------
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

        return {
            'img': img,
            'grid': grid,
            'resolution': resolution,
            'origin': origin,
            'width': grid.shape[1],
            'height': grid.shape[0],
        }

    # -------------------------
    # Inflate obstacles
    # -------------------------
    def inflate_obstacles(self, safety_distance_m):
        grid = self.map['grid'].astype(np.uint8)
        res = self.map['resolution']
        inflate_cells = ceil(safety_distance_m / res)

        kernel = np.ones((2*inflate_cells+1, 2*inflate_cells+1), dtype=np.uint8)
        inflated_grid = cv2.dilate(grid, kernel, iterations=1)
        self.map['inflated_grid'] = inflated_grid
        self.get_logger().info(f"Inflated obstacles by {safety_distance_m}m ({inflate_cells} cells)")

    # -------------------------
    # Path planning
    # -------------------------
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
        if path:
            self.path_indices = path
            self.publish_path(path)
        else:
            self.get_logger().warn("No path found.")
            self.path_indices = []

        self.visualize_map(sx, sy, gx, gy, self.path_indices)

    # -------------------------
    # Dijkstra
    # -------------------------
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

    # -------------------------
    # Coordinate transforms
    # -------------------------
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

    # -------------------------
    # Nearest free cell
    # -------------------------
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

    # -------------------------
    # Downsample path
    # -------------------------
    def downsample_path(self, path, desired_number):
        if not path:
            return []
        if len(path) <= desired_number:
            return path
        indices = np.linspace(0, len(path)-1, desired_number, dtype=int)
        return [path[i] for i in indices]

    # -------------------------
    # Publish global path
    # -------------------------
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

    # -------------------------
    # Odometry callback (waypoint follower)
    # -------------------------
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        robot_pos = np.array([x, y])

        # Find closest waypoint
        distances = [np.linalg.norm(robot_pos - np.array([wp[0], wp[1]])) for wp in self.downsampled_path_world]
        closest_idx = int(np.argmin(distances))

        # Set next waypoint
        if closest_idx + 1 < len(self.downsampled_path_world):
            next_wp = self.downsampled_path_world[closest_idx + 1]
        else:
            next_wp = self.downsampled_path_world[closest_idx]

        # Publish next waypoint
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = next_wp[0]
        msg.pose.position.y = next_wp[1]
        msg.pose.orientation.w = 1.0
        self.current_wp_pub.publish(msg)

    # -------------------------
    # Visualization
    # -------------------------
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


