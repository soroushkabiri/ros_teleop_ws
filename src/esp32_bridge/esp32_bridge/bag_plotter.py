#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sqlite3
import pandas as pd
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import math

# --- IEEE plotting style ---
plt.rcParams.update({
    "font.family": "serif",      # Times/Serif font
    "font.size": 15,              # base font
    "axes.titlesize": 18,
    "axes.labelsize": 18,
    "legend.fontsize": 14,
    "xtick.labelsize": 16,
    "ytick.labelsize": 16
})


class BagPlotter(Node):
    def __init__(self):
        super().__init__('bag_plotter')

        # --- Bag file ---
        bag_path = "map_real/map_real_0.db3"
        self.get_logger().info(f"Loading bag: {bag_path}")

        # Connect to SQLite bag
        con = sqlite3.connect(bag_path)
        cur = con.cursor()

        # --- Map topics ---
        topics = cur.execute("SELECT id, name, type FROM topics").fetchall()
        self.topic_map = {name: (id, type) for id, name, type in topics}
        self.get_logger().info(f"Topics in bag: {list(self.topic_map.keys())}")

        # --- Global start time ---
        cur.execute("SELECT MIN(timestamp) FROM messages")
        t0_ns = cur.fetchone()[0]
        self.global_t0 = t0_ns / 1e9

        # --- Create figure ---
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7, 7), sharex=True, dpi=100)

        # --- Subplot 1: Velocity ---
        self.plot_velocity(ax1, cur)

        # --- Subplot 2: Yaw ---
        self.plot_yaw(ax2, cur)

        plt.tight_layout()
        plt.show()

    def plot_velocity(self, ax, cur):
        leader_topic = "/cmd_vel_joy"
        follower_topics = ["/robot0_1/v_hat", "/robot1_0/v_hat"]

        df_leader = self.load_twist(cur, leader_topic)
        if not df_leader.empty:
            ax.plot(df_leader["time"], df_leader["linear_x"], "k--", label="Leader desired V")

        for i, topic in enumerate(follower_topics, start=1):
            df = self.load_float(cur, topic)
            if not df.empty:
                ax.plot(df["time"], df["value"], label=f"Follower {i} estimated V")

        ax.set_ylabel("Velocity (m/s)")
        ax.set_title("Leader desired V vs Followers estimated V")
        ax.grid(True)
        ax.legend()

    def plot_yaw(self, ax, cur):
        yaw_leader_topic = "/robot0_0/yaw_deg"
        yaw_follower_topics = ["/robot0_1/yaw_hat", "/robot1_0/yaw_hat"]

        # Leader yaw
        df_leader = self.load_float(cur, yaw_leader_topic, wrap360=True)
        if not df_leader.empty:
            # Hold at 360° from 0s to 65s
            df_leader.loc[df_leader["time"] <= 65.0, "value"] = 360.0
            ax.plot(df_leader["time"], df_leader["value"], "k--", label=r"Leader desired $\theta$")

        # Followers yaw
        for i, topic in enumerate(yaw_follower_topics, start=1):
            df = self.load_float(cur, topic, radians=True, wrap360=True)
            if not df.empty:
                ax.plot(df["time"], df["value"], label=rf"Follower {i} estimated $\theta$")

        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Heading (deg) ")
        ax.set_title("Leader desired θ vs Followers estimated θ")
        ax.grid(True)
        ax.legend()

    def load_twist(self, cur, topic_name):
        if topic_name not in self.topic_map:
            self.get_logger().warn(f"Topic {topic_name} not found in bag")
            return pd.DataFrame(columns=["time", "linear_x"])

        topic_id, _ = self.topic_map[topic_name]
        rows = cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=?", (topic_id,)).fetchall()

        data = []
        for t, raw in rows:
            msg = deserialize_message(raw, Twist)
            time_s = t / 1e9 - self.global_t0
            data.append([time_s, msg.linear.x])
        return pd.DataFrame(data, columns=["time", "linear_x"])

    def load_float(self, cur, topic_name, radians=False, wrap360=False):
        if topic_name not in self.topic_map:
            self.get_logger().warn(f"Topic {topic_name} not found in bag")
            return pd.DataFrame(columns=["time", "value"])

        topic_id, _ = self.topic_map[topic_name]
        rows = cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=?", (topic_id,)).fetchall()

        data = []
        for t, raw in rows:
            msg = deserialize_message(raw, Float32)
            val = msg.data
            if radians:
                val = math.degrees(val)
            if wrap360:
                val = (val + 360) % 360
            time_s = t / 1e9 - self.global_t0
            data.append([time_s, val])
        return pd.DataFrame(data, columns=["time", "value"])


def main(args=None):
    rclpy.init(args=args)
    plotter = BagPlotter()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
