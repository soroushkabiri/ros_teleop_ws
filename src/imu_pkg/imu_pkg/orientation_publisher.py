

# imu_mag_wifi_node_async.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import math
import asyncio
import aiohttp
import threading

ESP32_IP_L = "192.168.1.121"  # Default IP
ESP32_IP_F1 = "192.168.1.120"  # Default IP
ESP32_IP_F2 = "192.168.1.119"  # Default IP

# ESP32_IP_L = "172.20.10.2"  # Default IP
# ESP32_IP_F1 = "172.20.10.5"  # Default IP
# ESP32_IP_F2 = "172.20.10.4"  # Default IP



class IMUWIFINode(Node):
    def __init__(self):
        super().__init__('imu_wifi_node_async')

        # number of followers
        self.followers_number = '2'
        self.create_subscription(String, '/followers_number', self.followers_number_callback, 10)

        # publishers
        self.pub_imu_L  = self.create_publisher(Imu, 'imu/data_raw/L', 10)
        self.pub_imu_F1 = self.create_publisher(Imu, 'imu/data_raw/F1', 10)
        self.pub_imu_F2 = self.create_publisher(Imu, 'imu/data_raw/F2', 10)

        # start asyncio loop in a background thread
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self.loop.run_forever, daemon=True).start()

        # schedule periodic task
        asyncio.run_coroutine_threadsafe(self.periodic_task(), self.loop)

    def followers_number_callback(self, msg):
        self.followers_number = msg.data

    async def fetch_json(self, session, url):
        try:
            async with session.get(url, timeout=0.3) as resp:
                if resp.status == 200:
                    return await resp.json()
                else:
                    self.get_logger().warn(f"HTTP {resp.status} from {url}")
        except Exception as e:
            self.get_logger().warn(f"Failed request {url}: {e}")
        return None

    async def periodic_task(self):
        async with aiohttp.ClientSession() as session:
            while rclpy.ok():
                # fetch all ESP32 in parallel
                tasks = {
                    "L":  self.fetch_json(session, f"http://{ESP32_IP_L}/imu_L"),
                    "F1": self.fetch_json(session, f"http://{ESP32_IP_F1}/imu_F1"),
                    "F2": self.fetch_json(session, f"http://{ESP32_IP_F2}/imu_F2"),
                }
                results = await asyncio.gather(*tasks.values(), return_exceptions=True)

                # map results back
                for name, result in zip(tasks.keys(), results):
                    if isinstance(result, dict):  # valid JSON
                        if name == "L":  self.parse_json_L(result)
                        if name == "F1": self.parse_json_F1(result)
                        if name == "F2": self.parse_json_F2(result)

                await asyncio.sleep(1/15)  # 15 Hz

    def parse_json_L(self, data):
        try:
            imu_msg = Imu()
            imu_msg.linear_acceleration.x = float(data["ax_L"]) * 9.81
            imu_msg.linear_acceleration.y = float(data["ay_L"]) * 9.81
            imu_msg.linear_acceleration.z = float(data["az_L"]) * 9.81
            imu_msg.angular_velocity.x = math.radians(float(data["gx_L"]))
            imu_msg.angular_velocity.y = math.radians(float(data["gy_L"]))
            imu_msg.angular_velocity.z = math.radians(float(data["gz_L"]))
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "base_link_L"
            self.pub_imu_L.publish(imu_msg)
        except Exception as e:
            self.get_logger().warn(f"JSON parsing error L: {e}")

    def parse_json_F1(self, data):
        try:
            imu_msg = Imu()
            imu_msg.linear_acceleration.x = float(data["ax_F1"]) * 9.81
            imu_msg.linear_acceleration.y = float(data["ay_F1"]) * 9.81
            imu_msg.linear_acceleration.z = -float(data["az_F1"]) * 9.81
            imu_msg.angular_velocity.x = math.radians(float(data["gx_F1"]))
            imu_msg.angular_velocity.y = math.radians(float(data["gy_F1"]))
            imu_msg.angular_velocity.z = math.radians(float(data["gz_F1"]))
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "base_link_F1"
            self.pub_imu_F1.publish(imu_msg)
        except Exception as e:
            self.get_logger().warn(f"JSON parsing error F1: {e}")

    def parse_json_F2(self, data):
        try:
            imu_msg = Imu()
            imu_msg.linear_acceleration.x = float(data["ax_F2"]) * 9.81
            imu_msg.linear_acceleration.y = float(data["ay_F2"]) * 9.81
            imu_msg.linear_acceleration.z = float(data["az_F2"]) * 9.81
            imu_msg.angular_velocity.x = math.radians(float(data["gx_F2"]))
            imu_msg.angular_velocity.y = math.radians(float(data["gy_F2"]))
            imu_msg.angular_velocity.z = math.radians(float(data["gz_F2"]))
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "base_link_F2"
            self.pub_imu_F2.publish(imu_msg)
        except Exception as e:
            self.get_logger().warn(f"JSON parsing error F2: {e}")


def main():
    rclpy.init()
    node = IMUWIFINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
