#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String
import math
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy

class YawExtractor(Node):
    def __init__(self):
        super().__init__('yaw_extractor')


        # the variable to set how many followers i have.
        self.followers_number = '2'  
        # Subscribe to /followers_number to know when the process should begin
        self.create_subscription(String, '/followers_number', self.followers_number_callback, 10)

        # Subscriptions to IMUs
        self.subscription_L = self.create_subscription(Imu,'/imu/data/L',self.imu_callback_L,10)
        self.publisher_L = self.create_publisher(Float32, '/robot0_0/yaw_deg', 10)

        self.subscription_F1 = self.create_subscription(Imu,'/imu/data/F1',self.imu_callback_F1,10)
        self.publisher_F1 = self.create_publisher(Float32, '/robot0_1/yaw_deg', 10)

        self.subscription_F2 = self.create_subscription(Imu,'/imu/data/F2',self.imu_callback_F2,10)
        self.publisher_F2 = self.create_publisher(Float32, '/robot1_0/yaw_deg', 10)

        # Subscribe to calibration command
        self.robots_command_yaw_calib = 'calibrating'  # default
        self.create_subscription(String, '/robot_command_yaw_calib', self.robots_command_yaw_calib_callback, 10)

        # Subscribe to external reference yaw
        self.rect_obj_odom_yaw = 0.0
        # publisher so you can `ros2 topic echo /rect_obj_odom_yaw`
        self.rect_obj_odom_pub = self.create_publisher(Float32, '/rect_obj_odom_yaw', 10)
        # QoS to match rtabmap best-effort publisher

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.create_subscription(Odometry, '/rtabmap/odom', self.odom_callback, qos)

        # Offsets (used after calibration ends)
        self.offset_yaw_L = None
        self.offset_yaw_F1 = None
        self.offset_yaw_F2 = None

        self.get_logger().info('Yaw Extractor Node has been started.')


    def followers_number_callback(self, msg):
        self.followers_number = msg.data   # store the string ("1" or "2")

    def robots_command_yaw_calib_callback(self, msg):        
        prev_state = self.robots_command_yaw_calib
        self.robots_command_yaw_calib = msg.data   # "calibrating" or "calibrated"
        self.get_logger().info(f"Calibration state changed to: {self.robots_command_yaw_calib}")

        # When switching from calibrating -> calibrated, set offsets
        if prev_state == "calibrating" and self.robots_command_yaw_calib == "calibrated":
            if self.last_imu_yaw_L is not None:
                self.offset_yaw_L = self.last_imu_yaw_L - self.rect_obj_odom_yaw
                self.get_logger().info(f"Leader offset set to {self.offset_yaw_L:.2f}")
            if self.last_imu_yaw_F1 is not None:
                self.offset_yaw_F1 = self.last_imu_yaw_F1 - self.rect_obj_odom_yaw
                self.get_logger().info(f"Follower1 offset set to {self.offset_yaw_F1:.2f}")

            #if self.followers_number=='2':
            if self.last_imu_yaw_F2 is not None:
                self.offset_yaw_F2 = self.last_imu_yaw_F2 - self.rect_obj_odom_yaw
                self.get_logger().info(f"Follower2 offset set to {self.offset_yaw_F2:.2f}")


    def odom_callback(self, msg: Odometry):
        self.get_logger().info("Received odom message")
        q = msg.pose.pose.orientation
        yaw_rad = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        yaw_deg = math.degrees(yaw_rad)
        self.rect_obj_odom_yaw = math.degrees(yaw_rad)
        self.rect_obj_odom_pub.publish(Float32(data=self.rect_obj_odom_yaw))


    def imu_callback_L(self, msg):
        # Extract raw IMU yaw
        q = msg.orientation
        imu_yaw_rad = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        imu_yaw_deg = math.degrees(imu_yaw_rad)
        self.last_imu_yaw_L = imu_yaw_deg

        if self.robots_command_yaw_calib == "calibrating":
            yaw_out = self.rect_obj_odom_yaw
        else:  # calibrated
            if self.offset_yaw_L is None:
                yaw_out = self.rect_obj_odom_yaw
            else:
                yaw_out = imu_yaw_deg - self.offset_yaw_L

        # Normalize to [-180, 180]
        yaw_out = (yaw_out + 180) % 360 - 180
        self.publisher_L.publish(Float32(data=yaw_out))

    def imu_callback_F1(self, msg):
        # Extract raw IMU yaw
        q = msg.orientation
        imu_yaw_rad = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        imu_yaw_deg = math.degrees(imu_yaw_rad)
        self.last_imu_yaw_F1 = imu_yaw_deg

        if self.robots_command_yaw_calib == "calibrating":
            yaw_out = self.rect_obj_odom_yaw
        else:  # calibrated
            if self.offset_yaw_F1 is None:
                yaw_out = self.rect_obj_odom_yaw
            else:
                yaw_out = imu_yaw_deg - self.offset_yaw_F1

        # Normalize to [-180, 180]
        yaw_out = (yaw_out + 180) % 360 - 180
        self.publisher_F1.publish(Float32(data=yaw_out))


    def imu_callback_F2(self, msg):
        # Extract raw IMU yaw
        q = msg.orientation
        imu_yaw_rad = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)
        imu_yaw_deg = math.degrees(imu_yaw_rad)
        self.last_imu_yaw_F2 = imu_yaw_deg

        if self.robots_command_yaw_calib == "calibrating":
            yaw_out = self.rect_obj_odom_yaw
        else:  # calibrated
            if self.offset_yaw_F2 is None:
                yaw_out = self.rect_obj_odom_yaw
            else:
                yaw_out = imu_yaw_deg - self.offset_yaw_F2

        # Normalize to [-180, 180]
        yaw_out = (yaw_out + 180) % 360 - 180
        self.publisher_F2.publish(Float32(data=yaw_out))

    def quaternion_to_yaw(self, x, y, z, w):
        # Convert quaternion to yaw angle (Z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

def main(args=None):
    rclpy.init(args=args)
    node = YawExtractor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
