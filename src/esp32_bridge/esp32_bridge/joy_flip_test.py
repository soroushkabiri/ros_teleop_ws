#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoyFlipTester(Node):
    def __init__(self):
        super().__init__('joy_flip_tester')
        self.flip_orientation = False
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.get_logger().info("JoyFlipTester started. Press button 2 on your joystick to test.")

    def joy_callback(self, msg):
        # Example: button 2 (X on Xbox / Circle on PS)
        if msg.buttons[2] == 1:
            if not self.flip_orientation:  # Only log change
                self.get_logger().info("Flip orientation: ON")
            self.flip_orientation = True
        else:
            if self.flip_orientation:  # Only log change
                self.get_logger().info("Flip orientation: OFF")
            self.flip_orientation = False

def main(args=None):
    rclpy.init(args=args)
    node = JoyFlipTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
