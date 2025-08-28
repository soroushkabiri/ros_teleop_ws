from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    esp32_teleop=Node(
            package='esp32_bridge',
            executable='esp32_pwm_test',
            name='esp32_bridge',
            output='screen',
        )

    return LaunchDescription([
        esp32_teleop,
    ])