from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    esp32_teleop=Node(
            package='esp32_bridge',
            executable='esp32_better_pid',
            name='esp32_bridge',
            output='screen',
        )



    joy_node=Node(
        package='joy',
        executable='joy_node',
        #parameters=[joy_params],
    )

    teleop_node=Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        #parameters=[joy_params],
        parameters=[{
            'axis_linear': {'x': 1},
            'scale_linear': {'x': 0.35},
            'scale_linear_turbo': {'x': 0.7},

            'axis_angular': {'yaw': 0},
            'scale_angular': {'yaw': 0.7},
            'scale_angular_turbo': {'yaw': 1.4},

            'enable_button': 4,
            'enable_turbo_button': 5,
            'require_enable_button': True,
        }]
    )




    return LaunchDescription([
        esp32_teleop,
        joy_node,
        teleop_node
    ])


