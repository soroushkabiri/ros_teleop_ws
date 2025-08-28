# this launch file  
# 1) initialize consensus observer 
# 3) initialize desired cmd_vel node

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction

def generate_launch_description():

    ld = LaunchDescription()
    consensus_node = TimerAction(
        period=0.1,  # delay in seconds
        actions=[
            Node(
                package='esp32_bridge',
                executable='state_consensus_node',
                name='initialize_consensus_observer',
                output='screen',
            )
        ]
    )
    ld.add_action(consensus_node)  

    # Desired Velocity Publisher Node (delayed by 10 seconds total)
    des_publisher_node = TimerAction(
        period=1.0,  # delay in seconds (IMU + consensus time)
        actions=[
            Node(
                package='esp32_bridge',
                executable='pub_des_vel_node',
                name='initialize_des_publisher',
                output='screen',
                parameters=[{'kp_angular':1.0,
                             'ki_angular': 0.02,
                             'max_integral_angular':10.0}]
            )
        ]
    )
    ld.add_action(des_publisher_node)

    return ld


#how to used parameters:
#ros2 param set /initialize_des_publisher kp_angular 0.7
#ros2 param set /initialize_des_publisher ki_angular 0.05
#ros2 param set /initialize_des_publisher max_integral_angular 5.0