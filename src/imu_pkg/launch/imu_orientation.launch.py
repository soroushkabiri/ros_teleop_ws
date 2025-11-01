
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Your serial reader
        Node(
            package='imu_pkg',
            executable='orientation_publisher',
            name='imu_serial_node',
            output='screen'
        ),
        # Madgwick filter for Leader
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='madgwick_filter_leader',
            output='screen',
            parameters=[{
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu',  # or 'nwu' if you prefer
                'magnetic_declination_radians': 0.0,
                'reverse_mag_y': False,
                'reverse_mag_z': False
            }],
            remappings=[
                ('imu/data_raw', 'imu/data_raw/L'),
                ('imu/mag', 'imu/mag'),
                ('imu/data', 'imu/data/L')  # final orientation
            ]
        ),

        # Madgwick filter for follower 1
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='madgwick_filter_follower1',
            output='screen',
            parameters=[{
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu',  # or 'nwu' if you prefer
                'magnetic_declination_radians': 0.0,
                'reverse_mag_y': False,
                'reverse_mag_z': False
            }],
            remappings=[
                ('imu/data_raw', 'imu/data_raw/F1'),
                ('imu/mag', 'imu/mag'),
                ('imu/data', 'imu/data/F1')  # final orientation
            ]
        ),

        # Madgwick filter for follower 2
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='madgwick_filter_follower2',
            output='screen',
            parameters=[{
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu',  # or 'nwu' if you prefer
                'magnetic_declination_radians': 0.0,
                'reverse_mag_y': False,
                'reverse_mag_z': False
            }],
            remappings=[
                ('imu/data_raw', 'imu/data_raw/F2'),
                ('imu/mag', 'imu/mag'),
                ('imu/data', 'imu/data/F2')  # final orientation
            ]
        ),


        # magdwich to yaw
        Node(
            package='imu_pkg',
            executable='madgwick_to_yaw',
            name='yaw_extractor',
            output='screen'
        ),
           
        
    ])
