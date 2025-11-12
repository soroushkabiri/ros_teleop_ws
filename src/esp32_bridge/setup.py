from setuptools import find_packages, setup

package_name = 'esp32_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        #(f'share/{package_name}/config', ['src/esp32_bridge/config/joystick.yaml']),

        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/esp32_bringup.launch.py']),
        ('share/' + package_name + '/launch', ['launch/imu_yaw.launch.py']),
        ('share/' + package_name + '/launch', ['launch/esp32_pwm_initial.launch.py']),
        ('share/' + package_name + '/launch', ['launch/esp32_better_pid.launch.py']),



    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='soroush',
    maintainer_email='soroush@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                        'esp32_teleop_bridge = esp32_bridge.esp32_teleop_bridge:main',
                        'state_consensus_node = esp32_bridge.state_consensus_node:main',
                        'pub_des_vel_node = esp32_bridge.pub_des_vel_node:main',
                        'esp32_pwm_test = esp32_bridge.esp32_pwm_test:main',
                        'esp32_better_pid = esp32_bridge.esp32_better_pid:main',
                        'test_joy_node = esp32_bridge.joy_flip_test:main',
                        'web_command = esp32_bridge.web_command:main',
                        'compute_closest_obstacle = esp32_bridge.compute_closest_obstacle:main',
                        'fuzzy_planner = esp32_bridge.fuzzy_planner:main',
                        'yaw_degree_from_odom = esp32_bridge.yaw_degree_from_odom:main',
                        'bag_recorder = esp32_bridge.bag_recorder:main',
                        'bag_plotter = esp32_bridge.bag_plotter:main',
                        'map_data = esp32_bridge.map_data:main',
                        'waypoint_planner = esp32_bridge.waypoint_planner:main',
                        'map_localization_node = esp32_bridge.map_localization_node:main',

        ],
    },
)


