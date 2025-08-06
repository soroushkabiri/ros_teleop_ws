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

        ],
    },
)
