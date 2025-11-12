# Code contains of ros2 packages to connect with actual differential drive robots

## build
colcon build
source install/setup.bash

## differential drive robots
for this robot we use a robot with imu and encoders and an esp32 microcontroller that can connect with a rectangular pallete and move it via passive links:

<img width="682" height="406" alt="diff_drive_real_robot" src="https://github.com/user-attachments/assets/6762cb2e-1928-416d-996b-be1ca8c37b60" />

## rectangular object
this is the object that will be transport. this recrangular object has a kinect depth camera that uses it with rtabmap package to doing visual slam and visual odometry

<img width="581" height="413" alt="rect_obj_real" src="https://github.com/user-attachments/assets/35f26d86-a5f2-4626-a83f-df752f1400ba" />

## ros2 packages
### imu_pkg
it gets imu data from esp32 of each robots and process them with madgwick_filter package and find orientation of each robot
### esp32_bridge
this package is the main way to communicate between esp32 on each robot and center computer. with this code each robot take the desired velocity and apply them on robots.

### mapping
the slam of this system performed by rtabmap. for mapping the environment use this bash script:
actual_robot_mapping.sh 
and for saving the map:
actual_robot_map_saving.sh
### transporting the object:
first the web command node should be run so the user can define the goal of robots:
ros2 run esp32_bridge web_command
after that this bash script should run:
actual_robot_localising_and_teleop.sh

![Uploading rtab.png…]()

