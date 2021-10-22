# Rover_webots_ros2_bridge

Rover is a project that seeks to develop and build an autonomous robot that can compete in the URC, currently under development, has managed to make a connection between Webots and ROS2 and has been able to obtain information from all sensors and generate a map to simulate the characteristics of the navigation test in, also is currently working on a robust system of perception, with stereo camera, lidar, gps, imu 

# Rviz2 Visualization
![](https://github.com/UaoRover/Rover_webots_ros2_bridge/blob/main/Rviz%20Visualization.png)

## Webots Enviroment

![](https://github.com/UaoRover/Rover_webots_ros2_bridge/blob/main/Webots%20Enviroment.png)

# Run
sudo apt-get install ros-foxy-webots-ros2

sudo apt install ros-foxy-joint-state-publisher-gui

sudo apt install ros-foxy-xacro

sudo apt install ros-foxy-depth-image-proc

sudo apt install ros-foxy-robot-localization


sudo apt install ros-foxy-depth_image_proc

ros2 launch rover_webots rviz_webots_real.launch.py
