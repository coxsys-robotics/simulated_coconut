# simulated_coconut
This repository contains ROS Packages that relates the simulated COCONUT robot in Gazebo. This is done with ROS-Noetic

You need the following packages

aruco_detect

fiducial_slam

```
sudo apt install ros-noetic-effort-controllers
sudo apt install ros-noetic-aruco-detect
sudo apt install ros-noetic-fiducial-slam
roscd xxx_utilities/scripts
chmod +x xxx_odom.py
roslaunch xxx_bringup xxx_gazebo_aruco_localization.launch
```
