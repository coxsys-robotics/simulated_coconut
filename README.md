# simulated_coconut
This repository contains ROS Packages that relates the simulated COCONUT robot in Gazebo. This is done with ROS-Noetic

You need the following packages

aruco_detect

```
sudo apt install ros-noetic-effort-controllers
sudo apt install ros-noetic-aruco-detect
roscd xxx_utilities/scripts
chmod +x xxx_odom.py
chmod +x xxx_ik.py
chmod +x aruco2robot.py
chmod +x gazebo2robot.py
roscd xxx_control.py
chmod +x goal_giver.py
chmod +x turtle2pose.py
chmod +x pose_follower.py
roslaunch xxx_bringup xxx_gazebo_aruco_localization.launch
```
