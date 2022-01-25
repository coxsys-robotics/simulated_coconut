# simulated_coconut
This repository contains ROS Packages that relates the simulated COCONUT robot in Gazebo. This is done with ROS-Noetic

You need the following packages

aruco_detect

```
sudo apt install ros-noetic-effort-controllers
sudo apt install ros-noetic-aruco-detect
sudo apt install ros-noetic-robot-localization
roscd xxx_utilities/scripts
chmod +x xxx_odom.py
chmod +x xxx_ik.py
chmod +x aruco2robot.py
chmod +x gazebo2robot.py
roscd xxx_control/scripts
chmod +x goal_giver.py
chmod +x turtlesim2pose.py
chmod +x pose_follower.py
roscd xxx_teleop/scripts
chmod +x teleop.py
roscd xxx_diff_drive/scripts
chmod +x diff_drive.py
roscd sim_with_kalman/scripts
chmod +x sim_encoder.py
chmod +x unwrapper.py
roslaunch xxx_bringup xxx_gazebo_localization.launch
```
