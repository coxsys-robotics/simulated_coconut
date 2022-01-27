# simulated_coconut
This repository contains ROS Packages that relates the simulated COCONUT robot in Gazebo. This is done with ROS-Noetic

You need the following packages

effort_controllers
aruco_detect
robot_localization
map_server
gmapping
amcl
teb_local_planner
navigation
move_base

```
sudo apt install ros-noetic-effort-controllers
sudo apt install ros-noetic-aruco-detect
sudo apt install ros-noetic-robot-localization
sudo apt install ros-noetic-map-server
sudo apt install ros-noetic-gmapping
sudo apt install ros-noetic-amcl
sudo apt install ros-noetic-teb-local-planner
sudo apt install ros-noetic-navigation
sudo apt install ros-noetic-move-base
roscd xxx_bringup/scripts
chmod +x state_machine.py
chmod +x my_state_machine.py
roscd xxx_gui/scripts
chmod +x xxx_gui.py
chmod +x my_gui.py
roscd xxx_navigation/scripts
chmod +x check_status.py
chmod +x warp_prevention.py
roscd xxx_utilities/scripts
chmod +x xxx_joint_state_effort.py
chmod +x xxx_joint_state_vel.py
chmod +x xxx_odom.py
chmod +x xxx_ik.py
chmod +x aruco2robot.py
chmod +x gazebo2robot.py
roscd xxx_control/scripts
chmod +x goal_giver.py
chmod +x turtlesim2pose.py
chmod +x pose_follower.py
chmod +x teleop.py
roscd sim_with_kalman/scripts
chmod +x sim_encoder.py
chmod +x unwrapper.py
roslaunch xxx_bringup xxx_gazebo_robot_localization.launch
roslaunch xxx_navigation navigation.launch

```
