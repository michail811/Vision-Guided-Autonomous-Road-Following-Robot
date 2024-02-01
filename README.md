# Vision-Guided-Autonomous-Road-Following-Robot
This repository contains the source code for a vision-based navigation system implemented in Python using the Robot Operating System (ROS). The project focuses on enabling a robot to autonomously follow road lanes by processing camera images and making real-time navigation decisions.

## Overview

This repository includes two key components:

1. **Lanes Detection and heading angle error calculation  Node (`lanes_detection_and_goal_angle_calculation.py`):**
   - Subscribes to the camera topic for receiving image messages.
   - Applies advanced image processing techniques.
   - Implements a perspective transformation to obtain a top-down view of the road.
   - Uses Hough lines for accurate left and right lane detection, determining their slopes and intercepts.
   - Calculates the average lane position for stable road following and calculates the error angle of the heading line and the desired direction
   - Publishes line detection information to the `/line_follower/detector` topic.

2. **Line Following Controller Node (`line_controller.py`):**
   - Subscribes to line detection information from `/line_follower/detector`.
   - Determines the goal angle based on the detected line's orientation.
   - Implements a trigonometrical controller or kp classical controller to derive precise linear and angular velocities for the robot.
   - Publishes velocity commands to `/cmd_vel` to control the robot's movement.

## Dependencies
- Ubuntu 18.04 bionic
- ROS (Robot Operating System) Melodic
- Python 2.7
- OpenCV
- NumPy


## Installation
- Create a New Catkin Workspace:

  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws
  catkin_make

- Clone the Repository into src Directory:

  cd ~/catkin_ws/src
  git clone <repository_url>

- Build the Catkin Workspace:
  cd ~/catkin_ws
  catkin_make

- Source the Catkin Workspace:
  source devel/setup.bash

- optional:Install System Dependencies with rosdep
  Replace <your_ros_distro> with your ROS distribution (e.g., melodic, noetic).

  rosdep install --from-paths src --ignore-src --rosdistro <your_ros_distro> -y
  
## Run the ROS Package:
  Navigate to your workspace. For every new terminal you open source it.

  1st terminal: roslaunch <package name> <launch file>.launch , # launch your turtlebot
  2nd terminal: rosrun line_detection_package line_detection_and_goal_angle_calculation.py
  3d terminal:  rosrun line_detection_package road_following_controller.py 

## Customization
Experiment with vision-preprocessing (line_detection_and_goal_angle_calculation.py), and controller parameters (road_following_controller.py ) to suit your robot's specifications. Also, experiment with the camera's physical position to ensure road lanes are within the region of interest and have a clear visual perspective.
Modify the vision processing pipeline in line_detection_and_goal_angle_calculation.py for improved lane detection. Tweak parameters to achieve optimal performance based on your environment.

## Authors
Michail Potamitis 

## License
This project is licensed under the MIT License.
