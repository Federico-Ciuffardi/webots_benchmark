# webots_benchmark

A simple ros-webots simulation used as a benchmark.

![Demo](https://i.imgur.com/XF1R8Nk.gif)

## Modified controller
`controllers/` contains a modified version of the standard webots ROS Controller that, in addition to the features of the standard webots controller adds:
* A topic on `[robot_name]/pose` were the robot pose (`geometry_msgs/PoseStamped` ) is published periodically on each simulation step.
* A tf broadcast of the robot (`[robot_name]`) and its lidar frames (`[robot_name]/laser`).

## Known issues
* Robots wobble vertically as they move.
* Lidar values seems to be slightly tilted.

## How to run

* Install ROS Desktop-Full: http://wiki.ros.org/ROS/Installation

* Install Webots: https://cyberbotics.com/doc/guide/installation-procedure

* Create a catkin workspace: http://wiki.ros.org/catkin/Tutorials/create_a_workspace

* Execute: `cd ~/catkin_ws/src`

* Execute: `git clone https://github.com/cyberbotics/webots_ros`

* Execute: `git clone https://github.com/Federico-Ciuffardi/webots_benchmark`

* From now on execute the following command to run the simulation:`~/catkin_ws/src/webots_benchmark/run_simulation.sh`
