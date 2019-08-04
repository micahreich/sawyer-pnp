# Sawyer + Robotiq 2f 85 Gripper Pick and Place

Sawyer + Robotiq 2f 85 gripper pick and place is a repository which allows for pick and place functionality within Gazebo simulator. This code can be applied to a real Sawyer and Robotiq gripper as well.

## Installation
This repository depends on the following (clone into catkin workspace ``src`` folder):

[ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

[Robotiq Package](https://github.com/thinclab/robotiq.git)

[Sawyer IRL Project](https://github.com/prasuchit/sawyer_irl_project.git)

[Sawyer MoveIt Package](https://google.com)

[Sawyer Robot Package](https://github.com/thinclab/sawyer_robot.git)

[Intera SDK](https://github.com/RethinkRobotics/intera_sdk.git)

[Intera Common Package](https://github.com/RethinkRobotics/intera_common.git)

[Gazebo ROS Link Attacher Plugin](https://github.com/pal-robotics/gazebo_ros_link_attacher)

The contents of this repository can not all be installed all in one location. Please refer to the subdirectories' READMEs for information regarding installation of said directory / folder. 

## Usage
First, navigate to your ``catkin_ws`` folder in a new terminal window and make sure the workspace has been sourced properly:

```bash
cd ~/catkin_ws/
source devel/setup.bash
```


Next, start ``robot_gazebo.launch``:

```bash
roslaunch sawyer_irl_project robot_gazebo.launch
```

In a new terminal window, once ``robot_gazebo.launch`` has finished starting RViz and Gazebo, start ``simple_pnp_gazebo.py``:

```bash
rosrun robotiq_2f_gripper_control simple_pnp_gazebo.launch
```
