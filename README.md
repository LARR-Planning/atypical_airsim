# atypical_airsim

This code is modified version of [here](https://github.com/microsoft/AirSim/blob/master/docs/airsim_ros_pkgs.md) for use of ROS in autonomous vehicle research.

## Dependency
octomap_server  
turtlebot3_teleop_key

## Installation
1) Install and build Unreal Engine 4.24 and AirSim. Follow instructions from AirSim documentation (https://microsoft.github.io/AirSim/build_linux/)
2) Move to Airsim/ros/src directory and clone the package
3) Build the current package 

```bash
cd ~/AirSim/ros/src
git clone https://github.com/LARR-Planning/atypical_airsim.git
catkin build atypical_ros
source ../devel/setup.bash
```

## Run
1) Manual control with keyboard
```bash 
roslaunch atypical_ros airsim_all
```

2) External planner
```bash 
roslaunch atypical_ros airsim_with_planner
```
