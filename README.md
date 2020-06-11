# atypical_airsim

This code is modified version of [here](https://github.com/microsoft/AirSim/blob/master/docs/airsim_ros_pkgs.md) for use of ROS in autonomous vehicle research.

## Installation
1) Install and build Unreal Engine 4.24 and AirSim. Follow instructions from AirSim documentation (https://microsoft.github.io/AirSim/build_linux/)
2) Move to Airsim/ros/src directory and clone the package
3) Build current ros package 

```bash
cd ~/AirSim/ros/src
git clone https://github.com/LARR-Planning/atypical_airsim.git
catkin build atypical_airsim
source devel/setup.bash
```
