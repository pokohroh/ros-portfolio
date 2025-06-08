# ROS Tutorial: Building Packages

# Objective
To build a ROS package using the `catkin_make` build system and understand how ROS dependencies are resolved during the build process.


# Hardware Required
- None


# Software Required
- Ubuntu 20.04
- ROS Noetic
- A properly initialized `catkin` workspace


# Procedure

 1. Create a Catkin Workspace (if not already created)

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
