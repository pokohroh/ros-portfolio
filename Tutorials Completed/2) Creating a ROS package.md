# ROS Tutorial: Creating a Package

# Objective
This tutorial will show how to create a new ROS package using the `catkin` build system.


# Hardware Required
- None (tutorial is entirely software-based)


# Software Required
- Ubuntu 18.04
- ROS Noetic
- Terminal


# Procedure

1. Navigate to Your Catkin Workspace
2. Create a package
3. Check if a package was created
4. Go back to Catkin workspace root
5. Build the workspace
6. Source the workspace environment

 1. Navigate to Your Catkin Workspace
```bash

cd ~/catkin_ws/src
```
If your workspace doesn't exist yet, create it:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

2. Create a New Package

```bash
catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
```

This creates a new folder beginner_tutorials with the required files:

/package.xml
/CMakeLists.txt
/And src/ if needed

3. Check That the Package Was Created

List the contents of the current directory to verify
```bash
ls
```
You should see your package listed (beginner_tutorials).

4. Go Back to the Root of Your Workspace

```bash
cd ~/catkin_ws
```
5. Build the Workspace

```bash
catkin_make
```

6. Source the Workspace Environment
```bash

source devel/setup.bash
```

To make this permanent

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
