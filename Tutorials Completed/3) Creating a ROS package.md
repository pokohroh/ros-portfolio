# ROS Tutorial: Creating a ROS Package

## Objective
This tutorial will show how to create a new ROS package using the `catkin` build system.

## Procedure

1. Navigate to Your Catkin Workspace
```bash

cd catkin_ws/src
```

2. Create a New Package
```bash
catkin_create_pkg [name_of_the_package] [name_of_libraries]
```
For this tutorial, name of the package will be ***my_robot_controller***, and libraries that would be used are ***rospy*** and ***turtlesim***.

***rospy*** is a python libraty that allows python code to get access to ros functionalities.

```bash
catkin_create_pkg my_robot_controller rospy turtlesim
```


This creates a new folder my_robot_controller with the required files:

/package.xml<br/>
/CMakeLists.txt<br/>
/src<br/>


3. Go Back to the Root of Your Workspace and build catkin workspace

```bash
cd ..
catkin_make
```

4. Under catkin_ws/src, open VScode Editor

```bash
cd catkin_ws/src/
code
```

If VScode is not installed, run

```bash
sudo snap install code --clasic
```

Now, catkin workspace for our ros package has been setup, and we are ready to code.
