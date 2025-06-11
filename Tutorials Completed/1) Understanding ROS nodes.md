# ROS Tutorial: Understanding Nodes

# Objective
To understand what a **ROS Node** is, how nodes communicate within the ROS computational graph, and how to view and manage them using ROS command-line tools.

---

# Hardware Required
- None

---

# Software Required
- Ubuntu 18.04
- ROS Noetic
- Terminal
- Properly built and sourced catkin workspace

---

# Procedure

Start ROS master in your terminal with line
```bash
roscore
```
> [!WARNING]
> You can only run ROS master in one terminal. You cannot start another roscore without terminating the other one.

Then start a node with line
```bash
rosrun [name-of-the-package] [name-of-the-node]
```
For example, 
```bash
rosrun rospy_tutorials talker
```
![Image](https://github.com/user-attachments/assets/c81751d9-b8eb-4859-a201-d25d61f03bbf)
