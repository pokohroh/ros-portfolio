# ROS-portfolio GROUP 7
* CURRENTLY STILL IN PROGRESS *

Welcome to our learning portfolio, where we demonstrate our initial competencies in ROS (Robot Operating System). This project is part of our journey to understand robotic systems through hands-on tutorials and beginner simulations.



## For General Audience

We are a group of students learning how to control robots using software. We used a simulated turtle robot to explore how robots move and respond to commands. Think of it like learning how to talk to a robot and make it do simple things like move forward, turn, or draw shapes, all using code!


## ⚙️ For Technical Audience

This portfolio showcases our beginner-level ROS skills, including:

1. Understanding ROS Node [Understanding ROS Node](#understanding-ros-node).
2. Understanding ROS Topic [Understanding ROS Topic](#understanding-ros-topic).
---

# Setup
---

# Tutorials

## Understanding ROS Node

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
## Understanding ROS Topic

---
