# ROS Tutorial: Understanding Services and Parameters

# Objective
- To learn about ROS Services (request/reply communication) and ROS Parameters (global configuration variables), and how to interact with them using command-line tools.

# Hardware Required
- None

# Software Required
- Ubuntu 20.04
- ROS Noetic
- Terminal
- Properly built and sourced catkin workspace

# Procedure
1. Launch the ROS Master and turtlesim

```bash
roscore & rosrun turtlesim turtlesim_node
```
Starts the ROS master and the turtlesim simulator.

2. List Active Services
```bash
rosservice list
```
Output includes services like

```bash
/clear  
/kill  
/reset  
/spawn  
/turtle1/set_pen  
/turtle1/teleport_absolute  
/turtle1/teleport_relative  
```
3. Inspect a Service

Check the details of /spawn (creates a new turtle):

```bash
rosservice info /spawn
```
Output shows:

Service type: turtlesim/Spawn

Node: /turtlesim

Args: x y theta name

4. Call a Service

Spawn a new turtle at (x=5.0, y=5.0, theta=0.0):

```bash
rosservice call /spawn 5.0 5.0 0.0 "new_turtle"
```
Verify by checking topics (rostopic list) or the turtlesim window.

5. List ROS Parameters
```bash
rosparam list
```
Default parameters include:

```bash
/background_b  
/background_g  
/background_r  
/rosdistro  
/rosversion  
```
6. Get/Set a Parameter
Get the background color (red channel):

```bash
rosparam get /background_r
```
(Default: 69)

Set the background to red:

```bash
rosparam set /background_r 255  
rosservice call /clear "{}"  # Apply changes  
```
7. Save/Load Parameters
Save all parameters to a file:

```bash
rosparam dump ~/turtlesim_params.yaml
```
Load parameters from a file:

```bash
rosparam load ~/turtlesim_params.yaml
```
