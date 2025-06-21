# ROS Tutorial: Understanding Service and Parameters

## Objective
To learn about ROS Services (request/reply communication) and ROS Parameters (global configuration variables), and how to interact with them using command-line tools.

## Procedure
1. Launch the ROS Master and turtlesim

```bash
roscore
rosrun turtlesim turtlesim_node
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
3. Inspect a service

Check the details of /set_pen (setting the color and width of the turtle's trace):

```bash
rosservice info /turtle1/set_pen
```

Output shows:
```bash
Node: /turtlesim
Type: turtlesim/set_pen
Args: r g b width off
```
4. Call a service

Firstly, in a new terminal, run turtle_teleop_key node to move the turtle around.
```bash
rosrun turtlesim turtle_teleop_key
```
![Image](https://github.com/user-attachments/assets/cd94a523-404c-4d4d-a773-c652a56ed44b)
The trace (pen) color should be default white.

Now, we set turtle pen's color red (r: 200, g: 0, b: 0) by calling *turtle1/set_pen* service

```bash
rosservice call /turtle1/set_pen "{r: 200, g: 0, b: 0, width: 4, 'off': 0}"
```
![Image](https://github.com/user-attachments/assets/2a572ace-51c5-470c-940c-127538e22f6e)

The pen color is now red.

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
