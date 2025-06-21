# ROS Tutorial: Running Turtlesim Node

## Objective
- To run and explore turtlesim node, which will be used throughout the tutorial 

## Procedure
1. Launch the ROS Master
```bash
roscore
```
This starts the ROS master, which facilitates node registration and communication.

2. Run the turtlesim Node
Open a new terminal and run:

```bash
rosrun turtlesim turtlesim_node
```
This launches the turtlesim simulator, creating a node named /turtlesim.

3. List Active Nodes
In another terminal, execute:

```bash
rosnode list
```
Expected output:

```bash
/rosout
/turtlesim
/rosout is a default logging node, while /turtlesim is the running simulator.
```
4. Get Node Information
To inspect details about the turtlesim node:

```bash
rosnode info /turtlesim
```
Output includes:

Publications (topics it publishes to, e.g., /turtle1/pose)

Subscriptions (topics it listens to, e.g., /turtle1/cmd_vel)

Services (e.g., /turtlesim/teleport_absolute)

5. Run the Teleoperation Node
In a new terminal, start the keyboard teleop node:

```bash
rosrun turtlesim turtle_teleop_key
```
This creates a new node (typically named /teleop_turtle) that publishes velocity commands to /turtle1/cmd_vel.

6. Test Node Connectivity
Check if the turtlesim node is responsive:

```bash
rosnode ping /turtlesim
```
Expected output confirms communication latency.

7. Terminate a Node
To stop the turtlesim node:

```bash
rosnode kill /turtlesim
```
Verify with rosnode list (the node should disappear).
