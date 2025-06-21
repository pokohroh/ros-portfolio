# ROS Tutorial : Understanding ROS Topics

A **ROS topic** is a named bus over which nodes can send (publish) or receive (subscribe to) messages asynchronously.

Imagine a radio station, the station broadcasts on a frequency (like a publisher). Any radio tuned in to that frequency (a subscriber) hears the message.
Topics are like the frequencies.

## Objective
- To understand what a ROS Topic is as well s using ROS Computational grpahs such as rqt_plot and its commandline tools.

## Procedure

1. In a new terminal run `roscore`
   
```bash
$ roscore
```
> NOTE: Only one ROS master needs to be running, you will get an error message when you attempt to run another roscore node.

2. In a new terminal, run the turtlesim node

```bash
$ rosrun turtlesim turtlesim_node 
```
3. In a new terminal, launch the turtle keyboard teleoperation node 
```bash
$ rosrun turtlesim turtle_teleop_key
```
 By launching the teleoperation node, you can use arrowkeys of the keyboard to drive the turtle around. Now you have two nodes running, one is `turtlesim_node` and the other is `turtle_teleop_key` node. These nodes are communicating with each other over a ROS Topic.

* The publisher here is : `turtle_teleop_key` (Publishing key strokes on a topic)
* The subscriber here is : `turtlesim_node` (Subscribes to the same topic to recieve the key strokes)
  
We can use the `rqt_graph` to clearly see the nodes and topics currently running.

### Using rqt_graph
4. In a new terminal, enter the following command to launch rqt_graph:
```bash
$ rosrun rqt_graph rqt_graph
```
You should see something like this on your screen (place your mouse over the `turtle1/command_velocity` to highlight the nodes and topics):
![rqt_graph_turtle_key](https://github.com/user-attachments/assets/798dc19e-cf1e-4a69-83d1-b80d89f06c5b)

5. We can use the `rostopic` tool to get more information of ROS Topics. Use the help option for available sub-commands:
```bash
$ rostopic -h
```
6. Rostopic echo command shows the data publiched on a specific topic
```bash
$ rostopic /turtle1/command_velocity
```
The command above will display the data that is being traversed between the Publisher(`turtle_teleop_key`) and Subscriber(`turtlesim_node`)

**STILL UPDATING THIS IS NOT FINAL**








