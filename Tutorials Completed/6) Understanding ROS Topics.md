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

7. We can use `rostopic list` command to return a list of all the topics that nodes are currently subscribed/published to.
   
```bash
$ rostopic -list -h
```
## ROS Messages
ROS Messages are very important to topics, as these messages are sent during coommunication between nodes. For example, the publisher node (turtle_teleop_key) and subscriber node (turtlesim_node) can only communiate if they use the same kind of message. SO, the "type" of a topic depends on what kind of message it uses. You can check what message type a topic uses by running the `rostopic type` command. 

Example Usage of `rostopic type`:
```bash
$ rostopic type /turtle1/cmd_vel
```
For output you should get : `geometry_msgs/Twist`
To understand and see in detail what kind of message is `geometry_msgs/Twist`, we can use the following command:

```bash
$ rostopic show geometry_msgs/Twist
```
## Publishing ROS Messages on topics
To manually publish messages into a ROS topic, we use the `rostopic pub` command, to publish data that is currently advertised.

The command below will publish messages to the given topic `/turtle1/cmd_vel`. The option `(-1)` here causes rostopic to publish only one message and exit. it is followed by the message type `geometry_msgs/twist`, specifiying the message type to use when publishing to the topic. The double dash `(--)` option tells the option parser that none of the folliwng arguments is an option. 

```YAML
$ rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
```
A geometry_msgs/Twist message has two parts: linear and angular. Each part is made up of three numbers (for x, y, and z). In this example, `[2.0, 0.0, 0.0]` means the robot will move forward with a speed of 2.0 in the x direction, and no movement in y or z. The `[0.0, 0.0, 1.8]` means the robot will rotate with a speed of 1.8 around the z-axis. These values are written using YAML format.

This is the ouptut you should see in your screen:

![turtle(rostopicpub)](https://github.com/user-attachments/assets/8edc3ce8-ab37-4e14-b198-11abb420e088)

You might notice that the turtle stops moving after a while. That is because it needs to keep receiving movement commands regularly, at least once every second (1 Hz) to keep going. The previous command only sends the message once. We can send these continuous commands using the rostopic pub -r command, which repeatedly sends messages at a set rate.

```YAML
$ rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'
```
This is the output you should see in your screen:

![turtle(rostopicpub)2](https://github.com/user-attachments/assets/ed68220e-874a-4dc4-8825-55e4288e4317)

We can also see how the nodes are connected using rqt_graph. Just click the refresh button at the top left. You will see that the rostopic pub node (shown in red) is sending messages to the rostopic echo node (shown in green), showing how they are communicating.

![rqt_graph_pub](https://github.com/user-attachments/assets/23db824e-6f21-4d63-9587-3a1ac6810669)


## Rostopic rate
We can use `rostopic hz` command to give us the rate that data is published. For example, we can see how fast the `turtlesim_node` is publishing data to `/turtle1/pose` topic. 

```bash
$ rostopic hz /turtle1/pose
```
Output will show that turtlesim is publishing data about our turtle at the rate of 60Hz. 
