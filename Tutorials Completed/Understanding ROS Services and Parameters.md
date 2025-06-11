# ROS Tutorial: Understanding ROS Services and Parameters
*This tutorial assumes turtlesim node is running in the background

# Hardware Required
- None

# Software Required
- Ubuntu 18.04
- ROS Melodic
- A properly initialized `catkin` workspace


# ROS Services
ROS Services provide a synchronous client-server communication model. Unlike topics, services allow a node to send a **request** and receive a **response**.


Using rosservice
rosservice can easily attach to ROS's client/service framework with services. rosservice has many commands that can be used on services, as shown below:

- Usage:
```
rosservice list         print information about active services
rosservice call         call the service with the provided args
rosservice type         print service type
rosservice find         find services by service type
rosservice uri          print service ROSRPC uri
```
# Utilizing Rosservice List

`$ rosservice list`

The list command shows us that the turtlesim node provides nine services: reset, clear, spawn, kill, turtle1/set_pen, /turtle1/teleport_absolute, /turtle1/teleport_relative, turtlesim/get_loggers, and turtlesim/set_logger_level. 
There are also two services related to the separate rosout node: /rosout/get_loggers and /rosout/set_logger_level.

# Utilizing Rosservice Type
`rosservice type`

- Usage:

`rosservice type [service]`

# What is the Clear service

`$ rosservice type /clear`

This service call does not take any arguments, it is empty, thus it sends no data when making a request and receives no data when recieving a response)

# Utilizing Rosservice Call

Usage:

`rosservice call [service] [args]`

Here we'll call with no arguments because the service is of type empty:

`rosservice call /clear`

This does what we expect, it clears the background of the turtlesim_node.

# Service spawn
```
$ rosservice type /spawn | rossrv show
float32 x
float32 y
float32 theta
string name
---
string name
```
This service lets us spawn a new turtle at a given location and orientation. The name field is optional, so let's not give our new turtle a name and let turtlesim create one for us.

`rosservice call /spawn 2 2 0.2 ""`

The service call returns with the name of the newly created turtle

`name: turtle2`

Now our turtlesim should look like this:


![turtle(service)](https://github.com/user-attachments/assets/ae85675e-871c-42a7-83c5-6c49359ccbf9)


# Using rosparam

rosparam lets you save and manage data on the ROS Parameter Server. This server can store different types of values like numbers (integers and decimals), true/false (booleans), text (strings), lists, and key-value pairs (dictionaries). The data is written in YAML format, which is easy to read.

For example:

- 1 is an integer
- 1.0 is a decimal (float)
- "one" is a string
- true is a boolean
- [1, 2, 3] is a list of numbers
- {a: b, c: d} is a dictionary

You can use `rosparam` with several commands to work with these parameters, as shown below:
```
Usage:

rosparam set            set parameter
rosparam get            get parameter
rosparam load           load parameters from file
rosparam dump           dump parameters to file
rosparam delete         delete parameter
rosparam list           list parameter names

```

# What parameters are currently on the param server

`rosparam list`

Here we can see that the turtlesim node has three parameters on the param server for background color:
```
/rosdistro
/roslaunch/uris/host_nxt__43407
/rosversion
/run_id
/turtlesim/background_b
/turtlesim/background_g
/turtlesim/background_r
```

# Changing one of the parameter values using rosparam set:

`rosparam set` and `rosparam get`

- Usage:

```
rosparam set [param_name]
rosparam get [param_name]
```

*Here will change the red channel of the background color:*

` rosparam set /turtlesim/background_r 150`
This changes the parameter value, now we have to call the clear service for the parameter change to take effect:

`rosservice call /clear`

Now our turtlesim looks like this:

![turtle(param)](https://github.com/user-attachments/assets/3b018bb9-5c22-4b4b-b242-30c68dcdf685)


# Values of parameters on the param server. Let's get the value of the green background channel:

```
rosparam get /turtlesim/background_g 
86
```
We can also use `rosparam get /` to show us the contents of the entire Parameter Server.

```
rosparam get /
rosdistro: 'noetic

  '
roslaunch:
  uris:
    host_nxt__43407: http://nxt:43407/
rosversion: '1.15.5

  '
run_id: 7ef687d8-9ab7-11ea-b692-fcaa1494dbf9
turtlesim:
  background_b: 255
  background_g: 86
  background_r: 69
```
We can store the file so that we can reload it at another time. We can use rosparam command:
`rosparam dump and rosparam load`

- Usage:

```
rosparam dump [file_name] [namespace]
rosparam load [file_name] [namespace]
```

Here we write all the parameters to the file params.yaml

`rosparam dump params.yaml`

We can load these yaml files into new namespaces, e.g. `copy_turtle`:

```
rosparam load params.yaml copy_turtle
rosparam get /copy_turtle/turtlesim/background_b
255
```
