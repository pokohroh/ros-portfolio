# ROS Tutorial: Understanding Services and Parameters

# Objective
- Introduce using `rqt_console` and `rqt_logger_level` for debugging and `roslaunch` for starting many nodes as once.

# Hardware Required
- None

# Software Required
- Ubuntu 18.04
- ROS Melodic
- Terminal
- Properly built and sourced catkin workspace

# 1. Prerequisites rqt and turtlesim package
The tutorial uses both the rqt and turtlesim packages. To do this tutorial, please install both packages:

`sudo apt-get install ros-<distro>-rqt ros-<distro>-rqt-common-plugins ros-<distro>-turtlesim`

Replace <distro> with the name of your ROS distribution.

# 2. Using rqt_console and rqt_logger_level
rqt_console attaches to ROS's logging framework to display output from nodes. rqt_logger_level allows us to change the verbosity level (DEBUG, WARN, INFO, and ERROR) of nodes as they run.

Now let's look at the turtlesim output in rqt_console and switch logger levels in rqt_logger_level as we use turtlesim. Before we start the turtlesim, in two new terminals start rqt_console and rqt_logger_level:

`rosrun rqt_console rqt_console`

`rosrun rqt_logger_level rqt_logger_level`

You will see two windows popup:

![rqt_console(start)](https://github.com/user-attachments/assets/f4602c82-f5f5-4841-aaaf-00c9245045f7)

![rqt_logger_level](https://github.com/user-attachments/assets/dd3970b4-31ff-4cb6-8322-c073757d1be3)

Now let's start turtlesim in a new terminal:

`rosrun turtlesim turtlesim_node`

Since the default logger level is INFO you will see any info that the turtlesim publishes when it starts up, which should look like:

![rqt_console(turtlesimstart)](https://github.com/user-attachments/assets/ede4f9f3-7256-4791-a22b-5d7db8fc2e7e)

Now let's change the logger level to Warn by refreshing the nodes in the **rqt_logger_level** window and selecting Warn as shown below:


![rqt_logger_level(error)](https://github.com/user-attachments/assets/6d02531f-d096-49b6-aeda-f6ce625ebe0f)

Now let's run our turtle into the wall and see what is displayed in our rqt_console:

For ROS Hydro and later,

`rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'`


# 3. Quick Note about logger levels
Logging levels are prioritized in the following order:
```
Fatal
Error
Warn
Info
Debug
```
Fatal has the highest priority and Debug has the lowest. By setting the logger level, you will get all messages of that priority level or higher. For example, by setting the level to Warn, you will get all **Warn, Error, and Fatal** logging messages.

Ctrl-C the turtlesim and use roslaunch to bring up multiple turtlesim nodes and a mimicking node to cause one turtlesim to mimic another:

# 4. Using roslaunch
roslaunch starts nodes as defined in a launch file.

- Usage:
  
`roslaunch [package] [filename.launch]`

First go to the beginner_tutorials package we created and built earlier:

`roscd beginner_tutorials`

If roscd says something similar to roscd: No such package/stack 'beginner_tutorials' , you will need to source the environment setup file like you did at the end of the create_a_workspace tutorial:
```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roscd beginner_tutorials
```

Next make a launch directory:
```
$ mkdir launch
$ cd launch
```
**NOTE:** The directory to store launch files doesn't necessarily have to be named launch. In fact you don't even need to store them in a directory. roslaunch command automatically looks into the passed package and detects available launch files. However, this is considered good practice.

# 5. The Launch File
Now let's create a launch file called `turtlemimic.launch` and paste the following:

```
<launch>

  <group ns="turtlesim1">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <group ns="turtlesim2">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <node pkg="turtlesim" name="mimic" type="mimic">
    <remap from="input" to="turtlesim1/turtle1"/>
    <remap from="output" to="turtlesim2/turtle1"/>
  </node>

</launch>
```
# 6. The Launch File Explained
Now, let's break the launch xml down.

Toggle line numbers
 `   <launch>`
 
Here we start the launch file with the launch tag, so that the file is identified as a launch file.

```
 <group ns="turtlesim1">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <group ns="turtlesim2">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>
```

Here we start two groups with a namespace tag of turtlesim1 and turtlesim2 with a turtlesim node with a name of sim. This allows us to start two simulators without having name conflicts.

```
  <node pkg="turtlesim" name="mimic" type="mimic">
    <remap from="input" to="turtlesim1/turtle1"/>
    <remap from="output" to="turtlesim2/turtle1"/>
  </node>
```

Here we start the mimic node with the topics input and output renamed to turtlesim1 and turtlesim2. This renaming will cause turtlesim2 to mimic turtlesim1.

`</launch>`

This closes the xml tag for the launch file.


# 7. Roslaunching
Now let's roslaunch the launch file:

`roslaunch beginner_tutorials turtlemimic.launch`

Two turtlesims will start and in a new terminal send the rostopic command:

For ROS Hydro and later,

`rostopic pub /turtlesim1/turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'`

You will see two turtlesims moving though the publish command that is only sent to turtlesim1:

![mimic](https://github.com/user-attachments/assets/fde466c4-a3ee-42d3-8f26-c4cd13d28a4f)

We can also use `rqt_graph` to better understand what our launch file did. Run `rqt's` main window and select **Plugins > Introspection > Node Graph**:

`rqt`

Or simply:

`rqt_graph`

![mimiclaunch](https://github.com/user-attachments/assets/0f4649cf-d917-4d04-bc11-8b96297d8e26)

