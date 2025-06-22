# ROS Tutorial: Using rqt_console and roslaunch

## Objective
- Introduce using `rqt_console` and `rqt_logger_level` for debugging and `roslaunch` for starting many nodes as once.

## Procedure

1. The tutorial uses both the rqt and turtlesim packages. To do this tutorial, please install both packages:
   
```bash
$ sudo apt-get install ros-<distro>-rqt ros-<distro>-rqt-common-plugins ros-<distro>-turtlesim
```
2. Using rqt_console and rqt_logger_level
   
rqt_console lets you view messages and logs from ROS nodes, like errors or info messages. It connects to ROS's logging system. `rqt_logger_level` lets you change how much detail the nodes show in their messages. For example, you can choose to see only warnings or show detailed debug info.

Let's see what the turtlesim node is printing in rqt_console, and change its log level using `rqt_logger_level`. 


> Before running turtlesim, open two new terminals: in one, run rqt_console, and in the other, run rqt_logger_level.

```bash
$ rosrun rqt_console rqt_console
```
```bash
$ rosrun rqt_logger_level rqt_logger_level
```
You will see two windows popup:

![rqt_console(start)](https://github.com/user-attachments/assets/f4602c82-f5f5-4841-aaaf-00c9245045f7)

![rqt_logger_level](https://github.com/user-attachments/assets/dd3970b4-31ff-4cb6-8322-c073757d1be3)

3. Start turtlesim in a new terminal:

```bash
$ rosrun turtlesim turtlesim_node
```
By default, the logger level is set to **INFO**, so when you start turtlesim, you will see any information messages it sends out. These messages will look something like this:

![rqt_console(turtlesimstart)](https://github.com/user-attachments/assets/ede4f9f3-7256-4791-a22b-5d7db8fc2e7e)

Now change the logger level to **Warn** by refreshing the nodes in the **rqt_logger_level** window and selecting Warn as shown below:

![rqt_logger_level(error)](https://github.com/user-attachments/assets/6d02531f-d096-49b6-aeda-f6ce625ebe0f)

Now let's run our turtle into the wall and see what is displayed in our rqt_console:

```bash
$ rostopic pub /turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
```
4. Quick Note about logger levels
Logging levels are prioritized in the following order:
```
Fatal
Error
Warn
Info
Debug
```
**Fatal** is the most important message level, and **Debug** is the least. When you choose a certain logger level, you will see messages from that level and anything more serious. For example, if you set it to **Warn**, you’ll see **Warn**, **Error**, and **Fatal** messages — but not **Info** or **Debug**.

To stop the running turtlesim node, press Ctrl-C. After that, you can use roslaunch to start multiple turtlesim windows along with a special node that makes one turtle copy the movements of another.

5. Using roslaunch
roslaunch starts nodes as defined in a launch file. Before we use `roslaunch` command,  go to the beginner_tutorials package we created and built earlier:

```bash
$ roscd beginner_tutorials
```
If you see an error like roscd: No such package/stack `beginner_tutorials`, it means your system doesn't recognize the package yet. To fix this, you need to run the environment setup file, just like you did at the end of the `create_a_workspace` tutorial.
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
> NOTE: You don't have to name the folder for launch files launch, and you don't even need to put them in a separate folder. The roslaunch command will still find the launch files in the package. But naming the folder launch and organizing your files this way is considered good practice.


5. The Launch File
Create a launch file called `turtlemimic.launch` and paste the following:
```YAML
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

6. Roslaunching
Now let's roslaunch the launch file:

```bash
$ roslaunch beginner_tutorials turtlemimic.launch
```
Two turtlesims will start and in a new terminal send the rostopic command:

```bash
$ rostopic pub /turtlesim1/turtle1/cmd_vel geometry_msgs/Twist -r 1 -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, -1.8]'
```
You will see two turtlesims moving even though the publish command is only sent to turtlesim1 as seen here in the image below:

![mimic](https://github.com/user-attachments/assets/fde466c4-a3ee-42d3-8f26-c4cd13d28a4f)

We can also use rqt_graph to see what our launch file is doing behind the scenes. Open the main rqt window, then go to **Plugins > Introspection > Node Graph** to view the connections between nodes.

```bash
rqt
```

Or

```bash
rqt_graph
```

![mimiclaunch](https://github.com/user-attachments/assets/0f4649cf-d917-4d04-bc11-8b96297d8e26)

