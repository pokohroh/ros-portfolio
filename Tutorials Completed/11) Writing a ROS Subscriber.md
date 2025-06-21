# ROS Tutorial: Writing a ROS Subscriber 

## Objective
To familiarise with how to write your own ROS subscriber with python.

## Procedure

1. Create a python file "pose_subscriber.py" with executable rights

```bash
cd catkin_ws/src/my_robot_controller/scripts/
touch pose_subscriber.py
chmod +x pose_subscriber.py
```

2. Open VScode Editor
```bash
cd ../..
code .
```
3. In pose_subscriber.py, initialise the node with name "turtle_pose_subscriber"

```python
#!/usr/bin/env python3
import rospy

if __name__ == '__main__':
  rospy.init_node("turtle_pose_subscriber")
```

4. Setup a subscriber
```python
  sub = rospy.Subscriber("exact_name_of_the_topic, data_class, callback=name_of_function)
```
Callback calls a function that we will define to process the message received from the topic

To find the exact name of the topica we are going to use,
```bash
roscore
rosrun turtlesim turtlesim_node
rostopic list
```

These topics will be listed

/rosout<br/>
/rosout_agg<br/>
/turtle1/cmd_vel<br/>
/turtle1/color_sensor<br/>
/turtle1/pose<br/>

We will be using the topic "/turtle1/pose".

To find the data class of the topic, 
```bash
rostopic info /turtle1/pose
```

Type of turtlesim/Pose will be shown.

![Image](https://github.com/user-attachments/assets/3fdeba4b-8cab-4afb-bc22-b2014041b6ca)

Edit the code to add Pose library and setup a subscriber
```python
#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose

if __name__ == '__main__':
  rospy.init_node("turtle_pose_subscriber")
  sub = rospy.Subscriber("/turtle1/pose", Pose, callback=pose_callback)
  rospy.loginfo("Node has been started")

```

5. Add the line **rospy.spin()**, which will block until ROS node is shutdown

```python
rospy.spin()
```

6. Define the function "pose_callback"

```python
def pose_callback(msg: Pose):
  rospy.loginfo("(" + str(msg.x) + ", " + str(msg.y) + ")")
```

Final code should look as shown below.
```python
#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose

def pose_callback(msg: Pose):
  rospy.loginfo("(" + str(msg.x) + ", " + str(msg.y) + ")")

if __name__ == '__main__':
  rospy.init_node("turtle_pose_subscriber")
  sub = rospy.Subscriber("/turtle1/pose", Pose, callback=pose_callback)
  rospy.loginfo("Node has been started")

  rospy.spin()
```

7. Run the node
```bash
rosrun my_robot_controller pose_subscriber.py
```
When you move the turtle around, the X and Y coordinate will be updated and shown in the terminal.
