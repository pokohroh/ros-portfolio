# ROS Tutorial: Writing a ROS Publisher 

## Objective
To familiarise with how to write your own ROS publisher with python.

## Procedure

1. Create a python file "draw_circle.py" with executable rights

```bash
cd catkin_ws/src/my_robot_controller/scripts/
touch draw_circle.py
chmod +x draw_circle.py
```

2. Open VScode Editor
```bash
cd ../..
code .
```
3. In my_first_node.py, initialise the node with name "draw_circle"

```python
#!/usr/bin/env python3
import rospy

if __name__ == '__main__':
  rospy.init_node("draw_circle")
```

4. Setup a publisher
```python
  pub = rospy.Publisher("exact_name_of_the_topic, data_class, queue_size=0)
```
Queue size is like a buffer that prevents overload of messages.

To find the exact name of the topica we are going to use,
```bash
roscore
rosrun turtlesim turtlesim_node
rostopic list
```

These topics will be listed
```bash
/rosout
/rosout_agg
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

We will be using the topic "/turtle1/cmd_vel".

To find the data class of the topic, 
```bash
rostopic info /turtle1/cmd_vel
```

Type of geometry_msgs/Twist will be shown.

![Image](https://github.com/user-attachments/assets/50ae1e50-4f4f-4832-a70e-0b34f0ff957c)

Edit the code to add Twist library and setup a publisher
```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__':
  rospy.init_node("draw_circle")
  pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10) 
```


5. Add dependency in the package.xml, to link a new package geometry_msgs.
```xml
<build_depend>geometry_msgs</build_depend>
<build_export_depend>geometry_msgs</build_export_depend>
<exec_depend>geometry_msgs</exec_depend>
```

6. To check what's inside the topic /turtle1/cmd_vel,
```bash
rosmsg show geometry_msgs/Twist
```
![Image](https://github.com/user-attachments/assets/a98356e9-c37d-4f70-8e0c-f519d4615a20)

7. Add these lines of code to move the turtle in a circle continuously.

```python
rate = rospy.Rate(2) # rate set to 2Hz
while not rospy.is_shutdown():
  msg = Twist()
  msg.linear.x = 2.0 
  msg.angular.z = 1.0
  pub.publisher(msg)
  rate.sleep()
```

Final code should look as shown below.
```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__':
  rospy.init_node("draw_circle")
  pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

  rate = rospy.Rate(2) # rate set to 2Hz
  while not rospy.is_shutdown():
    msg = Twist()
    msg.linear.x = 2.0
    msg.angular.z = 1.0
    pub.publisher(msg)
    rate.sleep()
```

7. Run the node
```bash
rosrun my_robot_controller draw_circle.py
```

![Image](https://github.com/user-attachments/assets/488cb57f-2f94-4c40-9d6f-156f90914479)
