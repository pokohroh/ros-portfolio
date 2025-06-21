# ROS Tutorial: Combining Publisher and Subscriber

## Objective
To familiarise with how to combine both publisher and subscriber into one controller file.

## Procedure

1. Create a python file "turtle_controller.py" with executable rights

```bash
cd catkin_ws/src/my_robot_controller/scripts/
touch turtle_controller.py
chmod +x turtle_controller.py
```

2. Open VScode Editor
```bash
cd ../..
code .
```

3. Write a script that combines both publisher and subscriber in one file.
```python
#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

def pose_callback(pose: Pose):
  cmd = Twist()
  cmd.linear.x = 2.0 
  cmd.angular.z = 1.0
  pub.publisher(cmd) # move turtle in a circle

  rospy.loginfo("(" + str(pose.x) + ", " + str(pose.y) + ")") # shows the x and y coordinate of the turtle

if __name__ == '__main__':
  rospy.init_node("turtle_controller")
  pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
  sub = rospy.Subscriber("/turtle1/pose", Pose, callback=pose_callback)
  rospy.loginfo("Node has been started")

  rospy.spin()
```

7. Run the node
```bash
rosrun my_robot_controller turtle_controller.py
```

The turle will move in a circle, and the terminal will continuously update and display the x and y coordinate of the turtle.
