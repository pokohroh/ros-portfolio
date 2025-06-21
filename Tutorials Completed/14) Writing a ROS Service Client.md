# ROS Tutorial: Writing a ROS Service Client

## Objective
To familiarise with how to write a ROS service client with python.

By the end of this tutorial, you will write a service that turns turtle pen color red on the left half of the simulator, and green on the right half of the simulator as the turtle moves in circle.

## Procedure

1. Open src folder where it contains the python file "turtle_controller.py", which was written in tutorial 12.
```bash
cd catkin_ws/src/
```

2. Open VScode Editor
```bash
code .
```

3. Add a line under main to wait for service "turtle1/set_pen" to be up.
```python
rospy.wait_for_service("turtle1/set_pen")
```
4. Import SetPen service from turtlesim
```python
from turtlesim.srv import SetPen
```

5. Build a new function call_set_pen_service(), responsible to set pen service.
```python
def call_set_pen_service(r, g, b, width, off):
  try:
      set_pen = rospy.ServiceProxy("/turtle1/set_pen", SetPen)
      response = set_pen(r, g, b, width, off)
  except rospy.ServiceExeption as e:
      rospy.logwarn(e)
```   
6.Under the function pose_callback(pose: Pose), implement the call_set_pen_service() function when turtle's x coordinate reaches half of the terminal.

Complete code **turtle_controller.py**
```python
#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen

previous_x = 0 # initialise global variable to store previous x coodinate of the turtle in the simulator

def call_set_pen_service(r, g, b, width, off):
  try:
      set_pen = rospy.ServiceProxy("/turtle1/set_pen", SetPen)
      response = set_pen(r, g, b, width, off)
  except rospy.ServiceExeption as e:
      rospy.logwarn(e)

def pose_callback(pose: Pose):
  cmd = Twist()
  cmd.linear.x = 2.0 
  cmd.angular.z = 1.0
  pub.publisher(cmd) # move turtle in a circle

  rospy.loginfo("(" + str(pose.x) + ", " + str(pose.y) + ")") # shows the x and y coordinate of the turtle

  # Implementation added below
  global previous_x 
  if pose.x >= 5.5 and previous_x < 5.5:
     # 5.5 is x coordinate of the turtle in the half of the simulator space.
     # reason for comparing the previous x coordinate is to make sure call_set_pen_service function is only called when turtle reaches the half, and not every single time when pose_callback function is run
     call_set_pen_service(255, 0, 0, 3, 0)
  elif pose.x < 5.5 and previous_x >= 5.5:
     call_set_pen_service(0, 255, 0, 3, 0)
  previous_x = pose.x # set previous x coordinate to current x coordinate
  
  if __name__ == '__main__':
    rospy.init_node("turtle_controller")
    rospy.wait_for_service("turtle1/set_pen")
    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    sub = rospy.Subscriber("/turtle1/pose", Pose, callback=pose_callback)
    rospy.loginfo("Node has been started")

  rospy.spin()
```

7. Run the node
```bash
rosrun my_robot_controller turtle_controller.py
```

The turle will move in a circle, turtle pen stays red in right half, green in left half.
