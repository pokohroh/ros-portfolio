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
