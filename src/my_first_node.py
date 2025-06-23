#!/usr/bin/env python3
import rospy

if __name__ == '__main__':
  rospy.init_node("test_node")
  rospy.loginfo("Hello from test node") 
  rospy.logwarn("This is a warning")
  rospy.logerr("This is an error")

  rate = rospy.Rate(10) # rate is in hertz
  while not rospy.is_shutdown(): # will loop until the node is terminated
    rospy.loginfo("Hello")
    rate.sleep()
