#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import actionlib
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

class CombinedNavigator:
    def __init__(self):
        rospy.init_node('combined_navigator')

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")

        # Plot paths: placeholders for real coordinates
        self.plot_paths = {
            "Plot1": [{"x": "1.15", "y": "1.09", "yaw": "0.0"}],
            "Plot2": [{"x": "1.1", "y": "-0.189", "yaw": "0.0"}],
            "Plot3": [{"x": "0.995", "y": "-1.6", "yaw": "0.0"}],
            "Plot4": [{"x": "-0.0393", "y": "-0.922", "yaw": "0.0"}],
	    "Plot5": [{"x": "-1.87", "y": "-1.47", "yaw": "0.0"}],
            "Plot6": [{"x": "-1.53", "y": "0.0547", "yaw": "0.0"}],
            "Plot7": [{"x": "-1.56", "y": "1.25", "yaw": "0.0"}],
            "Plot8": [{"x": "-0.241", "y": "1.31", "yaw": "0.0"}],
            "Plot0": [{"x": "0.0", "y": "0.0", "yaw": "0.0"}]
        }

        # Home plot is static
        self.home_plot = "Plot0"
        self.current_location = self.home_plot

        self.run_menu()

    def run_menu(self):
        while not rospy.is_shutdown():
            print("\nWhere should LIMO go?")
            for key in sorted(self.plot_paths.keys()):
                print("  {} - {}".format(key[-1], key))  # Display as: 1 - Plot1
            print("  0 - Return Home (Plot0)")
            selection = raw_input("Enter your choice [0-9]: ")

            if selection == "0":
                self.navigate_to(self.current_location, self.home_plot)
                self.current_location = self.home_plot
            elif selection in [str(i) for i in range(1, 10) if i != 5]:
                plot_key = "Plot{}".format(selection)
                self.navigate_to(self.current_location, plot_key)
                self.current_location = plot_key
            else:
                print("Invalid selection. Try again.")

    def navigate_to(self, from_plot, to_plot):
        rospy.loginfo("Navigating from {} to {}".format(from_plot, to_plot))

        # Get path to target plot
        if from_plot == self.home_plot:
            path = self.plot_paths[to_plot]
        elif to_plot == self.home_plot:
            path = list(reversed(self.plot_paths[from_plot]))
        else:
            rospy.logwarn("Unsupported transition: {} → {}".format(from_plot, to_plot))
            return

        for i, wp in enumerate(path):
            rospy.loginfo("Sending waypoint {}/{}: x={}, y={}, yaw={}".format(i+1, len(path), wp["x"], wp["y"], wp["yaw"]))
            goal = self.create_goal(wp["x"], wp["y"], wp["yaw"])
            self.client.send_goal(goal)
            self.client.wait_for_result()
            state = self.client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Waypoint {} reached successfully.".format(i+1))
            else:
                rospy.logwarn("Waypoint {} failed. Attempting recovery...".format(i+1))
                self.clear_costmaps()
                self.client.send_goal(goal)
                self.client.wait_for_result()
                state = self.client.get_state()
                if state == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo("Waypoint {} reached on retry.".format(i+1))
                else:
                    rospy.logerr("Waypoint {} still failed. Skipping.".format(i+1))

    def create_goal(self, x, y, yaw):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # Replace placeholder strings with actual float if needed
        goal.target_pose.pose.position.x = float(x) if isinstance(x, str) else x
        goal.target_pose.pose.position.y = float(y) if isinstance(y, str) else y

        q = quaternion_from_euler(0, 0, float(yaw) if isinstance(yaw, str) else yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        return goal

    def clear_costmaps(self):
        try:
            rospy.wait_for_service('/move_base/clear_costmaps', timeout=2)
            clear = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
            clear()
            rospy.loginfo("Costmaps cleared.")
        except rospy.ServiceException as e:
            rospy.logwarn("Failed to clear costmaps: %s" % e)

if __name__ == '__main__':
    try:
        CombinedNavigator()
    except rospy.ROSInterruptException:
        pass
