#!/usr/bin/env python
import roslib
roslib.load_manifest('turtlebot_actions')

import rospy

import os
import sys
import time

from turtlebot_actions.msg import *
from actionlib_msgs.msg import *


import actionlib


def main():
  rospy.init_node("find_fiducial_pose_test")

  # Construct action ac
  rospy.loginfo("Starting action client...")
  action_client = actionlib.SimpleActionClient('find_fiducial_pose', FindFiducialAction)
  action_client.wait_for_server()
  rospy.loginfo("Action client connected to action server.")

  # Call the action
  rospy.loginfo("Calling the action server...")
  action_goal = FindFiducialGoal()
  action_goal.camera_name = "/camera/rgb"
  action_goal.pattern_width = 7
  action_goal.pattern_height = 6
  action_goal.pattern_size = 0.027
  action_goal.pattern_type = 0

  if action_client.send_goal_and_wait(action_goal, rospy.Duration(50.0), rospy.Duration(50.0)) == GoalStatus.SUCCEEDED:
    rospy.loginfo('Call to action server succeeded')
  else:
    rospy.logerr('Call to action server failed')


if __name__ == "__main__":
  main()
