#!/usr/bin/env python3
import rospy
import actionlib
from second_coursework.msg import CheckRulesAction, CheckRulesGoal

rospy.init_node("test_check_rules_client")

client = actionlib.SimpleActionClient('/check_rules', CheckRulesAction)
rospy.loginfo("Waiting for /check_rules action server...")
client.wait_for_server()
rospy.loginfo("Connected! Sending empty goal...")

goal = CheckRulesGoal()  # empty goal
client.send_goal(goal)

rospy.loginfo("Goal sent. This should start your SMACH machine.")
client.wait_for_result()
rospy.loginfo("Action finished.")
