#!/usr/bin/env python3
# coding=utf-8

import rospy
import sys
import os
import actionlib

# Add scripts directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '.'))

# Import Action Servers
from check_rules_action_server import CheckRulesActionServer
from find_object_action_server import FindObjectActionServer

# Import Messages for the Client
from second_coursework.msg import CheckRulesAction, CheckRulesGoal

if __name__ == '__main__':
    try:
        rospy.init_node('second_coursework_main')
        
        # 1. Start Servers FIRST
        # They must be running before we can send goals to them
        rospy.loginfo("Starting Action Servers...")
        check_rules_server = CheckRulesActionServer()
        find_object_server = FindObjectActionServer()
        
        # 2. Wait 10 seconds for localization (Required)
        rospy.loginfo("Waiting 10 seconds for localization to stabilize...")
        rospy.sleep(10.0)
        
        # 3. TRIGGER THE ROBOT (This is the missing part)
        rospy.loginfo("Startup complete. Sending goal to CheckRules...")
        
        # Create a client to talk to our own 'check_rules' server
        client = actionlib.SimpleActionClient('/check_rules', CheckRulesAction)
        
        # Wait for the server to be definitely ready
        rospy.loginfo("Waiting for check_rules server connection...")
        client.wait_for_server()
        
        # Send the goal! This is the "Start Button"
        goal = CheckRulesGoal()
        client.send_goal(goal)
        rospy.loginfo("GOAL SENT! Robot should now start moving between rooms.")
        
        # Keep the node alive
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass