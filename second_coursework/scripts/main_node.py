#!/usr/bin/env python3
# coding=utf-8

import rospy
import sys
import os
import actionlib

# Add scripts directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '.'))

from check_rules_action_server import CheckRulesActionServer
from find_object_action_server import FindObjectActionServer
from find_object_service import FindObjectService


if __name__ == '__main__':
    try:
        rospy.init_node('second_coursework_main')
        
        # Wait 10 seconds at startup for localization
        rospy.loginfo("Waiting 10 seconds for localization...")
        rospy.sleep(10.0)
        rospy.loginfo("Starting all nodes...")
        
        # Start all servers
        check_rules_server = CheckRulesActionServer()
        find_object_action_server = FindObjectActionServer()
        find_object_service = FindObjectService()
        
        rospy.loginfo("All nodes started successfully")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass

