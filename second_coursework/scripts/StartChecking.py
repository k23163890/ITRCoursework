#!/usr/bin/env python3
import rospy
import actionlib
from second_coursework.msg import CheckRulesAction, CheckRulesGoal

def trigger_check_rules():
    rospy.init_node("check_rules_client_node")
    
    # Create the client
    client = actionlib.SimpleActionClient('/check_rules', CheckRulesAction)
    
    rospy.loginfo("Waiting for /check_rules action server...")
    # Wait for the server to start (timeout 5s)
    if client.wait_for_server(timeout=rospy.Duration(20.0)):
        # Create and send the goal
        goal = CheckRulesGoal()
        client.send_goal(goal)
        rospy.loginfo("CheckRules Action automatically triggered!")
        
    else:
        rospy.logwarn("CheckRules server not found. Is it running?")

if __name__ == '__main__':
    try:
        trigger_check_rules()
    except rospy.ROSInterruptException:
        pass