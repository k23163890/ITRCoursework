#!/usr/bin/env python3
# coding=utf-8

"""
Simple test client for testing the coursework actions and services.
Run this after the main system is running.
"""

import rospy
import actionlib
from second_coursework.srv import FindObject, FindObjectRequest
from second_coursework.msg import CheckRulesAction, CheckRulesGoal
from second_coursework.msg import FindObjectAction, FindObjectGoal


def test_find_object_service():
    """Test the find_object service."""
    print("\n=== Testing FindObject Service ===")
    rospy.wait_for_service('/find_object')
    service = rospy.ServiceProxy('/find_object', FindObject)
    
    try:
        req = FindObjectRequest()
        req.object_name = 'bottle'
        resp = service(req)
        print(f"Service response: request_accepted = {resp.request_accepted}")
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")


def test_check_rules_action():
    """Test the check_rules action."""
    print("\n=== Testing CheckRules Action ===")
    client = actionlib.SimpleActionClient('/check_rules', CheckRulesAction)
    
    print("Waiting for server...")
    client.wait_for_server()
    print("Server available!")
    
    goal = CheckRulesGoal()
    print("Sending goal...")
    client.send_goal(goal)
    
    print("Waiting for result (will timeout after 30 seconds for testing)...")
    finished = client.wait_for_result(timeout=rospy.Duration(30))
    
    if finished:
        state = client.get_state()
        print(f"Action finished with state: {state}")
    else:
        print("Action timed out (this is expected if action keeps running)")
        print("Cancelling goal...")
        client.cancel_goal()


def test_find_object_action():
    """Test the find_object_action."""
    print("\n=== Testing FindObject Action ===")
    client = actionlib.SimpleActionClient('/find_object_action', FindObjectAction)
    
    print("Waiting for server...")
    client.wait_for_server()
    print("Server available!")
    
    goal = FindObjectGoal()
    goal.object_name = 'bottle'
    print(f"Sending goal to find: {goal.object_name}")
    client.send_goal(goal)
    
    print("Waiting for result...")
    finished = client.wait_for_result(timeout=rospy.Duration(60))
    
    if finished:
        state = client.get_state()
        result = client.get_result()
        print(f"Action finished with state: {state}")
        if result:
            print(f"Success: {result.success}")
            print(f"Room found: {result.room_found}")
            print(f"Object found: {result.object_found}")
    else:
        print("Action timed out")
        client.cancel_goal()


def main():
    rospy.init_node('test_client')
    
    print("=" * 50)
    print("Coursework Test Client")
    print("=" * 50)
    
    # Wait a bit for system to initialize
    rospy.sleep(2)
    
    # Test service
    test_find_object_service()
    rospy.sleep(2)
    
    # Test check_rules action (will run briefly then cancel)
    print("\nNote: check_rules will run continuously. Testing for 10 seconds...")
    test_check_rules_action()
    rospy.sleep(2)
    
    # Test find_object_action
    test_find_object_action()
    
    print("\n" + "=" * 50)
    print("Testing complete!")
    print("=" * 50)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

