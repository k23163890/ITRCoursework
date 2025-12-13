#!/usr/bin/env python3
import rospy
import actionlib
from second_coursework.srv import FindObject, FindObjectResponse
from second_coursework.msg import FindObjectAction, FindObjectGoal

class FindObjectServiceNode:
    def __init__(self):
        rospy.init_node("find_object_service_node")

        # 1. Create a Client to talk to the Action Server (The "Muscle")
        self.action_client = actionlib.SimpleActionClient('/find_object_action', FindObjectAction)
        
        rospy.loginfo("[FindObjectService] Waiting for Action Server...")
        self.action_client.wait_for_server()
        rospy.loginfo("[FindObjectService] Action Server Connected! Ready for requests.")

        # 2. Advertise the Service (The "Interface")
        self.service = rospy.Service("/find_object", FindObject, self.handle_find_object)

    def handle_find_object(self, req):
        """
        This function runs when you type: rosservice call /find_object "dog"
        """
        target = req.object_name
        rospy.loginfo(f"[FindObjectService] Received Service Request: Find '{target}'")

        # 3. Create the Action Goal
        goal = FindObjectGoal()
        goal.object_name = target

        # 4. Send the goal to the Action Server
        # This triggers the robot to stop patrolling and start searching
        self.action_client.send_goal(goal)

        rospy.loginfo("[FindObjectService] Forwarded request to Action Server.")

        # 5. Return success immediately (Non-blocking)
        return FindObjectResponse(request_accepted=True)

if __name__ == "__main__":
    try:
        node = FindObjectServiceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass