#!/usr/bin/env python3
import rospy
import actionlib
from second_coursework.srv import FindObject, FindObjectResponse
from second_coursework.msg import FindObjectAction, FindObjectGoal

class FindObjectServiceNode:
    def __init__(self):
        rospy.init_node("find_object_service_node")

        self.action_client = actionlib.SimpleActionClient('/find_object_action', FindObjectAction)
        self.action_client.wait_for_server()
        rospy.loginfo("[FindObjectService] Action Server Connected - Ready for requests.")

        self.service = rospy.Service("/find_object", FindObject, self.handle_find_object)

    def handle_find_object(self, req):
        target = req.object_name
        rospy.loginfo(f"[FindObjectService] Received Request: Find '{target}'")
        state = self.action_client.get_state()
        
        if state in [0, 1]:
            rospy.logwarn(f"[FindObjectService] REJECTED. Already searching (State: {state}).")
            return FindObjectResponse(request_accepted=False)

        goal = FindObjectGoal()
        goal.object_name = target
        self.action_client.send_goal(goal)
        
        rospy.loginfo("[FindObjectService] ACCEPTED. Starting search.")
        return FindObjectResponse(request_accepted=True)
if __name__ == "__main__":
    try:
        node = FindObjectServiceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
