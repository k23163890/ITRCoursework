#!/usr/bin/env python3

#if robot is already searching reject the requests  - if not then handle the request

import rospy
from second_coursework.srv import FindObject, FindObjectResponse

is_searching = False
pending_object_request = None 


def handle_find_object(req):
    global is_searching, pending_object_request

    rospy.loginfo(f"[FindObjectService] Received request for: {req.object_name}")

    if is_searching:
        rospy.loginfo("[FindObjectService] Already searching → return False")
        return FindObjectResponse(request_accepted=False)

    pending_object_request = req.object_name
    is_searching = True

    rospy.loginfo(f"[FindObjectService] New object request accepted: {pending_object_request}")
    return FindObjectResponse(request_accepted=True)


def start_service():
    rospy.init_node("find_object_service_node")

    rospy.Service("/find_object", FindObject, handle_find_object)
    rospy.loginfo("[FindObjectService] /find_object service ready.")

    rospy.spin()


if __name__ == "__main__":
    start_service()
