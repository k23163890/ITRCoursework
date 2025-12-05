#!/usr/bin/env python3
# coding=utf-8

import rospy
import actionlib
from second_coursework.srv import FindObject, FindObjectResponse
from second_coursework.msg import CheckRulesAction
from YOLOObjectDetection.srv import YOLOFrame


class FindObjectService:
    def __init__(self):
        # Don't init node here - main_node does it
        rospy.loginfo("Waiting for /detect_frame service...")
        try:
            rospy.wait_for_service("/detect_frame", timeout=rospy.Duration(10.0))
            self.yolo_client = rospy.ServiceProxy("/detect_frame", YOLOFrame)
            rospy.loginfo("Connected to YOLO detection service.")
        except rospy.ROSException:
            rospy.logerr("ERROR: /detect_frame service not available!")
            rospy.logerr("Make sure the YOLO detection node is running.")
            rospy.logerr("You may need to launch a YOLO node or check if it's included in your launch file.")
            raise
        
        # Client to stop check_rules action
        self.check_rules_client = None
        
        self.service = rospy.Service("/find_object", FindObject, self.find_object_callback)
        rospy.loginfo("FindObject service is ready.")
        
    def find_object_callback(self, request):
        """
        Service callback that checks if the requested object is detected in the current frame.
        Stops check_rules action when called.
        
        Args:
            request: FindObject request containing object_name (string)
            
        Returns:
            FindObjectResponse with request_accepted (bool)
        """
        response = FindObjectResponse()
        
        # Stop check_rules action if it's running
        try:
            if self.check_rules_client is None:
                self.check_rules_client = actionlib.SimpleActionClient('/check_rules', CheckRulesAction)
            
            if self.check_rules_client.wait_for_server(timeout=rospy.Duration(0.5)):
                if self.check_rules_client.get_state() == actionlib.GoalStatus.ACTIVE:
                    rospy.loginfo("Stopping check_rules action...")
                    self.check_rules_client.cancel_goal()
        except Exception as e:
            rospy.logdebug(f"Could not stop check_rules: {e}")
        
        try:
            # Get detections from YOLO service
            yolo_response = self.yolo_client()
            
            # Check if requested object is in detections
            requested_object = request.object_name.lower()
            
            for detection in yolo_response.detections:
                if detection.name.lower() == requested_object:
                    rospy.loginfo(f"Found object: {requested_object} (confidence: {detection.confidence:.2f})")
                    response.request_accepted = True
                    return response
            
            rospy.loginfo(f"Object '{requested_object}' not found in current frame.")
            response.request_accepted = False
            return response
            
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            response.request_accepted = False
            return response


if __name__ == '__main__':
    try:
        rospy.init_node('find_object_service_node')
        rospy.loginfo("Waiting 10 seconds for localization...")
        rospy.sleep(10.0)
        FindObjectService()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

