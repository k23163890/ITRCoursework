#!/usr/bin/env python3
# coding=utf-8

import rospy
from second_coursework.srv import FindObject, FindObjectResponse
from YOLOObjectDetection.srv import YOLOFrame


class FindObjectService:
    def __init__(self):
        rospy.loginfo("Waiting for /detect_frame service...")
        rospy.wait_for_service("/detect_frame")
        self.yolo_client = rospy.ServiceProxy("/detect_frame", YOLOFrame)
        rospy.loginfo("Connected to YOLO detection service.")
        
        self.service = rospy.Service("/find_object", FindObject, self.find_object_callback)
        rospy.loginfo("FindObject service is ready.")
        
    def find_object_callback(self, request):
        """
        Service callback that checks if the requested object is detected in the current frame.
        
        Args:
            request: FindObject request containing object_name (string)
            
        Returns:
            FindObjectResponse with request_accepted (bool)
        """
        response = FindObjectResponse()
        
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
    rospy.init_node('find_object_service_node')
    FindObjectService()
    rospy.spin()

