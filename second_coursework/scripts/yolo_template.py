#!/usr/bin/env python3
# coding=utf-8
import rospy
from cv_bridge import CvBridge  
import cv2
import numpy as np
from yolov4 import Detector
import random
# TODO add here missing imports from ros messages such as Image, YOLODetection, etc.



class YOLOv4ROSITR:
    def __init__(self):
        self.bridge = CvBridge()
        self.cv_image = None
        self.colors = {}

        # TODO ADD HERE YOUR CODE TO INITIALIZE THE DETECTOR, SUBSCRIBERS AND SERVICES


    def img_callback(self, msg):
        pass
        # TODO ADD HERE YOUR CODE

    def yolo_service(self, request):
        res = YOLOLastFrameResponse()
        if self.cv_image is not None:
            cv_copy = self.cv_image.copy()
            # TODO COMPLETE HERE CODE TO PERFORM DETECTIONS
            cv_height, cv_width, _ = self.cv_image.shape
            for detection in detections:
                box = detection.left_x, detection.top_y, detection.width, detection.height
                print(f'{detection.class_name.ljust(10)} | {detection.class_confidence * 100:.1f} % | {box}')
                d = YOLODetection(detection.class_name, detection.class_confidence, detection.left_x, detection.top_y,
                                  detection.width, detection.height)
                # convert bbox to image space
                d.bbox_x = int((d.bbox_x/self.detector.network_width())*cv_width)
                d.bbox_y = int((d.bbox_y/self.detector.network_height())*cv_height)
                d.width = int((d.width/self.detector.network_width())*cv_width)
                d.height = int((d.height/self.detector.network_height())*cv_height)
                res.detections.append(d)

                # This paints the bounding boxes in the images with the name in it.
                if d.name in self.colors:
                    color = self.colors[d.name]
                else:
                    color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
                    self.colors[d.name] = color
                cv2.rectangle(cv_copy, (d.bbox_x, d.bbox_y), (d.bbox_x+d.width, d.bbox_y+d.height), color, 2)
                cv2.rectangle(cv_copy, (d.bbox_x, d.bbox_y), (d.bbox_x+5+(23*len(d.name)), d.bbox_y+30), color, -1)
                cv2.putText(cv_copy, d.name, (d.bbox_x+2, d.bbox_y+25), cv2.FONT_HERSHEY_PLAIN, 2.5, (0,0,0))
                # TODO: publish the inpainted image
                # Helper: to convert from opencv image to a image message you can do : message = self.bridge.cv2_to_imgmsg(cv_copy, encoding="bgr8"))
            else:
                # TODO report error
                pass
        return res


if __name__ == '__main__':
    rospy.init_node('yolo_ros_itr')
    yolo_ros = YOLOv4ROSITR()
    rospy.spin()
