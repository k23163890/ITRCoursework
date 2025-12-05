#!/usr/bin/env python3
# coding=utf-8

import rospy
from cv_bridge import CvBridge  
import cv2
import random

from sensor_msgs.msg import Image
from YOLOObjectDetection.msg import YoloDetection
from YOLOObjectDetection.srv import YOLOFrame, YOLOFrameResponse

from yolov4 import Detector


class YOLOv4ROSITR:
    def __init__(self):
        self.bridge = CvBridge()
        self.cv_image = None
        self.colors = {}

        self.detector = Detector(
            gpu_id=0,
            config_path="/opt/darknet/cfg/yolov4.cfg",
            weights_path="/opt/darknet/yolov4.weights",
            lib_darknet_path="/opt/darknet/libdarknet.so",
            meta_path=rospy.get_param(
                "~coco_data",
                "/home/k23163890/catkin_ws/src/YOLOObjectDetection/config/coco.data"
            )
        )

        rospy.Subscriber("/usb_cam/image_raw",
                         Image,
                         self.img_callback)

        self.yolo_srv = rospy.Service("/detect_frame",YOLOFrame,
            self.yolo_service
        )

        self.annotated_pub = rospy.Publisher("/yolo/annotated",Image,queue_size=1)

        rospy.loginfo("[YOLO] Ready. Waiting images...")

    def img_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def yolo_service(self, request):
        res = YOLOFrameResponse()

        if self.cv_image is None:
            rospy.logwarn("[YOLO] No image received yet.")
            return res

        img_net = cv2.resize(
            self.cv_image,
            (self.detector.network_width(), self.detector.network_height())
        )

        detections = self.detector.perform_detect(
            image_path_or_buf=img_net,
            show_image=False
        )

        cv_copy = self.cv_image.copy()
        img_h, img_w, _ = cv_copy.shape

        for det in detections:
            d = YoloDetection()
            d.name = det.class_name.lower()
            d.confidence = det.class_confidence

            # map bbox coords back to original image space
            d.bbox_x = int((det.left_x / self.detector.network_width()) * img_w)
            d.bbox_y = int((det.top_y  / self.detector.network_height()) * img_h)
            d.width  = int((det.width  / self.detector.network_width()) * img_w)
            d.height = int((det.height / self.detector.network_height()) * img_h)

            res.detections.append(d)

            # pick or store a color for this class
            if d.name not in self.colors:
                self.colors[d.name] = (
                    random.randint(0, 255),
                    random.randint(0, 255),
                    random.randint(0, 255)
                )
            color = self.colors[d.name]

            # draw box + label
            cv2.rectangle(
                cv_copy,
                (d.bbox_x, d.bbox_y),
                (d.bbox_x + d.width, d.bbox_y + d.height),
                color,
                2
            )
            cv2.putText(
                cv_copy,
                d.name,
                (d.bbox_x, d.bbox_y - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2
            )

        # publish annotated
        msg = self.bridge.cv2_to_imgmsg(cv_copy, encoding="bgr8")
        self.annotated_pub.publish(msg)

        return res
    # -----------------------------------------------------


if __name__ == '__main__':
    rospy.init_node("yolo_ros_itr")
    YOLOv4ROSITR()
    rospy.spin()
