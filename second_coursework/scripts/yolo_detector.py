#!/usr/bin/env python3
# coding=utf-8

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from yolov4 import Detector

from second_coursework.srv import YOLOFrame, YOLOFrameResponse
from second_coursework.msg import YoloDetection

class YoloCWNode:
    def __init__(self):
        rospy.init_node("yolo_cw_node")

        self.bridge = CvBridge()
        self.cv_image = None
        
        rospy.loginfo("[YOLO CW] Initialising YOLOv4 detector")

        self.detector = Detector(
            gpu_id=0,
            config_path="/opt/darknet/cfg/yolov4.cfg",
            weights_path="/opt/darknet/yolov4.weights",
            lib_darknet_path="/opt/darknet/libdarknet.so",
            meta_path=rospy.get_param("~coco_data", "/opt/darknet/cfg/coco.data")
        )

        self.image_sub = rospy.Subscriber("/camera/image", Image, self.img_callback, queue_size=1)

        self.detected_pub = rospy.Publisher("/detected_objects", String, queue_size=1)

        self.service = rospy.Service('/detect_frame', YOLOFrame, self.handle_detect_frame)

        rospy.loginfo("[YOLO CW] Ready. Detector runs only on Service calls.")

    def img_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as e:
            rospy.logwarn(f"[YOLO CW] cv_bridge error: {e}")

    def handle_detect_frame(self, req):
        resp = YOLOFrameResponse()

        if self.cv_image is None:
            rospy.logwarn("[YOLO CW] Service called but no image received yet!")
            return resp

        cv_height, cv_width = self.cv_image.shape[:2]

        img_net = cv2.resize(self.cv_image,(self.detector.network_width(), self.detector.network_height()))
        
        detections = self.detector.perform_detect(image_path_or_buf=img_net, show_image=False)

        names = [det.class_name.lower() for det in detections]
        msg_out = String()
        msg_out.data = ",".join(names)
        self.detected_pub.publish(msg_out)
        rospy.loginfo(f"[YOLO CW] Service Scan Found: {names}")

        for det in detections:
            d = YoloDetection()
            d.name = det.class_name
            d.confidence = det.class_confidence
            
            d.bbox_x = int((det.left_x / self.detector.network_width()) * cv_width)
            d.bbox_y = int((det.top_y / self.detector.network_height()) * cv_height)
            d.width = int((det.width / self.detector.network_width()) * cv_width)
            d.height = int((det.height / self.detector.network_height()) * cv_height)
            
            resp.detections.append(d)

        return resp

if __name__ == "__main__":
    try:
        node = YoloCWNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass