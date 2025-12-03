#!/usr/bin/env python3
# YOLO ROS Detection Server
# Consumes camera feed, uses darknet, and answers YOLOFrame.srv requests

import sys
import os   # 👉 add this
sys.path.append('/opt/darknet')   # make darknet.py visible

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from YOLOObjectDetection.srv import YOLOFrame, YOLOFrameResponse
from YOLOObjectDetection.msg import YoloDetection

import darknet   # official python binding


class YOLOv4ROS:
    def __init__(self):
        rospy.loginfo("Initialising YOLOv4 ROS Server...")

        # 🔹 Ensure relative paths in coco.data work:
        #    coco.data has: names = data/coco.names
        #    but that's only valid if cwd == /opt/darknet
        os.chdir("/opt/darknet")

        # Paths for Darknet
        self.cfg_path     = "/opt/darknet/cfg/yolov4.cfg"
        self.weights_path = "/opt/darknet/yolov4.weights"
        self.data_path    = "/opt/darknet/cfg/coco.data"

        # Load network
        self.network, self.class_names, _ = darknet.load_network(
            config_file=self.cfg_path,
            data_file=self.data_path,
            weights=self.weights_path,
            batch_size=1
        )

        self.net_w = darknet.network_width(self.network)
        self.net_h = darknet.network_height(self.network)

        rospy.loginfo("YOLOv4 loaded successfully!")

        self.bridge = CvBridge()
        self.cv_img = None

        # Listen to camera
        rospy.Subscriber(
            "/usb_cam/image_raw",
            Image,
            self.image_cb,
            queue_size=1
        )

        # Provide detection service
        rospy.Service(
            "/detect_frame",
            YOLOFrame,
            self.detect_cb
        )

        rospy.loginfo("YOLO Frame Detection Service is READY!")

    # --------------------------------------------------------------------
    def image_cb(self, msg):
        """Receive latest camera image"""
        try:
            self.cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logwarn(f"Image conversion error: {e}")

    # --------------------------------------------------------------------
    def detect_cb(self, req):
        """Service handler: returns objects detected in the most recent frame"""
        response = YOLOFrameResponse()

        if self.cv_img is None:
            rospy.logwarn("No camera frame yet — sending empty detection list.")
            return response

        # Prepare image for YOLO
        resized = cv2.resize(self.cv_img, (self.net_w, self.net_h))
        rgb     = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)

        darknet_img = darknet.make_image(self.net_w, self.net_h, 3)
        darknet.copy_image_from_bytes(darknet_img, rgb.tobytes())

        detections = darknet.detect_image(
            self.network,
            self.class_names,
            darknet_img,
            thresh=0.25
        )

        for (cls, conf, bbox) in detections:
            x, y, w, h = bbox

            det = YoloDetection()
            det.name       = cls
            det.confidence = float(conf) / 100.0

            # Convert center → top-left int and clamp
            det.bbox_x = max(0, int(x - w/2))
            det.bbox_y = max(0, int(y - h/2))
            det.width  = max(0, int(w))
            det.height = max(0, int(h))

            response.detections.append(det)

        darknet.free_image(darknet_img)
        return response


if __name__ == "__main__":
    rospy.init_node("yolo_server")
    node = YOLOv4ROS()
    rospy.spin()