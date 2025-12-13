#!/usr/bin/env python3
# coding=utf-8

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from yolov4 import Detector

# Import the custom service and message
from second_coursework.srv import YOLOFrame, YOLOFrameResponse
from second_coursework.msg import YoloDetection

class YoloCWNode:
    def __init__(self):
        rospy.init_node("yolo_cw_node")

        self.bridge = CvBridge()
        self.cv_image = None

        rospy.loginfo("[YOLO CW] Initialising YOLOv4 detector...")

        self.detector = Detector(
            gpu_id=0,
            config_path="/opt/darknet/cfg/yolov4.cfg",
            weights_path="/opt/darknet/yolov4.weights",
            lib_darknet_path="/opt/darknet/libdarknet.so",
            meta_path=rospy.get_param("~coco_data", "/opt/darknet/cfg/coco.data")
        )

        # 1. Subscribe to camera (Keeps the topic logic working)
        self.image_sub = rospy.Subscriber(
            "/camera/image", Image, self.img_callback, queue_size=1
        )

        # 2. Publish string results (Keeps your logs working)
        self.detected_pub = rospy.Publisher(
            "/detected_objects", String, queue_size=1
        )

        # 3. NEW: Advertise the Service '/detect_frame'
        self.service = rospy.Service('/detect_frame', YOLOFrame, self.handle_detect_frame)

        rospy.loginfo("[YOLO CW] Ready. Service /detect_frame is available.")

    def img_callback(self, msg):
        """Standard callback to update image and publish to topic."""
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn(f"[YOLO CW] cv_bridge error: {e}")
            return

        # Resize and detect for the topic/log output
        img_net = cv2.resize(
            self.cv_image,
            (self.detector.network_width(), self.detector.network_height())
        )
        detections = self.detector.perform_detect(image_path_or_buf=img_net, show_image=False)

        names = [det.class_name.lower() for det in detections]
        msg_out = String()
        msg_out.data = ",".join(names)
        self.detected_pub.publish(msg_out)
        
        rospy.loginfo(f"[YOLO CW] Detected objects: {names}")

    def handle_detect_frame(self, req):
        """Service callback: Returns detections to the State Machine."""
        resp = YOLOFrameResponse()

        if self.cv_image is None:
            rospy.logwarn("[YOLO CW] Service called but no image received yet!")
            return resp

        # Perform detection on the stored image for the service response
        img_net = cv2.resize(
            self.cv_image,
            (self.detector.network_width(), self.detector.network_height())
        )
        detections = self.detector.perform_detect(image_path_or_buf=img_net, show_image=False)

        # Convert to YoloDetection Message format
        for det in detections:
            d = YoloDetection()
            d.name = det.class_name
            d.confidence = det.class_confidence
            d.bbox_x = int(det.left_x)
            d.bbox_y = int(det.top_y)
            d.width = int(det.width)
            d.height = int(det.height)
            resp.detections.append(d)

        return resp

if __name__ == "__main__":
    try:
        node = YoloCWNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass