#!/usr/bin/env python3
# coding=utf-8

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from yolov4 import Detector


class YoloCWNode:
    def __init__(self):
        rospy.init_node("yolo_cw_node")

        self.bridge = CvBridge()
        self.cv_image = None

        rospy.loginfo("[YOLO CW] Initialising YOLOv4 detector...")

        # Paths are the same ones you used in your previous YOLO lab
        self.detector = Detector(
            gpu_id=0,
            config_path="/opt/darknet/cfg/yolov4.cfg",
            weights_path="/opt/darknet/yolov4.weights",
            lib_darknet_path="/opt/darknet/libdarknet.so",
            # try param first, then fallback
            meta_path=rospy.get_param(
                "~coco_data",
                "/opt/darknet/cfg/coco.data"
            )
        )

        # 📷 Subscribe to the camera images from the coursework video player
        self.image_sub = rospy.Subscriber(
            "/camera/image", Image, self.img_callback, queue_size=1
        )

        # 📡 Publish just the class names (comma separated) for your yolo_client
        self.detected_pub = rospy.Publisher(
            "/detected_objects", String, queue_size=1
        )

        rospy.loginfo("[YOLO CW] Ready. Waiting for images on /camera/image...")

    def img_callback(self, msg):
        # Convert ROS image to OpenCV
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn(f"[YOLO CW] cv_bridge error: {e}")
            return

        # Resize to network input size
        img_net = cv2.resize(
            self.cv_image,
            (self.detector.network_width(), self.detector.network_height())
        )

        # Run YOLO detection
        detections = self.detector.perform_detect(
            image_path_or_buf=img_net,
            show_image=False
        )

        # Extract class names in lowercase
        names = [det.class_name.lower() for det in detections]

        # Publish comma-separated string
        msg_out = String()
        msg_out.data = ",".join(names)
        self.detected_pub.publish(msg_out)

        rospy.loginfo(f"[YOLO CW] Detected objects: {names}")


if __name__ == "__main__":
    try:
        node = YoloCWNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
