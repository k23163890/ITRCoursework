#!/usr/bin/env python3
# coding=utf-8

import rospy
import cv2
import threading
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
        
        # Locks to prevent crashes when reading/writing images or using detector simultaneously
        self.img_lock = threading.Lock()
        self.det_lock = threading.Lock()

        rospy.loginfo("[YOLO CW] Initialising YOLOv4 detector...")

        self.detector = Detector(
            gpu_id=0,
            config_path="/opt/darknet/cfg/yolov4.cfg",
            weights_path="/opt/darknet/yolov4.weights",
            lib_darknet_path="/opt/darknet/libdarknet.so",
            meta_path=rospy.get_param("~coco_data", "/opt/darknet/cfg/coco.data")
        )

        self.image_sub = rospy.Subscriber(
            "/camera/image", Image, self.img_callback, queue_size=1
        )

        self.detected_pub = rospy.Publisher(
            "/detected_objects", String, queue_size=1
        )

        self.service = rospy.Service('/detect_frame', YOLOFrame, self.handle_detect_frame)
        
        # START THE BACKGROUND THREAD
        self.log_thread = threading.Thread(target=self.logging_loop)
        self.log_thread.daemon = True # Kills thread when node stops
        self.log_thread.start()

        rospy.loginfo("[YOLO CW] Ready. Service /detect_frame is available.")

    def img_callback(self, msg):
        """
        FAST UPDATE: Only converts the image.
        This ensures self.cv_image is ALWAYS fresh (0ms lag).
        """
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self.img_lock:
                self.cv_image = img
        except Exception as e:
            rospy.logwarn(f"[YOLO CW] cv_bridge error: {e}")

    def logging_loop(self):
        """
        BACKGROUND TASK: Runs detection periodically for logs/topic.
        Does not block the image update!
        """
        rate = rospy.Rate(1.0) # Run logs at 1Hz (once per second)
        while not rospy.is_shutdown():
            # 1. Grab the latest image safely
            img_to_process = None
            with self.img_lock:
                if self.cv_image is not None:
                    img_to_process = self.cv_image.copy()
            
            if img_to_process is None:
                rate.sleep()
                continue

            # 2. Run Heavy Detection (Safe behind lock)
            try:
                with self.det_lock:
                    img_net = cv2.resize(
                        img_to_process,
                        (self.detector.network_width(), self.detector.network_height())
                    )
                    detections = self.detector.perform_detect(image_path_or_buf=img_net, show_image=False)

                # 3. Publish
                names = [det.class_name.lower() for det in detections]
                msg_out = String()
                msg_out.data = ",".join(names)
                self.detected_pub.publish(msg_out)
                
                # Visible log
                rospy.loginfo(f"[YOLO CW] Detected objects: {names}")

            except Exception as e:
                rospy.logwarn(f"[YOLO CW] Log Error: {e}")

            rate.sleep()

    def handle_detect_frame(self, req):
        """Service callback: Returns detections to the State Machine."""
        resp = YOLOFrameResponse()

        # 1. Grab fresh image
        img_to_process = None
        with self.img_lock:
            if self.cv_image is not None:
                img_to_process = self.cv_image.copy()

        if img_to_process is None:
            rospy.logwarn("[YOLO CW] Service called but no image received yet!")
            return resp

        # 2. Run Detection (Priority)
        # We use the lock to ensure we don't crash the detector if the background thread is using it
        with self.det_lock:
            img_net = cv2.resize(
                img_to_process,
                (self.detector.network_width(), self.detector.network_height())
            )
            detections = self.detector.perform_detect(image_path_or_buf=img_net, show_image=False)

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