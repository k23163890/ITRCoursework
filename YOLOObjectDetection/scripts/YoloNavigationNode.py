# !/usr/bin/env python3

import rospy
import actionlib
import sys
import cv2
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import darknet

VIDEO_SOURCE = "/home/k23163890/catkin_ws/src/YOLOObjectDetection/cs_cw_2526_videos"


class YoloNavigationNode:
    def __init__(self):
        rospy.init_node('yolo_navigation_node', anonymous=True)

        self.config_path = "yolov4.cfg"
        self.weights_path = "yolov4.weights"
        self.meta_path = "coco.data"

        self.locations = {
            "book": {"x": -5.0, "y": 5.0},
            "cell phone": {"x": 5.0, "y": 5.0},
            "bottle": {"x": 0.0, "y": 0.0}
        }

        rospy.loginfo("Loading YOLO Network...")
        try:
            self.network, self.class_names, self.class_colors = darknet.load_network(
                self.config_path,
                self.meta_path,
                self.weights_path,
                batch_size=1
            )
            self.net_width = darknet.network_width(self.network)
            self.net_height = darknet.network_height(self.network)
        except Exception as e:
            rospy.logerr(f"Failed to load YOLO: {e}")
            sys.exit(1)

        self.darknet_image = darknet.make_image(self.net_width, self.net_height, 3)

        self.bridge = CvBridge()
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base.")

        self.current_target_label = None
        self.program_active = True

        if VIDEO_SOURCE is None:
            self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
            rospy.loginfo("Node started in WEBCAM mode.")
        else:
            rospy.loginfo(f"Node started in FILE mode: {VIDEO_SOURCE}")

    def process_frame(self, cv_image):
        if not self.program_active:
            return

        image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        image_resized = cv2.resize(image_rgb, (self.net_width, self.net_height), interpolation=cv2.INTER_LINEAR)

        darknet.copy_image_from_bytes(self.darknet_image, image_resized.tobytes())

        detections = darknet.detect_image(self.network, self.class_names, self.darknet_image, thresh=0.5)

        detected_labels = [d[0] for d in detections]

        if detected_labels:
            rospy.loginfo(f"Seen: {detected_labels}")

        if "apple" in detected_labels:
            rospy.logwarn("APPLE DETECTED! Stopping robot.")
            self.client.cancel_all_goals()
            self.program_active = False
            rospy.signal_shutdown("Apple found")
            return

        target_label = None
        if "book" in detected_labels:
            target_label = "book"
        elif "cell phone" in detected_labels:
            target_label = "cell phone"
        elif "bottle" in detected_labels:
            target_label = "bottle"

        if target_label and target_label != self.current_target_label:
            rospy.loginfo(f"Navigate Trigger: {target_label.upper()}")
            self.send_goal_to_location(target_label)
            self.current_target_label = target_label

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_frame(cv_image)
        except CvBridgeError as e:
            rospy.logerr(e)

    def send_goal_to_location(self, label):
        if label not in self.locations:
            return

        coords = self.locations[label]
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = coords["x"]
        goal.target_pose.pose.position.y = coords["y"]
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo(f"Sending goal: {label} -> ({coords['x']}, {coords['y']})")
        self.client.send_goal(goal)

    def run(self):
        if VIDEO_SOURCE is not None:
            cap = cv2.VideoCapture(VIDEO_SOURCE)
            rate = rospy.Rate(30)
            while not rospy.is_shutdown() and cap.isOpened() and self.program_active:
                ret, frame = cap.read()
                if ret:
                    self.process_frame(frame)
                    cv2.waitKey(1)
                else:
                    break
                rate.sleep()
        else:
            rospy.spin()


if __name__ == '__main__':
    try:
        node = YoloNavigationNode()
        node.run()
    except rospy.ROSInterruptException:
        pass