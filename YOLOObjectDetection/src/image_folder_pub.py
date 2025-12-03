#!/usr/bin/env python3
# coding=utf-8

import rospy
import cv2
import os
import random
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from YOLOObjectDetection.srv import YOLOFrame

def main():
    rospy.init_node('image_folder_publisher')

    # ------------ PATH TO YOUR IMAGES ------------
    folder_path = "/home/k23163890/catkin_ws/src/YOLOObjectDetection/cs_cw_2526_videos"
    # ---------------------------------------------

    pub = rospy.Publisher("/usb_cam/image_raw", Image, queue_size=1)
    bridge = CvBridge()

    publish_delay = rospy.get_param("~publish_delay", 2.5)

    # Track robot busy state
    nav_busy = False
    current_msg = None

    def busy_cb(msg):
        nonlocal nav_busy
        nav_busy = msg.data

    rospy.Subscriber("/nav_busy", Bool, busy_cb)

    # Validate folder
    if not os.path.isdir(folder_path):
        rospy.logerr(f"[image_folder_pub] Folder does not exist: {folder_path}")
        return

    valid_ext = (".jpg", ".jpeg", ".png", ".bmp")
    files = [f for f in os.listdir(folder_path) if f.lower().endswith(valid_ext)]

    if not files:
        rospy.logerr("[image_folder_pub] No image files found!")
        return

    rospy.loginfo(f"[image_folder_pub] Loaded {len(files)} images. Shuffling...")
    random.shuffle(files)

    # Wait for YOLO service
    rospy.loginfo("[image_folder_pub] Waiting for YOLO service...")
    rospy.wait_for_service('/detect_frame')
    detect_srv = rospy.ServiceProxy('/detect_frame', YOLOFrame)

    idx = 0

    while not rospy.is_shutdown():

        # If navigating, freeze publication on same image
        if nav_busy and current_msg is not None:
            pub.publish(current_msg)
            rospy.sleep(0.3)
            continue

        if idx >= len(files):
            rospy.loginfo("[image_folder_pub] Finished sequence. Reshuffling.")
            random.shuffle(files)
            idx = 0

        # Load next image
        img_path = os.path.join(folder_path, files[idx])
        img = cv2.imread(img_path)

        if img is None:
            rospy.logwarn(f"[image_folder_pub] Cannot load: {files[idx]}")
            idx += 1
            continue

        current_msg = bridge.cv2_to_imgmsg(img, "bgr8")
        pub.publish(current_msg)
        rospy.loginfo(f"[image_folder_pub] Published: {files[idx]}")

        rospy.sleep(publish_delay)

        # Query YOLO
        try:
            res = detect_srv()
        except rospy.ServiceException:
            rospy.logerr("[image_folder_pub] YOLO service call failed.")
            continue

        detected_names = [d.name.lower() for d in res.detections]

        if "apple" in detected_names:
            rospy.logwarn("[image_folder_pub] APPLE DETECTED — stopping image publisher.")
            break

        if not nav_busy:
            idx += 1


if __name__ == "__main__":
    main()
