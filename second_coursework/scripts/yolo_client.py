#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

_latest = []   # list of last seen class names


def _cb(msg):
    global _latest
    # expecting comma-separated names: "person,banana,bottle"
    text = msg.data.strip()
    if not text:
        _latest = []
    else:
        _latest = [name.strip() for name in text.split(",") if name.strip()]


def init_yolo():
    """
    Subscribe once to /detected_objects and keep
    the latest list of detected classes in _latest.
    """
    rospy.Subscriber("/detected_objects", String, _cb)
    rospy.loginfo("[yolo_client] Subscribed to /detected_objects")


def detect_objects():
    """
    Return the most recently detected object names (list of strings).
    """
    return list(_latest)
