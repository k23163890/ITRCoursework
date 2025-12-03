#!/usr/bin/env python3
# coding=utf-8

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from YOLOObjectDetection.srv import YOLOFrame

# ---- MAP TARGETS ----
TOP_LEFT   = {"x": 2.0,  "y": 8.5, "w": 1.0}
TOP_RIGHT  = {"x": 10.0, "y": 8.5, "w": 1.0}
BOTTOM_MID = {"x": 6.1,  "y": 3.5, "w": 1.0}


class ObjectNavBehaviour:
    def __init__(self):
        rospy.loginfo("Waiting for /detect_frame service...")
        rospy.wait_for_service("/detect_frame")
        self.detect_srv = rospy.ServiceProxy("/detect_frame", YOLOFrame)
        rospy.loginfo("Connected to YOLO detection service.")

        # move_base client
        self.mb_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base...")
        self.mb_client.wait_for_server()
        rospy.loginfo("move_base ready.")

        # Publisher to tell image node when nav is active
        self.busy_pub = rospy.Publisher("/nav_busy", Bool, queue_size=1)

        # Store last target so we don't resend goals endlessly
        self.last_target = None


    # ---- SEND NAV GOAL ----
    def send_goal(self, x, y, w, label):
        if self.last_target == label:
            rospy.loginfo(f"Already at target for {label}, skipping new goal.")
            return

        self.last_target = label
        rospy.loginfo(f"[NAV] Sending robot to {label} @ ({x}, {y})")

        goal = MoveBaseGoal()
        goal.target_pose = PoseStamped()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = w

        # Tell image publisher to freeze images
        self.busy_pub.publish(True)

        self.mb_client.send_goal(goal)
        self.mb_client.wait_for_result()

        # Navigation complete → resume image streaming
        self.busy_pub.publish(False)

        rospy.loginfo("[NAV] Reached target.")


    # ---- MAIN LOGIC LOOP ----
    def run(self):
        rate = rospy.Rate(1.0)   # check 1 fps

        while not rospy.is_shutdown():
            try:
                res = self.detect_srv()
            except Exception as e:
                rospy.logwarn(f"YOLO service failed: {e}")
                rate.sleep()
                continue

            names = [d.name.lower() for d in res.detections]

            if not names:
                rospy.loginfo("Detected: nothing")
                rate.sleep()
                continue

            rospy.loginfo(f"Detected objects: {names}")

            # ---- STOP on APPLE ----
            if "apple" in names:
                rospy.logwarn("APPLE FOUND — stopping robot behaviour.")
                rospy.signal_shutdown("Apple detected")
                break

            # ---- OBJECT BEHAVIOUR ----
            if "book" in names:
                self.send_goal(**TOP_LEFT, label="book")

            elif "cell phone" in names or "cellphone" in names:
                self.send_goal(**TOP_RIGHT, label="cell phone")

            elif "bottle" in names:
                self.send_goal(**BOTTOM_MID, label="bottle")

            else:
                rospy.loginfo("Ignored: no relevant objects")

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("object_navigation_node")
    ObjectNavBehaviour().run()
