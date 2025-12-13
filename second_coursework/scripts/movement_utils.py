#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

# --------------------------
# Coordinates for each room
# YOU MUST ADJUST THESE USING RVIZ!!
# --------------------------
ROOM_COORDS = {
    "A": (1.8, 8.6, 0.0),
    "B": (5.5, 8.2, 0.0),
    "C": (10.3, 8.1, 0.0),
    "D": (2.0, 3.6, 0.0),
    "E": (6.6, 4.8, 0.0),
    "F": (10.0, 3.8, 0.0),
}

# Create move_base client once
move_base_client = None


def init_move_base():
    global move_base_client

    if move_base_client is None:
        rospy.loginfo("[movement_utils] Connecting to move_base...")
        move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        move_base_client.wait_for_server()
        rospy.loginfo("[movement_utils] Connected.")


def go_to_room(room_letter):
    """Send a move_base goal to the center of a room."""
    init_move_base()

    if room_letter not in ROOM_COORDS:
        rospy.logerr(f"[movement_utils] Unknown room: {room_letter}")
        return False

    x, y, theta = ROOM_COORDS[room_letter]

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

    q = quaternion_from_euler(0, 0, theta)
    goal.target_pose.pose.orientation = Quaternion(*q)

    rospy.loginfo(f"[movement_utils] Sending robot to room {room_letter} @ ({x}, {y})")

    move_base_client.send_goal(goal)
    move_base_client.wait_for_result()

    if move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo(f"[movement_utils] Arrived at room {room_letter}")
        return True
    else:
        rospy.logwarn(f"[movement_utils] Failed to reach room {room_letter}")
        return False
