#!/usr/bin/env python3
import rospy
import smach
from smach_ros import SimpleActionState
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

ROOM_COORDS = {
    "A": (1.8, 8.6, 0.0),
    "B": (5.5, 8.2, 0.0),
    "C": (10.3, 8.1, 0.0),
    "D": (2.0, 3.6, 0.0),
    "E": (6.6, 4.8, 0.0),
    "F": (9.0, 3.0, 0.0),
}

class WaitState(smach.State):
    """Waits for a specific duration (e.g., for localization stabilization)."""
    def __init__(self, duration=10.0):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'])
        self.duration = duration

    def execute(self, userdata):
        rospy.loginfo(f"[CheckRules] Waiting {self.duration} seconds for localization...")
        start_time = rospy.Time.now()
        
        try:
            while (rospy.Time.now() - start_time).to_sec() < self.duration:
                if self.preempt_requested() or rospy.is_shutdown():
                    self.service_preempt()
                    return 'preempted'
                rospy.sleep(0.1)
        except rospy.ROSInterruptException:
            return 'preempted'
            
        return 'succeeded'

class GoToRoomState(SimpleActionState):
    """
    Navigates to a room using SimpleActionState.
    """
    def __init__(self, room_letter):
        self.room_letter = room_letter
        
        super(GoToRoomState, self).__init__(
            'move_base', 
            MoveBaseAction, 
            goal_cb=self._make_goal,
            result_cb=self._result_callback
        )

    def _make_goal(self, userdata, goal):
        """Internal callback to create the goal with current time."""
        if self.room_letter not in ROOM_COORDS:
            rospy.logerr(f"[Navigation] Unknown room: {self.room_letter}")
            return None

        x, y, theta = ROOM_COORDS[self.room_letter]
        
        target = MoveBaseGoal()
        target.target_pose.header.frame_id = "map"
        target.target_pose.header.stamp = rospy.Time.now()
        target.target_pose.pose.position.x = x
        target.target_pose.pose.position.y = y
        
        q = quaternion_from_euler(0, 0, theta)
        target.target_pose.pose.orientation = Quaternion(*q)
        
        rospy.loginfo(f"[Navigation] Going to Room {self.room_letter} ({x}, {y})")
        return target

    def _result_callback(self, userdata, status, result):
        """
        Called when the action finishes. 
        If ROS is shutting down, we force a 'preempted' outcome 
        to break any retry loops in the State Machine.
        """
        if rospy.is_shutdown():
            return 'preempted'
        return None