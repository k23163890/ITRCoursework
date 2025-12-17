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
    "F": (10.0, 3.8, 0.0),
}

class WaitState(smach.State):
    def __init__(self, duration=10.0):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'])
        self.duration = duration

    def execute(self, userdata):
        rospy.loginfo(f"[CheckRules] Waiting {self.duration} seconds for localisation...")
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
    def __init__(self, room_letter):
        self.room_letter = room_letter
        
        super(GoToRoomState, self).__init__(
            'move_base', 
            MoveBaseAction, 
            goal_cb=self._make_goal,
            result_cb=self._result_callback
        )

    def _make_goal(self, userdata, goal):
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
        if rospy.is_shutdown():
            return 'preempted'
        return None