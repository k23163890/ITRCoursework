#!/usr/bin/env python3
import rospy
import smach
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

# Embedded coordinates from the original movement_utils
ROOM_COORDS = {
    "A": (1.8, 8.6, 0.0),
    "B": (5.5, 8.2, 0.0),
    "C": (10.3, 8.1, 0.0),
    "D": (2.0, 3.6, 0.0),
    "E": (6.6, 4.8, 0.0),
    "F": (10.0, 3.8, 0.0),
}

class WaitState(smach.State):
    """Waits for a specific duration (e.g., for localization stabilization)."""
    def __init__(self, duration=10.0):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'])
        self.duration = duration

    def execute(self, userdata):
        rospy.loginfo(f"[CheckRules] Waiting {self.duration} seconds for localization...")
        start_time = rospy.Time.now()
        
        while (rospy.Time.now() - start_time).to_sec() < self.duration:
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            rospy.sleep(0.1)
            
        return 'succeeded'

class GoToRoomState(smach.State):
    """
    Navigates to a room. 
    If 'room_letter' is provided in __init__, it uses that.
    Otherwise, it looks for 'target_room' in userdata.
    """
    def __init__(self, room_letter=''):
        smach.State.__init__(self, 
                             outcomes=['arrived', 'failed', 'preempted'],
                             input_keys=['target_room'])
        self.fixed_room = room_letter
        self.client = None # Lazy initialization

    def _init_client(self):
        if self.client is None:
            rospy.loginfo("[GoToRoomState] Connecting to move_base...")
            self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            if not self.client.wait_for_server(timeout=rospy.Duration(5.0)):
                rospy.logwarn("[GoToRoomState] move_base server not available!")
                return False
        return True

    def execute(self, userdata):
        # 1. Determine target room
        target = self.fixed_room if self.fixed_room else userdata.target_room
        
        rospy.loginfo(f"[Navigation] Preparing to go to Room {target}...")

        # 2. Check Preemption
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        # 3. Validate Room
        if target not in ROOM_COORDS:
            rospy.logerr(f"[Navigation] Unknown room: {target}")
            return 'failed'

        # 4. Initialize Client
        if not self._init_client():
            return 'failed'

        # 5. Create Goal
        x, y, theta = ROOM_COORDS[target]
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, theta)
        goal.target_pose.pose.orientation = Quaternion(*q)

        # 6. Send Goal
        rospy.loginfo(f"[Navigation] Sending goal: Room {target} at ({x}, {y})")
        self.client.send_goal(goal)

        # 7. Wait for result with preemption handling
        while not self.client.wait_for_result(timeout=rospy.Duration(0.5)):
            if self.preempt_requested():
                self.client.cancel_goal()
                self.service_preempt()
                return 'preempted'
            if rospy.is_shutdown():
                return 'failed'

        # 8. Check Outcome
        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"[Navigation] Arrived at Room {target}")
            return 'arrived'
        else:
            rospy.logwarn(f"[Navigation] Failed to reach Room {target}")
            return 'failed'