#!/usr/bin/env python3
# coding=utf-8

import rospy
import smach
import smach_ros
import actionlib
import random
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from second_coursework.msg import FindObjectAction, FindObjectResult, CheckRulesAction
from YOLOObjectDetection.srv import YOLOFrame

# --- CONFIGURATION ---
# IMPORTANT: Verify these X/Y coordinates on your map!
# The labels (F, C, E) match your coursework description.

LIVING_ROOM_E = {"x": 10.4, "y": 8.9, "w": 1.57, "label": "E"}

# Search spots for random patrolling
SEARCH_SPOTS = [
    {"x": 10.7, "y": 4.3, "w": 1.0, "label": "F"}, # Kitchen
    {"x": 2.0, "y": 8.5, "w": 1.0, "label": "C"},  # Bedroom
    {"x": 10.4, "y": 8.9, "w": 1.0, "label": "E"}, # Living Room
    {"x": 6.0, "y": 6.0, "w": 1.0, "label": "A"},  # Room A (Example coords)
    {"x": 4.0, "y": 4.0, "w": 1.0, "label": "B"},  # Room B (Example coords)
    {"x": 8.0, "y": 2.0, "w": 1.0, "label": "D"},  # Room D (Example coords)
]

# Helper for room lookup
def get_closest_room_label(x, y):
    min_dist = float('inf')
    closest_label = 'unknown'
    for spot in SEARCH_SPOTS:
        dist = ((x - spot["x"])**2 + (y - spot["y"])**2)**0.5
        if dist < 3.0: # Threshold radius
            if dist < min_dist:
                min_dist = dist
                closest_label = spot["label"]
    return closest_label

class NavigateToState(smach.State):
    """Generic Navigation State"""
    def __init__(self, target_coords=None):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'], 
                            input_keys=['target_coords'])
        self.fixed_target = target_coords
        self.move_base_client = None
        
    def execute(self, userdata):
        # Determine target: either fixed (constructor) or from userdata (random)
        target = self.fixed_target if self.fixed_target else userdata.target_coords
        
        rospy.loginfo(f"Navigating to {target.get('label', 'target')} ({target['x']}, {target['y']})...")
        
        # Lazy connect
        if self.move_base_client is None:
            self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
            if not self.move_base_client.wait_for_server(timeout=rospy.Duration(5.0)):
                rospy.logwarn("move_base not ready, skipping move")
                return 'succeeded' 
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = target["x"]
        goal.target_pose.pose.position.y = target["y"]
        goal.target_pose.pose.orientation.w = target["w"]
        
        self.move_base_client.send_goal(goal)
        
        while not rospy.is_shutdown():
            if self.preempt_requested():
                self.move_base_client.cancel_all_goals()
                self.service_preempt()
                return 'preempted'
            
            state = self.move_base_client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                return 'succeeded'
            elif state in [actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.REJECTED]:
                rospy.logwarn("Navigation failed/blocked, continuing...")
                return 'succeeded'
            
            rospy.sleep(0.1)
        return 'preempted'

class PickRandomSpotState(smach.State):
    """Picks a random location from SEARCH_SPOTS"""
    def __init__(self):
        smach.State.__init__(self, outcomes=['selected'], output_keys=['target_coords'])
        
    def execute(self, userdata):
        spot = random.choice(SEARCH_SPOTS)
        userdata.target_coords = spot
        rospy.loginfo(f"Random Search: Decided to check Room {spot['label']}")
        return 'selected'

class SearchObjectState(smach.State):
    """Checks YOLO for the object"""
    def __init__(self, object_name):
        smach.State.__init__(self, outcomes=['found', 'not_found', 'preempted'],
                            output_keys=['room_label', 'object_name'])
        self.object_name = object_name.lower()
        self.yolo_client = None
        self.robot_pos = (0,0)
        
    def execute(self, userdata):
        rospy.loginfo(f"Scanning for {self.object_name}...")
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        # Get current position for reporting
        try:
            msg = rospy.wait_for_message('/odom', Odometry, timeout=1.0)
            self.robot_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        except:
            pass
            
        # Call YOLO
        if self.yolo_client is None:
            try:
                rospy.wait_for_service("/detect_frame", timeout=2.0)
                self.yolo_client = rospy.ServiceProxy("/detect_frame", YOLOFrame)
            except:
                rospy.logwarn("YOLO service unavailable")
                return 'not_found'
        
        try:
            response = self.yolo_client()
            detected = [d.name.lower() for d in response.detections]
            rospy.loginfo(f"I see: {detected}")
            
            if self.object_name in detected:
                userdata.object_name = self.object_name
                # Determine current room based on position
                userdata.room_label = get_closest_room_label(self.robot_pos[0], self.robot_pos[1])
                return 'found'
            return 'not_found'
        except Exception as e:
            rospy.logerr(f"YOLO Check failed: {e}")
            return 'not_found'

class AnnounceState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.tts_pub = rospy.Publisher('/tts', String, queue_size=1)
        
    def execute(self, userdata):
        # Format: "I found [Object] in room [X]"
        msg = f"I found {userdata.object_name} in room {userdata.room_label}"
        rospy.loginfo(f"ANNOUNCING: {msg}")
        
        # Publish multiple times to ensure TTS catches it
        for _ in range(3):
            self.tts_pub.publish(msg)
            rospy.sleep(0.5)
            
        rospy.sleep(2.0)
        return 'succeeded'

class FindObjectActionServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('/find_object_action', FindObjectAction, self.execute_cb, False)
        self.server.start()
        self.check_rules_client = actionlib.SimpleActionClient('/check_rules', CheckRulesAction)
        rospy.loginfo("FindObject Server Started (With Random Search)")
        
    def execute_cb(self, goal):
        rospy.loginfo(f"--- STARTING SEARCH FOR: {goal.object_name} ---")
        
        # 1. Stop the CheckRules action
        rospy.loginfo("Preempting CheckRules...")
        self.check_rules_client.cancel_all_goals()
        rospy.sleep(1.0)
        
        # 2. Run SMACH
        sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
        sm.userdata.room_label = ''
        sm.userdata.object_name = goal.object_name
        sm.userdata.target_coords = {} # Placeholder for random spots
        
        # Register preempt callback
        def preempt_cb():
            if self.server.is_preempt_requested():
                sm.request_preempt()
        self.server.register_preempt_callback(preempt_cb)
        
        with sm:
            # Step 1: Scan current location
            sm.add('SEARCH_START', SearchObjectState(goal.object_name), 
                   transitions={'found':'NAV_TO_LIVING_ROOM', 
                              'not_found':'PICK_SPOT', 
                              'preempted':'preempted'})
            
            # Step 2: Pick a random spot
            sm.add('PICK_SPOT', PickRandomSpotState(),
                   transitions={'selected':'NAV_RANDOM'})
            
            # Step 3: Go to that spot
            sm.add('NAV_RANDOM', NavigateToState(),
                   transitions={'succeeded':'SEARCH_AFTER_MOVE',
                              'preempted':'preempted'})
            
            # Step 4: Scan again
            sm.add('SEARCH_AFTER_MOVE', SearchObjectState(goal.object_name),
                   transitions={'found':'NAV_TO_LIVING_ROOM', 
                              'not_found':'PICK_SPOT', # LOOP BACK if not found
                              'preempted':'preempted'})
            
            # Found sequence
            sm.add('NAV_TO_LIVING_ROOM', NavigateToState(LIVING_ROOM_E),
                   transitions={'succeeded':'ANNOUNCE', 'preempted':'preempted'})
            sm.add('ANNOUNCE', AnnounceState(),
                   transitions={'succeeded':'succeeded'})
                   
        outcome = sm.execute()
        
        result = FindObjectResult()
        if outcome == 'succeeded':
            result.success = True
            result.room_found = sm.userdata.room_label
            result.object_found = sm.userdata.object_name
            self.server.set_succeeded(result)
        elif outcome == 'preempted':
            self.server.set_preempted(result)
        else:
            self.server.set_aborted(result)

if __name__ == '__main__':
    rospy.init_node('find_object_action_server')
    FindObjectActionServer()
    rospy.spin()