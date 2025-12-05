#!/usr/bin/env python3
# coding=utf-8

import rospy
import smach
import smach_ros
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from second_coursework.msg import CheckRulesAction, CheckRulesFeedback, CheckRulesResult
from YOLOObjectDetection.srv import YOLOFrame

# Coordinates
KITCHEN = {"x": 10.7, "y": 4.3, "w": 1.0}
BEDROOM = {"x": 2.0, "y": 8.5, "w": 1.0}
# Food items list
FOOD_ITEMS = ['apple', 'banana', 'broccoli', 'pizza', 'sandwich', 'bottle', 'fork', 'cake', 'donut']

class NavigateToRoomState(smach.State):
    """Navigate to a specific room."""
    def __init__(self, room_name, room_coords):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'])
        self.room_name = room_name
        self.room_coords = room_coords
        self.move_base_client = None
        
    def execute(self, userdata):
        rospy.loginfo(f"Navigating to {self.room_name}...")
        
        # 1. Check for preemption immediately
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        # Lazy connect to move_base
        if self.move_base_client is None:
            self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
            # Use a timeout so we don't hang forever if move_base is down
            if not self.move_base_client.wait_for_server(timeout=rospy.Duration(5.0)):
                rospy.logwarn("move_base server not available, skipping navigation")
                return 'succeeded' # Skip to check if nav fails
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.room_coords["x"]
        goal.target_pose.pose.position.y = self.room_coords["y"]
        goal.target_pose.pose.orientation.w = self.room_coords["w"]
        
        self.move_base_client.send_goal(goal)
        
        # Monitor navigation
        while not rospy.is_shutdown():
            # 2. Check for preemption while moving
            if self.preempt_requested():
                self.move_base_client.cancel_all_goals()
                self.service_preempt()
                return 'preempted'
            
            state = self.move_base_client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                return 'succeeded'
            elif state in [actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.REJECTED]:
                return 'succeeded' # Continue patrolling anyway
            
            rospy.sleep(0.1)
        return 'preempted'

class CheckKitchenState(smach.State):
    def __init__(self, action_server):
        smach.State.__init__(self, outcomes=['violation', 'ok', 'preempted'])
        self.action_server = action_server
        self.yolo_client = None
        
    def execute(self, userdata):
        rospy.loginfo("Checking kitchen...")
        
        # Check preemption
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        # Lazy load YOLO
        if self.yolo_client is None:
            try:
                rospy.wait_for_service("/detect_frame", timeout=2.0)
                self.yolo_client = rospy.ServiceProxy("/detect_frame", YOLOFrame)
            except rospy.ROSException:
                return 'ok' # Skip if service not ready
        
        try:
            response = self.yolo_client()
            detected_names = [d.name.lower() for d in response.detections]
            
            if 'person' in detected_names:
                rospy.logwarn("RULE 1 VIOLATION: Person in kitchen!")
                feedback = CheckRulesFeedback()
                feedback.broken_rule = 1
                self.action_server.publish_feedback(feedback)
                return 'violation'
            return 'ok'
        except Exception:
            return 'ok'

class CheckBedroomState(smach.State):
    def __init__(self, action_server):
        smach.State.__init__(self, outcomes=['violation', 'ok', 'preempted'])
        self.action_server = action_server
        self.yolo_client = None
        
    def execute(self, userdata):
        rospy.loginfo("Checking bedroom...")
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
            
        if self.yolo_client is None:
            try:
                rospy.wait_for_service("/detect_frame", timeout=2.0)
                self.yolo_client = rospy.ServiceProxy("/detect_frame", YOLOFrame)
            except rospy.ROSException:
                return 'ok'
        
        try:
            response = self.yolo_client()
            detected_names = [d.name.lower() for d in response.detections]
            
            for food in FOOD_ITEMS:
                if food in detected_names:
                    rospy.logwarn(f"RULE 2 VIOLATION: {food} in bedroom!")
                    feedback = CheckRulesFeedback()
                    feedback.broken_rule = 2
                    self.action_server.publish_feedback(feedback)
                    return 'violation'
            return 'ok'
        except Exception:
            return 'ok'

class CheckRulesActionServer:
    def __init__(self):
        self.action_server = actionlib.SimpleActionServer(
            '/check_rules',
            CheckRulesAction,
            self.execute_cb,
            auto_start=False
        )
        self.action_server.start()
        rospy.loginfo("CheckRules Action Server Started")
        
    def execute_cb(self, goal):
        sm = smach.StateMachine(outcomes=['succeeded', 'preempted'])
        
        # --- CRITICAL FIX: PREEMPTION CALLBACK ---
        # This allows the ROS Action Server to signal SMACH to stop.
        def preempt_cb():
            if self.action_server.is_preempt_requested():
                sm.request_preempt()
        
        self.action_server.register_preempt_callback(preempt_cb)
        # -----------------------------------------
        
        with sm:
            sm.add('NAV_TO_KITCHEN', NavigateToRoomState("kitchen", KITCHEN),
                   transitions={'succeeded': 'CHECK_KITCHEN', 'preempted': 'preempted'})
            sm.add('CHECK_KITCHEN', CheckKitchenState(self.action_server),
                   transitions={'violation': 'NAV_TO_BEDROOM', 'ok': 'NAV_TO_BEDROOM', 'preempted': 'preempted'})
            sm.add('NAV_TO_BEDROOM', NavigateToRoomState("bedroom", BEDROOM),
                   transitions={'succeeded': 'CHECK_BEDROOM', 'preempted': 'preempted'})
            sm.add('CHECK_BEDROOM', CheckBedroomState(self.action_server),
                   transitions={'violation': 'NAV_TO_KITCHEN', 'ok': 'NAV_TO_KITCHEN', 'preempted': 'preempted'})
        
        outcome = sm.execute()
        
        result = CheckRulesResult()
        if outcome == 'preempted':
            self.action_server.set_preempted(result)
            rospy.loginfo("CheckRules action preempted successfully.")
        else:
            self.action_server.set_succeeded(result)

if __name__ == '__main__':
    try:
        rospy.init_node('check_rules_action_server')
        # Wait logic is handled in main_node.py, but safe to keep here too
        server = CheckRulesActionServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass