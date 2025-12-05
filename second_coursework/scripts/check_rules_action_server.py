#!/usr/bin/env python3
# coding=utf-8

import rospy
import smach
import smach_ros
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from second_coursework.msg import CheckRulesAction, CheckRulesFeedback, CheckRulesResult
from YOLOObjectDetection.srv import YOLOFrame

# Room coordinates (need to be determined from the map)
# Kitchen coordinates
KITCHEN = {"x": 10.7, "y": 4.3, "w": 1.0}
# Bedroom coordinates (approximate - may need adjustment)
BEDROOM = {"x": 2.0, "y": 8.5, "w": 1.0}

# Food items that shouldn't be in bedroom
FOOD_ITEMS = ['apple', 'banana', 'broccoli', 'pizza', 'sandwich', 'bottle', 'fork']

class NavigateToRoomState(smach.State):
    """Navigate to a specific room."""
    def __init__(self, room_name, room_coords):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'])
        self.room_name = room_name
        self.room_coords = room_coords
        self.move_base_client = None
        
    def execute(self, userdata):
        rospy.loginfo(f"Navigating to {self.room_name}...")
        
        if self.move_base_client is None:
            self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
            if not self.move_base_client.wait_for_server(timeout=rospy.Duration(5.0)):
                rospy.logerr("move_base server not available")
                return 'succeeded'  # Continue anyway
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.room_coords["x"]
        goal.target_pose.pose.position.y = self.room_coords["y"]
        goal.target_pose.pose.orientation.w = self.room_coords["w"]
        
        self.move_base_client.send_goal(goal)
        
        while not rospy.is_shutdown():
            if self.preempt_requested():
                self.move_base_client.cancel_all_goals()
                self.service_preempt()
                return 'preempted'
            
            state = self.move_base_client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo(f"Arrived at {self.room_name}")
                return 'succeeded'
            elif state in [actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.REJECTED]:
                rospy.logwarn(f"Failed to reach {self.room_name}, continuing anyway")
                return 'succeeded'
            
            rospy.sleep(0.1)
        
        return 'preempted'


class CheckKitchenState(smach.State):
    """Check kitchen for rule violations (person in kitchen = rule 1)."""
    def __init__(self, action_server):
        smach.State.__init__(self, outcomes=['violation', 'ok', 'preempted'])
        self.action_server = action_server
        self.yolo_client = None
        
    def execute(self, userdata):
        rospy.loginfo("Checking kitchen for violations...")
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        if self.yolo_client is None:
            rospy.wait_for_service("/detect_frame", timeout=2.0)
            self.yolo_client = rospy.ServiceProxy("/detect_frame", YOLOFrame)
        
        try:
            response = self.yolo_client()
            detected_names = [d.name.lower() for d in response.detections]
            
            # Check for person in kitchen (Rule 1)
            if 'person' in detected_names:
                rospy.logwarn("RULE 1 VIOLATION: Person found in kitchen!")
                feedback = CheckRulesFeedback()
                feedback.broken_rule = 1
                self.action_server.publish_feedback(feedback)
                return 'violation'
            
            rospy.loginfo("Kitchen check: OK")
            return 'ok'
            
        except rospy.ServiceException as e:
            rospy.logerr(f"YOLO service error: {e}")
            return 'ok'


class CheckBedroomState(smach.State):
    """Check bedroom for rule violations (food in bedroom = rule 2)."""
    def __init__(self, action_server):
        smach.State.__init__(self, outcomes=['violation', 'ok', 'preempted'])
        self.action_server = action_server
        self.yolo_client = None
        
    def execute(self, userdata):
        rospy.loginfo("Checking bedroom for violations...")
        
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        if self.yolo_client is None:
            rospy.wait_for_service("/detect_frame", timeout=2.0)
            self.yolo_client = rospy.ServiceProxy("/detect_frame", YOLOFrame)
        
        try:
            response = self.yolo_client()
            detected_names = [d.name.lower() for d in response.detections]
            
            # Check for food items in bedroom (Rule 2)
            for food in FOOD_ITEMS:
                if food in detected_names:
                    rospy.logwarn(f"RULE 2 VIOLATION: {food} found in bedroom!")
                    feedback = CheckRulesFeedback()
                    feedback.broken_rule = 2
                    self.action_server.publish_feedback(feedback)
                    return 'violation'
            
            rospy.loginfo("Bedroom check: OK")
            return 'ok'
            
        except rospy.ServiceException as e:
            rospy.logerr(f"YOLO service error: {e}")
            return 'ok'


class CheckRulesActionServer:
    def __init__(self):
        # Don't init node here - main_node does it
        rospy.loginfo("Starting check_rules action server")
        
        self.action_server = actionlib.SimpleActionServer(
            '/check_rules',
            CheckRulesAction,
            self.execute_cb,
            auto_start=False
        )
        self.action_server.start()
        
        # Flag to check if find_object service was called
        self.object_request_received = False
        
        rospy.loginfo("CheckRules action server ready")
        
    def execute_cb(self, goal):
        """Execute the check_rules action using SMACH."""
        rospy.loginfo("CheckRules action started")
        
        # Reset flag
        self.object_request_received = False
        
        # Create SMACH state machine
        sm = smach.StateMachine(outcomes=['succeeded', 'preempted'])
        
        # Create navigation states
        nav_to_kitchen = NavigateToRoomState("kitchen", KITCHEN)
        nav_to_bedroom = NavigateToRoomState("bedroom", BEDROOM)
        check_kitchen = CheckKitchenState(self.action_server)
        check_bedroom = CheckBedroomState(self.action_server)
        
        with sm:
            # Add states for checking kitchen
            sm.add('NAV_TO_KITCHEN', nav_to_kitchen,
                   transitions={'succeeded': 'CHECK_KITCHEN', 'preempted': 'preempted'})
            sm.add('CHECK_KITCHEN', check_kitchen,
                   transitions={'violation': 'NAV_TO_BEDROOM', 'ok': 'NAV_TO_BEDROOM', 'preempted': 'preempted'})
            
            # Add states for checking bedroom
            sm.add('NAV_TO_BEDROOM', nav_to_bedroom,
                   transitions={'succeeded': 'CHECK_BEDROOM', 'preempted': 'preempted'})
            sm.add('CHECK_BEDROOM', check_bedroom,
                   transitions={'violation': 'NAV_TO_KITCHEN', 'ok': 'NAV_TO_KITCHEN', 'preempted': 'preempted'})
        
        # Execute state machine
        outcome = sm.execute()
        
        if outcome == 'preempted' or self.object_request_received:
            result = CheckRulesResult()
            self.action_server.set_preempted(result)
            rospy.loginfo("CheckRules action preempted")
        else:
            result = CheckRulesResult()
            self.action_server.set_succeeded(result)
            rospy.loginfo("CheckRules action completed")
    
    def set_object_request_flag(self):
        """Called when find_object service is requested."""
        self.object_request_received = True
        if self.action_server.is_active():
            self.action_server.set_preempted(CheckRulesResult())


if __name__ == '__main__':
    try:
        rospy.init_node('check_rules_action_server')
        rospy.loginfo("Waiting 10 seconds for localization...")
        rospy.sleep(10.0)
        server = CheckRulesActionServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

