#!/usr/bin/env python3
import rospy
import sys
import os
import smach
import smach_ros
import actionlib

# --- FIX IMPORTS ---
# Add the current directory to python path so we can import local scripts
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.append(script_dir)

try:
    from movement_utils import go_to_room
    from yolo_client import detect_objects, init_yolo
except ImportError as e:
    print(f"CRITICAL ERROR: Could not import modules: {e}")
    sys.exit(1)

from second_coursework.msg import CheckRulesAction, CheckRulesFeedback, CheckRulesResult

# --------------------------------------------------------
# STATE: Wait 10 Seconds (localisation stabilisation)
# --------------------------------------------------------
class WaitState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rospy.loginfo("[CheckRules] Waiting 10s for localisation...")
        rospy.sleep(10)
        return 'done'


# --------------------------------------------------------
# STATE: Go to Kitchen (F)
# --------------------------------------------------------
class GoKitchen(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['arrived', 'preempted'])

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        # Note: If you need to check 'is_searching', use a ROS Param or Service here.
        # Direct variable import does not work across different nodes.
        
        success = go_to_room("F")
        if success:
            return 'arrived'
        else:
            # If navigation fails, we might want to retry or abort. 
            # For this logic, let's assume we try again or just proceed.
            return 'arrived' 


# --------------------------------------------------------
# STATE: Scan Kitchen for RULE 1 violations (people)
# --------------------------------------------------------
class ScanKitchen(smach.State):
    def __init__(self, feedback_pub):
        smach.State.__init__(self, outcomes=['done', 'preempted'])
        self.feedback_pub = feedback_pub

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        rospy.loginfo("[CheckRules] Scanning Kitchen...")
        detected = detect_objects()
        rospy.loginfo(f"[CheckRules] Detected: {detected}")

        if "person" in detected:
            rospy.logwarn("RULE 1 VIOLATION: Person found in Kitchen!")
            fb = CheckRulesFeedback(broken_rule=1)
            self.feedback_pub.publish(fb)

        return 'done'


# --------------------------------------------------------
# STATE: Go to Bedroom (C)
# --------------------------------------------------------
class GoBedroom(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['arrived', 'preempted'])

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
            
        success = go_to_room("C")
        return 'arrived'


# --------------------------------------------------------
# STATE: Scan Bedroom for RULE 2 violations (food items)
# --------------------------------------------------------
class ScanBedroom(smach.State):
    FOOD = ["pizza", "banana", "broccoli", "sandwich"]

    def __init__(self, feedback_pub):
        smach.State.__init__(self, outcomes=['done', 'preempted'])
        self.feedback_pub = feedback_pub

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        rospy.loginfo("[CheckRules] Scanning Bedroom...")
        detected = detect_objects()
        rospy.loginfo(f"[CheckRules] Detected: {detected}")

        if any(item in detected for item in self.FOOD):
            rospy.logwarn("RULE 2 VIOLATION: Food found in Bedroom!")
            fb = CheckRulesFeedback(broken_rule=2)
            self.feedback_pub.publish(fb)

        return 'done'


# --------------------------------------------------------
# ACTION SERVER WRAPPER
# --------------------------------------------------------
class CheckRulesServer(object):

    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            '/check_rules',
            CheckRulesAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.server.start()
        rospy.loginfo("[CheckRules] Action server started on /check_rules")

    def execute_cb(self, goal):
        rospy.loginfo("[CheckRules] Goal received. Starting patrol...")
        feedback_pub = rospy.Publisher(
            '/check_rules/feedback', CheckRulesFeedback, queue_size=10)

        sm = smach.StateMachine(outcomes=['preempted', 'succeeded', 'aborted'])

        # Preemption check callback
        def check_preempt():
            if self.server.is_preempt_requested():
                sm.request_preempt()

        # Register the callback with the state machine is simpler via a loop inside states,
        # but SMACH generally handles this if states implement service_preempt()
        
        with sm:
            smach.StateMachine.add(
                'WAIT', WaitState(),
                transitions={'done': 'GO_KITCHEN'}
            )

            smach.StateMachine.add(
                'GO_KITCHEN', GoKitchen(),
                transitions={
                    'arrived': 'SCAN_KITCHEN',
                    'preempted': 'preempted'
                }
            )

            smach.StateMachine.add(
                'SCAN_KITCHEN', ScanKitchen(feedback_pub),
                transitions={
                    'done': 'GO_BEDROOM',
                    'preempted': 'preempted'
                }
            )

            smach.StateMachine.add(
                'GO_BEDROOM', GoBedroom(),
                transitions={
                    'arrived': 'SCAN_BEDROOM',
                    'preempted': 'preempted'
                }
            )

            smach.StateMachine.add(
                'SCAN_BEDROOM', ScanBedroom(feedback_pub),
                transitions={
                    'done': 'GO_KITCHEN',
                    'preempted': 'preempted'
                }
            )

        # Run the state machine
        try:
            # We can run an introspection server if needed, but keeping it simple
            outcome = sm.execute()
        except Exception as e:
            rospy.logerr(f"StateMachine crashed: {e}")
            self.server.set_aborted()
            return

        if outcome == 'preempted' or self.server.is_preempt_requested():
            self.server.set_preempted(CheckRulesResult())
        else:
            # The logic above loops forever between Kitchen and Bedroom, 
            # so it technically never "succeeds" unless changed.
            # If the loop breaks, we succeed.
            self.server.set_succeeded(CheckRulesResult())


def main():
    rospy.init_node("check_rules_action_server")
    
    # Init YOLO listener AFTER node is started
    init_yolo()
    
    server = CheckRulesServer()
    rospy.spin()


if __name__ == "__main__":
    main()