#!/usr/bin/env python3
import rospy
import actionlib
import smach
import smach_ros
import sys
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.append(script_dir)

from second_coursework.msg import CheckRulesAction, CheckRulesResult
from rules.states.navigation import WaitState, GoToRoomState
from rules.states.detection import CheckRoomState

class CheckRulesServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            '/check_rules', 
            CheckRulesAction, 
            execute_cb=self.execute_cb, 
            auto_start=False
        )
        self.server.start()
        rospy.loginfo("[CheckRules] Action Server Started")

    def execute_cb(self, goal):
        rospy.loginfo("[CheckRules] Goal Received. Starting Patrol.")
        
        sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])

        # 1. Handle Action Client Preemption (e.g., when FindObject starts)
        def check_preempt():
            if self.server.is_preempt_requested():
                sm.request_preempt()
        self.server.register_preempt_callback(check_preempt)

        # 2. NEW: Handle ROS Shutdown (Ctrl+C) Instantly
        def shutdown_hook():
            rospy.logwarn("[CheckRules] Ctrl+C detected! Preempting State Machine...")
            sm.request_preempt() # This forces the current state to return 'preempted'
        rospy.on_shutdown(shutdown_hook)

        with sm:
            # 1. Wait (Start)
            smach.StateMachine.add('WAIT_INIT', WaitState(8.7), 
                                   transitions={'succeeded': 'GO_KITCHEN', 'preempted': 'preempted'})

            # 2. Go to Kitchen
            smach.StateMachine.add('GO_KITCHEN', GoToRoomState('F'),
                                   transitions={'succeeded': 'CHECK_KITCHEN', 
                                                'aborted': 'RETRY_WAIT_F', 
                                                'preempted': 'preempted'})
            
            # Retry Wait State
            smach.StateMachine.add('RETRY_WAIT_F', WaitState(2.0),
                                   transitions={'succeeded': 'GO_KITCHEN', 'preempted': 'preempted'})

            # 3. Check Kitchen
            smach.StateMachine.add('CHECK_KITCHEN', CheckRoomState(self.server, rule_type=1),
                                   transitions={'checked': 'GO_BEDROOM', 
                                                'violation': 'GO_BEDROOM', 
                                                'preempted': 'preempted'})

            # 4. Go to Bedroom
            smach.StateMachine.add('GO_BEDROOM', GoToRoomState('C'),
                                   transitions={'succeeded': 'CHECK_BEDROOM', 
                                                'aborted': 'RETRY_WAIT_C', 
                                                'preempted': 'preempted'})

            # Retry Wait State
            smach.StateMachine.add('RETRY_WAIT_C', WaitState(2.0),
                                   transitions={'succeeded': 'GO_BEDROOM', 'preempted': 'preempted'})

            # 5. Check Bedroom
            smach.StateMachine.add('CHECK_BEDROOM', CheckRoomState(self.server, rule_type=2),
                                   transitions={'checked': 'GO_KITCHEN', 
                                                'violation': 'GO_KITCHEN', 
                                                'preempted': 'preempted'})

        sis = smach_ros.IntrospectionServer('check_rules_server', sm, '/CHECK_RULES_SM')
        sis.start()
        outcome = sm.execute()
        sis.stop()

        result = CheckRulesResult()
        if outcome == 'preempted':
            self.server.set_preempted(result)
        elif outcome == 'succeeded':
            self.server.set_succeeded(result)
        else:
            self.server.set_aborted(result)

if __name__ == '__main__':
    rospy.init_node('check_rules_action_server')
    server = CheckRulesServer()
    rospy.spin()