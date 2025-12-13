#!/usr/bin/env python3
import rospy
import actionlib
import smach
import smach_ros
import sys
import os

# Ensure we can import local packages from current directory
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.append(script_dir)

from second_coursework.msg import CheckRulesAction, CheckRulesResult
# Import modular states
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

        # Preemption Callback
        def check_preempt():
            if self.server.is_preempt_requested():
                sm.request_preempt()

        self.server.register_preempt_callback(check_preempt)

        with sm:
            # 1. Initial Wait (10s for localization)
            smach.StateMachine.add('WAIT_INIT', WaitState(10.0), 
                                   transitions={'succeeded': 'GO_KITCHEN', 'preempted': 'preempted'})

            # 2. Go To Kitchen (F)
            smach.StateMachine.add('GO_KITCHEN', GoToRoomState('F'),
                                   transitions={'arrived': 'CHECK_KITCHEN', 
                                                'failed': 'GO_KITCHEN', # Retry loop if nav fails
                                                'preempted': 'preempted'})

            # 3. Check Kitchen (Rule 1)
            smach.StateMachine.add('CHECK_KITCHEN', CheckRoomState(self.server, rule_type=1),
                                   transitions={'checked': 'GO_BEDROOM', 
                                                'violation': 'GO_BEDROOM',
                                                'preempted': 'preempted'})

            # 4. Go To Bedroom (C)
            smach.StateMachine.add('GO_BEDROOM', GoToRoomState('C'),
                                   transitions={'arrived': 'CHECK_BEDROOM', 
                                                'failed': 'GO_BEDROOM',
                                                'preempted': 'preempted'})

            # 5. Check Bedroom (Rule 2) - Loops back to Kitchen
            smach.StateMachine.add('CHECK_BEDROOM', CheckRoomState(self.server, rule_type=2),
                                   transitions={'checked': 'GO_KITCHEN', 
                                                'violation': 'GO_KITCHEN',
                                                'preempted': 'preempted'})

        outcome = sm.execute()

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