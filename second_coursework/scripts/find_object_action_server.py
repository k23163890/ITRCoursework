#!/usr/bin/env python3
import rospy
import actionlib
import smach
import sys
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.append(script_dir)

# Import CheckRulesAction so we can cancel it
from second_coursework.msg import FindObjectAction, FindObjectResult, CheckRulesAction
from findObject.states.search import SelectNextRoomState, VerifyObjectState
from findObject.states.announce import AnnounceState
from rules.states.navigation import GoToRoomState 

class FindObjectServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            '/find_object_action', 
            FindObjectAction, 
            execute_cb=self.execute_cb, 
            auto_start=False
        )
        
        # Client to control the patrolling behavior
        self.check_rules_client = actionlib.SimpleActionClient('/check_rules', CheckRulesAction)
        
        self.server.start()
        rospy.loginfo("[FindObject] Action Server Started")

    def execute_cb(self, goal):
        rospy.loginfo("[FindObject] Goal Received. Stopping any active patrols...")
        
        # 1. Stop the Rule Checker (Patrol)
        # This prevents the "fighting" for move_base control
        self.check_rules_client.cancel_all_goals()
        
        # 2. Wait briefly to ensure the robot stops and the other node handles the cancel
        rospy.sleep(2.0)

        target_object = goal.object_name
        rospy.loginfo(f"[FindObject] Searching for: {target_object}")

        sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
        
        sm.userdata.object_to_find = target_object
        sm.userdata.found_room = "" 
        sm.userdata.target_room = ""

        def check_preempt():
            if self.server.is_preempt_requested():
                sm.request_preempt()
        self.server.register_preempt_callback(check_preempt)

        with sm:
            # 1. Select Next Room (Iterates A->F)
            smach.StateMachine.add('SELECT_ROOM', SelectNextRoomState(),
                                   transitions={'next_room': 'GO_TO_ROOM', 
                                                'all_visited': 'aborted', 
                                                'preempted': 'preempted'})

            # 2. Navigate
            smach.StateMachine.add('GO_TO_ROOM', GoToRoomState(), 
                                   transitions={'arrived': 'LOOK_FOR_OBJECT',
                                                'failed': 'SELECT_ROOM', 
                                                'preempted': 'preempted'})

            # 3. Look for Object
            smach.StateMachine.add('LOOK_FOR_OBJECT', VerifyObjectState(),
                                   transitions={'found': 'GO_TO_LIVING_ROOM', 
                                                'not_found': 'SELECT_ROOM',
                                                'preempted': 'preempted'})

            # 4. Success? Go to Living Room (E)
            smach.StateMachine.add('GO_TO_LIVING_ROOM', GoToRoomState('E'),
                                   transitions={'arrived': 'ANNOUNCE', 
                                                'failed': 'ANNOUNCE', 
                                                'preempted': 'preempted'})

            # 5. Announce Result
            smach.StateMachine.add('ANNOUNCE', AnnounceState(),
                                   transitions={'succeeded': 'succeeded'})

        outcome = sm.execute()
        
        res = FindObjectResult()
        if outcome == 'succeeded':
            res.success = True
            self.server.set_succeeded(res)
        elif outcome == 'preempted':
            self.server.set_preempted(res)
        else:
            res.success = False
            self.server.set_aborted(res)

if __name__ == '__main__':
    rospy.init_node('find_object_action_server')
    server = FindObjectServer()
    rospy.spin()