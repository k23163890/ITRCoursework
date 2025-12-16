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

from second_coursework.msg import FindObjectAction, FindObjectResult, CheckRulesAction, CheckRulesGoal
from findObject.states.search import VerifyObjectState
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
        self.check_rules_client = actionlib.SimpleActionClient('/check_rules', CheckRulesAction)
        self.server.start()
        rospy.loginfo("[FindObject] Action Server Started")

    def execute_cb(self, goal):
        rospy.loginfo("[FindObject] Goal Received. Stopping active patrols...")
        self.check_rules_client.cancel_all_goals()
        rospy.sleep(2.0)

        target_object = goal.object_name
        rospy.loginfo(f"[FindObject] Starting Sequential Search for: {target_object}")

        sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
        sm.userdata.object_to_find = target_object
        sm.userdata.found_room = "" 

        def check_preempt():
            if self.server.is_preempt_requested():
                sm.request_preempt()
        self.server.register_preempt_callback(check_preempt)

        with sm:
            # === SEQUENCE: GO -> SEARCH -> GO -> SEARCH ===
            
            # ROOM A
            smach.StateMachine.add('GO_A', GoToRoomState('A'),
                                   transitions={'succeeded': 'SEARCH_A', 'aborted': 'GO_B', 'preempted': 'preempted'})
            smach.StateMachine.add('SEARCH_A', VerifyObjectState('A'),
                                   transitions={'found': 'GO_TO_LIVING_ROOM', 'not_found': 'GO_B', 'preempted': 'preempted'})

            # ROOM B
            smach.StateMachine.add('GO_B', GoToRoomState('B'),
                                   transitions={'succeeded': 'SEARCH_B', 'aborted': 'GO_C', 'preempted': 'preempted'})
            smach.StateMachine.add('SEARCH_B', VerifyObjectState('B'),
                                   transitions={'found': 'GO_TO_LIVING_ROOM', 'not_found': 'GO_C', 'preempted': 'preempted'})

            # ROOM C
            smach.StateMachine.add('GO_C', GoToRoomState('C'),
                                   transitions={'succeeded': 'SEARCH_C', 'aborted': 'GO_D', 'preempted': 'preempted'})
            smach.StateMachine.add('SEARCH_C', VerifyObjectState('C'),
                                   transitions={'found': 'GO_TO_LIVING_ROOM', 'not_found': 'GO_D', 'preempted': 'preempted'})

            # ROOM D
            smach.StateMachine.add('GO_D', GoToRoomState('D'),
                                   transitions={'succeeded': 'SEARCH_D', 'aborted': 'GO_E', 'preempted': 'preempted'})
            smach.StateMachine.add('SEARCH_D', VerifyObjectState('D'),
                                   transitions={'found': 'GO_TO_LIVING_ROOM', 'not_found': 'GO_E', 'preempted': 'preempted'})

            # ROOM E (Search Phase)
            smach.StateMachine.add('GO_E', GoToRoomState('E'),
                                   transitions={'succeeded': 'SEARCH_E', 'aborted': 'GO_F', 'preempted': 'preempted'})
            smach.StateMachine.add('SEARCH_E', VerifyObjectState('E'),
                                   transitions={'found': 'GO_TO_LIVING_ROOM', 'not_found': 'GO_F', 'preempted': 'preempted'})

            # ROOM F (Kitchen)
            # If navigation to F fails, we try the retry loop
            smach.StateMachine.add('GO_F', GoToRoomState('F'),
                                   transitions={'succeeded': 'SEARCH_F', 'aborted': 'GO_LOBBY_RETRY', 'preempted': 'preempted'})
            
            # If object NOT found in F, we go to Lobby (Retry State)
            smach.StateMachine.add('SEARCH_F', VerifyObjectState('F'),
                                   transitions={'found': 'GO_TO_LIVING_ROOM', 'not_found': 'GO_LOBBY_RETRY', 'preempted': 'preempted'})

            # === RETRY LOOP ===
            # Go to Lobby (Room E) because search failed, then restart at GO_A
            smach.StateMachine.add('GO_LOBBY_RETRY', GoToRoomState('E'),
                                   transitions={'succeeded': 'GO_A', 'aborted': 'GO_A', 'preempted': 'preempted'})

            # === SUCCESS DESTINATION ===
            # Go to Lobby (Room E) because object WAS found, then Announce
            smach.StateMachine.add('GO_TO_LIVING_ROOM', GoToRoomState('E'),
                                   transitions={'succeeded': 'ANNOUNCE', 'aborted': 'ANNOUNCE', 'preempted': 'preempted'})

            # ANNOUNCE
            smach.StateMachine.add('ANNOUNCE', AnnounceState(),
                                   transitions={'succeeded': 'succeeded'})

        sis = smach_ros.IntrospectionServer('find_object_server', sm, '/FIND_OBJECT_SM')
        sis.start()
        outcome = sm.execute()
        sis.stop()
        
        res = FindObjectResult()
        if outcome == 'succeeded':
            res.success = True
            res.object_found = target_object
            self.server.set_succeeded(res)


            rospy.loginfo("[FindObject] Mission Complete. Automatically resuming Rule Checking Patrol...")
            
            # Send a new goal to the CheckRules Action Server
            patrol_goal = CheckRulesGoal()
            self.check_rules_client.send_goal(patrol_goal)


        elif outcome == 'preempted':
            self.server.set_preempted(res)
        else:
            res.success = False
            self.server.set_aborted(res)

if __name__ == '__main__':
    rospy.init_node('find_object_action_server')
    server = FindObjectServer()
    rospy.spin()