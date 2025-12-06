#!/usr/bin/env python3
import rospy
import smach
import smach_ros
import actionlib

from second_coursework.msg import CheckRulesAction, CheckRulesFeedback, CheckRulesResult
from movement_utils import go_to_room
from yolo_client import detect_objects

# READ the state from your service file
from find_object_service import is_searching
from yolo_client import init_yolo
init_yolo()



# --------------------------------------------------------
# STATE: Wait 10 Seconds (localisation stabilisation)
# --------------------------------------------------------
class WaitState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rospy.sleep(10)
        return 'done'


# --------------------------------------------------------
# STATE: Go to Kitchen (F)
# --------------------------------------------------------
class GoKitchen(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['arrived', 'preempted'])

    def execute(self, userdata):
        if is_searching:
            return 'preempted'
        go_to_room("F")
        return 'arrived'


# --------------------------------------------------------
# STATE: Scan Kitchen for RULE 1 violations (people)
# --------------------------------------------------------
class ScanKitchen(smach.State):
    def __init__(self, feedback_pub):
        smach.State.__init__(self, outcomes=['done', 'preempted'])
        self.feedback_pub = feedback_pub


    

    def execute(self, userdata):
        if is_searching:
            return 'preempted'

        detected = detect_objects()

        if "person" in detected:
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
        if is_searching:
            return 'preempted'
        go_to_room("C")
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
        if is_searching:
            return 'preempted'

        detected = detect_objects()

        if any(item in detected for item in self.FOOD):
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
        rospy.loginfo("[CheckRules] Action server started")

    def execute_cb(self, goal):

        feedback_pub = rospy.Publisher(
            '/check_rules/feedback', CheckRulesFeedback, queue_size=10)

        sm = smach.StateMachine(outcomes=['preempted'])

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

        result = sm.execute()

        if result == 'preempted':
            self.server.set_preempted(CheckRulesResult())
        else:
            self.server.set_succeeded(CheckRulesResult())


def main():
    rospy.init_node("check_rules_action_server")
    CheckRulesServer()
    rospy.spin()


if __name__ == "__main__":
    main()
