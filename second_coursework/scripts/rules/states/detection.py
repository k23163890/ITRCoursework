#!/usr/bin/env python3
import rospy
import smach
from geometry_msgs.msg import Twist
from second_coursework.msg import CheckRulesFeedback
from second_coursework.srv import YOLOFrame 

class CheckRoomState(smach.State):
    def __init__(self, action_server, rule_type):
        smach.State.__init__(self, outcomes=['checked', 'violation', 'preempted'])
        self.action_server = action_server
        self.rule_type = rule_type
        self.food_items = ['pizza', 'sandwich', 'banana', 'broccoli']
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)



    def execute(self, userdata):
        rospy.loginfo(f"[CheckRules] Arrived. Scanning room (Rule {self.rule_type})...")

        spin_cmd = Twist()
        spin_cmd.angular.z = 1.57
        end_time = rospy.Time.now() + rospy.Duration(15.0)
        try:
            while rospy.Time.now() < end_time:
                if self.preempt_requested() or rospy.is_shutdown():
                    self.service_preempt()
                    return 'preempted'
                self.cmd_vel_pub.publish(spin_cmd)
                rospy.sleep(0.1)
            self.cmd_vel_pub.publish(Twist())
            rospy.sleep(0.5)

            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'

            detections = []
            
            rospy.wait_for_service('/detect_frame', timeout=2.0)
            detect_service = rospy.ServiceProxy('/detect_frame', YOLOFrame)
            response = detect_service()
            detections = [d.name.lower() for d in response.detections]
            rospy.loginfo(f"[CheckRules] YOLO Detected: {detections}")

        except rospy.ROSInterruptException:
            return 'preempted'
            
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logwarn(f"[CheckRules] Detection failed: {e}")
            return 'checked'
        violation_found = False
        if self.rule_type == 1 and 'person' in detections:
            violation_found = True
            rospy.logwarn("RULE 1 VIOLATION: Person detected in Kitchen")
        elif self.rule_type == 2:
            for item in detections:
                if item in self.food_items:
                    violation_found = True
                    rospy.logwarn(f"RULE 2 VIOLATION: {item} detected in Bedroom")
                    break




                

        if violation_found:
            if self.action_server:
                feedback = CheckRulesFeedback()
                feedback.broken_rule = self.rule_type
                self.action_server.publish_feedback(feedback)
            return 'violation'
        
        return 'checked'