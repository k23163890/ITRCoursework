#!/usr/bin/env python3
import rospy
import smach
import threading
from geometry_msgs.msg import Twist
from second_coursework.srv import YOLOFrame

class VerifyObjectState(smach.State):
    def __init__(self, room_name):
        smach.State.__init__(self, 
                             outcomes=['found', 'not_found', 'preempted'],
                             input_keys=['object_to_find'],
                             output_keys=['found_room'])
        self.room_name = room_name
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.keep_spinning = False

    def spin_robot_thread(self):
        """Background task to keep publishing velocity."""
        spin_cmd = Twist()
        spin_cmd.angular.z = 0.8
        
        rate = rospy.Rate(10)
        while self.keep_spinning and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(spin_cmd)
            rate.sleep()

    def execute(self, userdata):
        target = userdata.object_to_find.lower().strip()
        rospy.loginfo(f"\n>>> [FindObject] ARRIVED IN ROOM {self.room_name}")
        rospy.sleep(2.0)
        
        rospy.loginfo(f">>> [FindObject] SEARCHING FOR TARGET: '{target}' in {self.room_name}")

        timeout = 40
        start_time = rospy.Time.now()

        try:
            rospy.wait_for_service('/detect_frame', timeout=5.0)
            detect_service = rospy.ServiceProxy('/detect_frame', YOLOFrame)
        except (rospy.ROSInterruptException, rospy.ROSException) as e:
            rospy.logwarn(f"[FindObject] Service failed: {e}")
            return 'not_found'

        self.keep_spinning = True
        spinner = threading.Thread(target=self.spin_robot_thread)
        spinner.start()

        outcome = 'not_found'

        try:
            while (rospy.Time.now() - start_time).to_sec() < timeout:
                if self.preempt_requested() or rospy.is_shutdown():
                    outcome = 'preempted'
                    break

                try:
                    response = detect_service()
                    detections = [d.name.lower().strip() for d in response.detections]
                    
                    if target in detections:
                        rospy.loginfo(f">>> [FindObject] MATCH FOUND! '{target}' detected in Room {self.room_name}")
                        userdata.found_room = self.room_name
                        outcome = 'found'
                        break

                except Exception as e:
                    rospy.logwarn(f"[FindObject] Detection Error: {e}")
                
                rospy.sleep(0.1)

        finally:
            self.keep_spinning = False
            spinner.join()
            
            self.cmd_vel_pub.publish(Twist())

        return outcome