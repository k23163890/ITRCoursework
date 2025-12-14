#!/usr/bin/env python3
import rospy
import smach
from second_coursework.srv import YOLOFrame

class VerifyObjectState(smach.State):
    """
    Checks if the requested object is in the specific room provided in __init__.
    """
    def __init__(self, room_name):
        smach.State.__init__(self, 
                             outcomes=['found', 'not_found', 'preempted'],
                             input_keys=['object_to_find'],
                             output_keys=['found_room'])
        self.room_name = room_name

    def execute(self, userdata):
        target = userdata.object_to_find.lower().strip()
        rospy.loginfo(f"\n>>> [FindObject] ARRIVED IN ROOM {self.room_name}")
        rospy.loginfo(f">>> [FindObject] SEARCHING FOR TARGET: '{target}' in {self.room_name}")

        timeout = 15.0 
        start_time = rospy.Time.now()
        
        try:
            rospy.wait_for_service('/detect_frame', timeout=5.0)
            detect_service = rospy.ServiceProxy('/detect_frame', YOLOFrame)
        except rospy.ROSInterruptException:
            return 'preempted'
        except Exception as e:
            rospy.logwarn(f"[FindObject] Service connection failed: {e}")
            return 'not_found'

        # Polling Loop
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            if self.preempt_requested() or rospy.is_shutdown():
                self.service_preempt()
                return 'preempted'

            try:
                response = detect_service()
                detections = [d.name.lower().strip() for d in response.detections]
                
                if target in detections:
                    rospy.loginfo(f">>> [FindObject] MATCH FOUND! '{target}' detected in Room {self.room_name}")
                    userdata.found_room = self.room_name
                    return 'found'

            except rospy.ROSInterruptException:
                return 'preempted'
            except Exception as e:
                rospy.logwarn(f"[FindObject] Snapshot failed: {e}")

            rospy.sleep(0.5)

        rospy.loginfo(f">>> [FindObject] '{target}' NOT found in Room {self.room_name}.")
        return 'not_found'