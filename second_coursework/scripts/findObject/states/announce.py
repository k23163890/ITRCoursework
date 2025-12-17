#!/usr/bin/env python3
import rospy
import smach
from std_msgs.msg import String

class AnnounceState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'], input_keys=['object_to_find', 'found_room'])
        self.tts_pub = rospy.Publisher('/tts/phrase', String, queue_size=1)

    def execute(self, userdata):
        if userdata.found_room:
            room = userdata.found_room
        else:
            room = "unknown"
            
        text = f"I found the {userdata.object_to_find} in room {room}"
        rospy.loginfo(text)
        msg = String()
        msg.data = text
        self.tts_pub.publish(msg)
        
        rospy.sleep(5.0) 
        return 'succeeded'