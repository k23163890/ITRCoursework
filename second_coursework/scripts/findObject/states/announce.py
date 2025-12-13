#!/usr/bin/env python3
import rospy
import smach
from std_msgs.msg import String

class AnnounceState(smach.State):
    """Speaks the result using the Speech Database (Google TTS)."""
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['object_to_find', 'found_room'])
        
        # Topic '/speech' is used by the hmi_speech_database node (Better quality)
        # Topic '/tts/phrase' is used by the tts_engine node (Robotic voice)
        self.tts_pub = rospy.Publisher('/speech', String, queue_size=1)

    def execute(self, userdata):
        # Format the sentence as required: Room + Object Name
        text = f"I found the {userdata.object_to_find} in room {userdata.found_room}"
        rospy.loginfo(f"[FindObject] Announcing: {text}")
        
        msg = String()
        msg.data = text
        
        # Publish multiple times to ensure the node catches it
        for _ in range(3):
            if rospy.is_shutdown(): break
            self.tts_pub.publish(msg)
            rospy.sleep(1.0) # Wait 1 second between repeats
            
        return 'succeeded'