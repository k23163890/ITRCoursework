#!/usr/bin/env python3
import rospy
import smach
from std_msgs.msg import String

class AnnounceState(smach.State):
    """Speaks the result using the TTS Engine (eSpeak/Robotic voice)."""
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['object_to_find', 'found_room'])
        
        # CHANGED: Now publishing to '/tts/phrase' instead of '/speech'
        self.tts_pub = rospy.Publisher('/tts/phrase', String, queue_size=1)

    def execute(self, userdata):
        # Format the sentence as required: Room + Object Name
        text = f"I found the {userdata.object_to_find} in room {userdata.found_room}"
        rospy.loginfo(f"[FindObject] Announcing via /tts/phrase: {text}")
        
        msg = String()
        msg.data = text
        
        # Publish multiple times to ensure the node catches it
        for _ in range(3):
            if rospy.is_shutdown(): break
            self.tts_pub.publish(msg)
            rospy.sleep(1.0) # Wait 1 second between repeats
            
        return 'succeeded'
    
    #Make sure to test speech recog again - used /speech previously and need to use tts_engine