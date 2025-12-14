#!/usr/bin/env python3
import rospy
import smach
from std_msgs.msg import String

class AnnounceState(smach.State):
    """Speaks the result using the TTS Engine."""
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'], input_keys=['object_to_find', 'found_room'])
        
        # PRIMARY: The lecture transcript says 'speech_database' listens to '/speech'
        self.speech_pub = rospy.Publisher('/speech', String, queue_size=1)
        
        # BACKUP: Some versions listen to '/tts/phrase'
        self.tts_pub = rospy.Publisher('/tts/phrase', String, queue_size=1)

    def execute(self, userdata):
        # 1. Format the sentence
        text = f"I found the {userdata.object_to_find} in room {userdata.found_room}"
        
        # 2. Print to terminal so you KNOW it reached this state
        rospy.loginfo(f"---------------------------------------------------")
        rospy.loginfo(f"[FindObject] ANNOUNCING: '{text}'")
        rospy.loginfo(f"---------------------------------------------------")
        
        msg = String()
        msg.data = text
        
        try:
            # 3. Publish multiple times to ensure the sound node catches it
            # We publish to BOTH topics to be 100% sure.
            for i in range(3):
                if rospy.is_shutdown() or self.preempt_requested(): 
                    return 'preempted'
                
                self.speech_pub.publish(msg)  # Topic from Week 10
                self.tts_pub.publish(msg)     # Common alternative
                
                rospy.sleep(1.5) # Wait for speech to finish
                
        except rospy.ROSInterruptException:
            return 'preempted'
            
        return 'succeeded'