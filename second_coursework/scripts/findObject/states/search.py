#!/usr/bin/env python3
import rospy
import smach
from second_coursework.srv import YOLOFrame

class SelectNextRoomState(smach.State):
    """Iterates through rooms A-F to set the next target."""
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['next_room', 'all_visited', 'preempted'],
                             output_keys=['target_room'])
        self.rooms = ['A', 'B', 'C', 'D', 'E', 'F']
        self.current_idx = 0

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        if self.current_idx < len(self.rooms):
            room = self.rooms[self.current_idx]
            userdata.target_room = room
            rospy.loginfo(f"--- [FindObject] DEBUG: Selected next room: {room} ---")
            self.current_idx += 1
            return 'next_room'
        else:
            return 'all_visited'


class VerifyObjectState(smach.State):
    """
    Checks if the requested object is in the current view.
    It watches for up to 15 seconds to allow the video playlist to cycle.
    """
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['found', 'not_found', 'preempted'],
                             input_keys=['object_to_find', 'target_room'],
                             output_keys=['found_room'])

    def execute(self, userdata):
        # 1. Clean up the target string
        target = userdata.object_to_find.lower().strip()
        
        rospy.loginfo(f"\n>>> [FindObject] ARRIVED IN ROOM {userdata.target_room}")
        rospy.loginfo(f">>> [FindObject] SEARCHING FOR TARGET: '{target}' (Watching for 15s...)")

        # 2. Setup the timer
        timeout = 15.0  # How long to wait for the object to appear
        start_time = rospy.Time.now()
        
        # 3. Wait for service
        try:
            rospy.wait_for_service('/detect_frame', timeout=5.0)
            detect_service = rospy.ServiceProxy('/detect_frame', YOLOFrame)
        except Exception as e:
            rospy.logwarn(f"[FindObject] Service connection failed: {e}")
            return 'not_found'

        # 4. Start Polling Loop
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            
            # Check for Preemption (in case you want to stop it manually)
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'

            try:
                # Call the service (Take a snapshot)
                response = detect_service()
                detections = [d.name.lower().strip() for d in response.detections]
                
                # Check if our target is in the list
                if target in detections:
                    rospy.loginfo(f">>> [FindObject] MATCH FOUND! '{target}' detected in {detections}")
                    rospy.loginfo(f">>> [FindObject] Transitioning to GO_TO_LIVING_ROOM...\n")
                    userdata.found_room = userdata.target_room
                    return 'found'
                
                # Optional: Log what we see periodically (helps debugging but can be spammy)
                # rospy.loginfo(f">>> [FindObject] Saw: {detections}")

            except Exception as e:
                rospy.logwarn(f"[FindObject] Snapshot failed: {e}")

            # Wait a bit before checking again
            rospy.sleep(0.5)

        # 5. Timeout reached - Object not found in this room
        rospy.logwarn(f">>> [FindObject] TIMEOUT. '{target}' was NOT seen in Room {userdata.target_room} after {timeout}s.")
        return 'not_found'