#!/usr/bin/env python3
# coding=utf-8

import rospy
import smach
import smach_ros
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from second_coursework.msg import FindObjectAction, FindObjectGoal, FindObjectResult
from YOLOObjectDetection.srv import YOLOFrame

# Room coordinates - need to be mapped to A, B, C, D, E, F
# Living room is Room E
LIVING_ROOM = {"x": 10.4, "y": 8.9, "w": 1.57}

# Room labels mapping (x, y coordinates to room letter)
# These need to be determined from the map - using approximate positions
ROOM_COORDS = {
    (10.4, 8.9): 'E',  # Living room (Room E)
    (10.7, 4.3): 'A',  # Kitchen (example)
    (2.0, 8.5): 'B',   # Bedroom (example)
    # Add more room mappings as needed
}

def get_room_label(x, y):
    """Determine room label from coordinates."""
    # Find closest room
    for (rx, ry), label in ROOM_COORDS.items():
        if abs(x - rx) < 2.0 and abs(y - ry) < 2.0:
            return label
    return 'unknown'


class NavigateToRoomState(smach.State):
    """Navigate to living room."""
    def __init__(self, room_coords):
        smach.State.__init__(self, outcomes=['succeeded', 'preempted'])
        self.room_coords = room_coords
        self.move_base_client = None
        
    def execute(self, userdata):
        rospy.loginfo("Navigating to living room (Room E)...")
        
        if self.move_base_client is None:
            self.move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
            if not self.move_base_client.wait_for_server(timeout=rospy.Duration(5.0)):
                rospy.logerr("move_base server not available")
                return 'succeeded'  # Continue anyway
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.room_coords["x"]
        goal.target_pose.pose.position.y = self.room_coords["y"]
        goal.target_pose.pose.orientation.w = self.room_coords["w"]
        
        self.move_base_client.send_goal(goal)
        
        while not rospy.is_shutdown():
            if self.preempt_requested():
                self.move_base_client.cancel_all_goals()
                self.service_preempt()
                return 'preempted'
            
            state = self.move_base_client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Arrived at living room")
                return 'succeeded'
            elif state in [actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.REJECTED]:
                rospy.logwarn("Failed to reach living room, continuing anyway")
                return 'succeeded'
            
            rospy.sleep(0.1)
        
        return 'preempted'


class SearchObjectState(smach.State):
    """Search for the target object using YOLO."""
    def __init__(self, object_name):
        smach.State.__init__(self, 
                            outcomes=['found', 'not_found', 'preempted'],
                            output_keys=['room_label', 'object_name', 'robot_x', 'robot_y'])
        self.object_name = object_name.lower()
        self.yolo_client = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.odom_sub = None
        
    def odom_callback(self, msg):
        """Store current robot position."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
    def execute(self, userdata):
        rospy.loginfo(f"Searching for object: {self.object_name}")
        
        # Subscribe to odometry to track position
        if self.odom_sub is None:
            self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
            rospy.sleep(0.5)  # Give time to get first odom message
        
        if self.preempt_requested():
            self.service_preempt()
            if self.odom_sub:
                self.odom_sub.unregister()
            return 'preempted'
        
        if self.yolo_client is None:
            try:
                rospy.wait_for_service("/detect_frame", timeout=5.0)
                self.yolo_client = rospy.ServiceProxy("/detect_frame", YOLOFrame)
            except rospy.ROSException:
                rospy.logerr("YOLO /detect_frame service not available!")
                return 'not_found'  # Return not_found if service unavailable
        
        try:
            response = self.yolo_client()
            detected_names = [d.name.lower() for d in response.detections]
            
            # Check if object is found
            if self.object_name in detected_names:
                rospy.loginfo(f"Found {self.object_name}!")
                userdata.object_name = self.object_name
                userdata.robot_x = self.robot_x
                userdata.robot_y = self.robot_y
                # Determine room from position
                room_label = get_room_label(self.robot_x, self.robot_y)
                userdata.room_label = room_label
                rospy.loginfo(f"Object found at position ({self.robot_x:.2f}, {self.robot_y:.2f}) in Room {room_label}")
                if self.odom_sub:
                    self.odom_sub.unregister()
                return 'found'
            
            return 'not_found'
            
        except rospy.ServiceException as e:
            rospy.logerr(f"YOLO service error: {e}")
            return 'not_found'


class AnnounceState(smach.State):
    """Announce the found object using TTS."""
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        # Try common TTS topics
        self.tts_pub = None
        
    def execute(self, userdata):
        room = userdata.room_label
        obj_name = userdata.object_name
        
        message = f"I found {obj_name} in room {room}"
        rospy.loginfo(f"Announcing: {message}")
        
        # Try publishing to common TTS topics
        if self.tts_pub is None:
            # Try /tts or /speech or /say topic
            try:
                self.tts_pub = rospy.Publisher('/tts', String, queue_size=1)
                rospy.sleep(0.5)
            except:
                try:
                    self.tts_pub = rospy.Publisher('/speech', String, queue_size=1)
                    rospy.sleep(0.5)
                except:
                    try:
                        self.tts_pub = rospy.Publisher('/say', String, queue_size=1)
                        rospy.sleep(0.5)
                    except:
                        pass
        
        if self.tts_pub:
            msg = String()
            msg.data = message
            self.tts_pub.publish(msg)
            rospy.loginfo(f"Published TTS message: {message}")
            rospy.sleep(1.0)  # Give time for TTS to process
        else:
            rospy.logwarn("TTS publisher not available, just logging message")
        
        return 'succeeded'


class FindObjectActionServer:
    def __init__(self):
        # Don't init node here - main_node does it
        rospy.loginfo("Starting find_object_action server")
        
        self.action_server = actionlib.SimpleActionServer(
            '/find_object_action',
            FindObjectAction,
            self.execute_cb,
            auto_start=False
        )
        self.action_server.start()
        
        rospy.loginfo("FindObject action server ready")
        
    def execute_cb(self, goal):
        """Execute the find_object_action using SMACH."""
        object_name = goal.object_name
        rospy.loginfo(f"FindObject action started for: {object_name}")
        
        # Create SMACH state machine
        sm = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted'])
        
        # Userdata to pass room and object info
        sm.userdata.room_label = ''
        sm.userdata.object_name = object_name
        sm.userdata.robot_x = 0.0
        sm.userdata.robot_y = 0.0
        
        with sm:
            # Search for object
            sm.add('SEARCH_OBJECT', 
                   SearchObjectState(object_name),
                   transitions={'found': 'NAV_TO_LIVING_ROOM',
                              'not_found': 'SEARCH_OBJECT',
                              'preempted': 'preempted'})
            
            # Navigate to living room
            sm.add('NAV_TO_LIVING_ROOM',
                   NavigateToRoomState(LIVING_ROOM),
                   transitions={'succeeded': 'ANNOUNCE',
                              'preempted': 'preempted'})
            
            # Announce the finding
            sm.add('ANNOUNCE',
                   AnnounceState(),
                   transitions={'succeeded': 'succeeded'})
        
        # Execute state machine
        outcome = sm.execute()
        
        # Set result
        result = FindObjectResult()
        if outcome == 'succeeded':
            result.success = True
            result.room_found = sm.userdata.room_label
            result.object_found = sm.userdata.object_name
            self.action_server.set_succeeded(result)
            rospy.loginfo(f"FindObject action completed: Found {result.object_found} in Room {result.room_found}")
        elif outcome == 'preempted':
            self.action_server.set_preempted(result)
            rospy.loginfo("FindObject action preempted")
        else:
            result.success = False
            self.action_server.set_aborted(result)
            rospy.loginfo("FindObject action aborted")


if __name__ == '__main__':
    try:
        rospy.init_node('find_object_action_server')
        rospy.loginfo("Waiting 10 seconds for localization...")
        rospy.sleep(10.0)
        server = FindObjectActionServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

