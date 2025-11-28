#!/usr/bin/env python

import rospy
import smach
import smach_ros
from geometry_msgs.msg import Twist, Vector3
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty, EmptyResponse
from sensor_msgs.msg import LaserScan
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Empty as EmptyMsg

# --- Global Variables for Event Simulation (Triggered by Services) ---
LOW_BATTERY = False
DIRT_FOUND = False
OBSTACLE_FOUND = False

# Global variable to store laser scan data
LASER_DATA = None

def laser_callback(data):
    """Callback to update laser scan data"""
    global LASER_DATA
    LASER_DATA = data

# --- 1. Custom State for MOVING (Main Cleaning & Event Monitor) ---
class MovingState(smach.State):
    """The robot's primary cleaning state. Monitors for high-priority events."""
    def __init__(self):
        smach.State.__init__(self, outcomes=['obstacle', 'low_battery', 'dirt', 'succeeded'])
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.move_duration = 5.0  # Time to spend moving before checking
        self.linear_speed = 0.3  # Reduced speed for better control
        self.angular_speed = 0.1  # Slight curve for coverage

    def execute(self, userdata):
        rospy.loginfo('Executing state: MOVING (Random Cleaning Pattern)')
        global LOW_BATTERY, DIRT_FOUND, OBSTACLE_FOUND, LASER_DATA

        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10hz loop rate

        while (rospy.Time.now() - start_time).to_sec() < self.move_duration:
            # --- Check for Events (Transitions) ---
            if LOW_BATTERY:
                rospy.loginfo("Event: LOW BATTERY detected! Transitioning to NAVIGATING_TO_CHARGER.")
                LOW_BATTERY = False
                self.pub.publish(Twist())  # Stop
                return 'low_battery'
                
            if DIRT_FOUND:
                rospy.loginfo("Event: DIRT FOUND detected! Transitioning to SPIRALING.")
                DIRT_FOUND = False
                self.pub.publish(Twist())  # Stop
                return 'dirt'
            
            # Check for obstacles using laser data
            if LASER_DATA is not None:
                # Check front sensors (center 60 degrees)
                ranges = LASER_DATA.ranges
                if len(ranges) > 0:
                    # Check front 60 degrees (30 degrees each side of center)
                    center = len(ranges) // 2
                    front_range = 60  # degrees
                    samples_per_degree = len(ranges) / 360.0
                    front_samples = int(front_range * samples_per_degree / 2)
                    
                    front_readings = ranges[center - front_samples:center + front_samples]
                    # Filter out invalid readings (0.0 or inf)
                    valid_readings = [r for r in front_readings if 0.1 < r < 10.0]
                    
                    if valid_readings and min(valid_readings) < 0.5:  # Obstacle within 0.5m
                        rospy.loginfo("Obstacle detected ahead! Transitioning to TURNING.")
                        self.pub.publish(Twist())  # Stop
                        return 'obstacle'
                
            if OBSTACLE_FOUND:
                rospy.loginfo("Event: OBSTACLE FOUND detected! Transitioning to TURNING.")
                OBSTACLE_FOUND = False
                self.pub.publish(Twist())  # Stop
                return 'obstacle'

            # Move forward with slight turn for better coverage
            twist = Twist(Vector3(self.linear_speed, 0, 0), Vector3(0, 0, self.angular_speed))
            self.pub.publish(twist)
            
            rate.sleep()
        
        # Stop before transitioning
        self.pub.publish(Twist())
        
        # Loop back to MOVING if no major event occurred
        return 'succeeded'


# --- 2. Custom State for TURNING (180-degree turn upon Obstacle) ---
# --- 2. Custom State for TURNING (180-degree turn upon Obstacle) ---
class TurningState(smach.State):
    """Executes a turn for obstacle recovery."""
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.angular_speed = 1.5  # Angular velocity (rad/s)
        self.min_turn_angle = 90  # Minimum turn angle
        self.max_turn_angle = 135  # Maximum turn angle for tight spaces

    def corner_Turn(self, in_corner, turn_angle, turn_right):
        # Add extra turn if in corner
        if in_corner:
            rospy.loginfo(f"Executing corner escape maneuver: {turn_angle} degrees")
            # First, back up a bit to get more clearance
            backup_cmd = Twist(Vector3(-0.1, 0, 0), Vector3(0, 0, 0))
            backup_time = rospy.Time.now()
            rate = rospy.Rate(10)
            while (rospy.Time.now() - backup_time).to_sec() < 0.5:
                self.pub.publish(backup_cmd)
                rate.sleep()
            self.pub.publish(Twist())
            rospy.sleep(0.2)

        # Calculate turn duration
        turn_duration = abs(turn_angle * 3.14159 / 180.0 / self.angular_speed)

        # Execute turn
        angular_vel = -self.angular_speed if turn_right else self.angular_speed
        turn_cmd = Twist(Vector3(0, 0, 0), Vector3(0, 0, angular_vel))

        rospy.loginfo(
            f"Turning {'right' if turn_right else 'left'} by {turn_angle} degrees for {turn_duration:.2f} seconds")

        start_time = rospy.Time.now()
        rate = rospy.Rate(10)

        while (rospy.Time.now() - start_time).to_sec() < turn_duration:
            self.pub.publish(turn_cmd)
            rate.sleep()

        
    def execute(self, userdata):
        rospy.loginfo('Executing state: TURNING (Obstacle avoidance turn)')
        global LASER_DATA
        
        # Stop first
        self.pub.publish(Twist())
        rospy.sleep(0.3)
        
        # Determine turn direction and angle by checking surroundings
        turn_right = True  # Default to right
        turn_angle = self.min_turn_angle
        in_corner = False
        
        if LASER_DATA is not None:
            ranges = LASER_DATA.ranges
            if len(ranges) > 0:
                # Divide laser scan into sections
                num_ranges = len(ranges)
                
                # Front: center ±30 degrees
                front_width = int(num_ranges * 60 / 360)
                center = num_ranges // 2
                front_start = center - front_width // 2
                front_end = center + front_width // 2
                
                # Left: 30-120 degrees from center
                left_start = int(num_ranges * 30 / 360)
                left_end = int(num_ranges * 120 / 360)
                
                # Right: -120 to -30 degrees from center
                right_start = num_ranges - int(num_ranges * 120 / 360)
                right_end = num_ranges - int(num_ranges * 30 / 360)
                
                # Get readings for each section
                front_readings = ranges[front_start:front_end]
                left_readings = ranges[left_start:left_end]
                right_readings = ranges[right_start:right_end]
                
                # Filter valid readings
                front_valid = [r for r in front_readings if 0.1 < r < 10.0]
                left_valid = [r for r in left_readings if 0.1 < r < 10.0]
                right_valid = [r for r in right_readings if 0.1 < r < 10.0]
                
                # Calculate average distances
                front_dist = sum(front_valid) / len(front_valid) if front_valid else 10.0
                left_dist = sum(left_valid) / len(left_valid) if left_valid else 10.0
                right_dist = sum(right_valid) / len(right_valid) if right_valid else 10.0
                
                # Detect if in corner (obstacles on multiple sides)
                front_blocked = front_dist < 0.6
                left_blocked = left_dist < 0.8
                right_blocked = right_dist < 0.8
                
                rospy.loginfo(f"Front: {front_dist:.2f}m, Left: {left_dist:.2f}m, Right: {right_dist:.2f}m")
                
                # Check for corner condition
                if front_blocked and (left_blocked or right_blocked):
                    in_corner = True
                    turn_angle = self.max_turn_angle  # Turn more in corners
                    rospy.loginfo("Corner detected! Using larger turn angle.")
                    
                    # If both sides blocked, turn towards less blocked side
                    if left_blocked and right_blocked:
                        turn_right = right_dist > left_dist
                    elif left_blocked:
                        turn_right = True
                    else:
                        turn_right = False
                else:
                    # Normal obstacle avoidance - turn towards more open side
                    if left_dist > right_dist:
                        turn_right = False
                        turn_angle = self.min_turn_angle
                    else:
                        turn_right = True
                        turn_angle = self.min_turn_angle
                    
                    # If one side is much more open, increase turn angle
                    space_difference = abs(left_dist - right_dist)
                    if space_difference > 1.0:
                        turn_angle = 110  # Medium turn for better clearance


        self.corner_Turn(in_corner, turn_angle, turn_right)

        # Add extra turn if in corner
        if in_corner:
            rospy.loginfo(f"Executing corner escape maneuver: {turn_angle} degrees")
            # First, back up a bit to get more clearance
            backup_cmd = Twist(Vector3(-0.1, 0, 0), Vector3(0, 0, 0))
            backup_time = rospy.Time.now()
            rate = rospy.Rate(10)
            while (rospy.Time.now() - backup_time).to_sec() < 0.5:
                self.pub.publish(backup_cmd)
                rate.sleep()
            self.pub.publish(Twist())
            rospy.sleep(0.2)
        
        # Calculate turn duration
        turn_duration = abs(turn_angle * 3.14159 / 180.0 / self.angular_speed)
        
        # Execute turn
        angular_vel = -self.angular_speed if turn_right else self.angular_speed
        turn_cmd = Twist(Vector3(0, 0, 0), Vector3(0, 0, angular_vel))
        
        rospy.loginfo(f"Turning {'right' if turn_right else 'left'} by {turn_angle} degrees for {turn_duration:.2f} seconds")
        
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)
        
        while (rospy.Time.now() - start_time).to_sec() < turn_duration:
            self.pub.publish(turn_cmd)
            rate.sleep()
        
        # Stop movement
        self.pub.publish(Twist())
        rospy.sleep(0.3)
        
        # If was in corner, add a small forward movement to clear the corner
        if in_corner:
            rospy.loginfo("Moving forward to clear corner...")
            forward_cmd = Twist(Vector3(0.2, 0, 0), Vector3(0, 0, 0))
            forward_time = rospy.Time.now()
            while (rospy.Time.now() - forward_time).to_sec() < 0.5:
                self.pub.publish(forward_cmd)
                rate.sleep()
            self.pub.publish(Twist())
            rospy.sleep(0.2)
        
        rospy.loginfo('Turn complete. Returning to Moving.')
        return 'succeeded'

# --- 3. Custom State for SPIRALING (Upon Dirt) ---
class SpiralingState(smach.State):
    """Executes a focused cleaning pattern."""
    def __init__(self):
        smach.State.__init__(self, outcomes=['dirt_cleared'])
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.spiral_time = 5.0

    def execute(self, userdata):
        rospy.loginfo('Executing state: SPIRALING (Cleaning dirt)')
        
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)

        while (rospy.Time.now() - start_time).to_sec() < self.spiral_time:
            # Move forward slowly while rotating (simulating spiral)
            twist = Twist(Vector3(0.1, 0, 0), Vector3(0, 0, 2.0))
            self.pub.publish(twist)
            rate.sleep()

        self.pub.publish(Twist())  # Stop
        rospy.loginfo("Dirt cleared. Returning to Moving.")
        return 'dirt_cleared'


# --- 4. Custom State for CHARGING (Simulated Wait) ---
class ChargingState(smach.State):
    """Simulates the time required to recharge the battery."""
    def __init__(self):
        smach.State.__init__(self, outcomes=['charged'])
        self.charging_time = 10.0

    def execute(self, userdata):
        rospy.loginfo(f'Executing state: CHARGING for {self.charging_time} seconds...')
        rospy.sleep(self.charging_time)
        rospy.loginfo('Battery charged. Returning to Moving.')
        return 'charged'


# --- Event Simulation Service Callbacks (Triggering Global Flags) ---

def low_battery_cb(req):
    global LOW_BATTERY
    rospy.loginfo("SERVICE TRIGGERED: LOW BATTERY")
    LOW_BATTERY = True
    return EmptyResponse()

def dirt_found_cb(req):
    global DIRT_FOUND
    rospy.loginfo("SERVICE TRIGGERED: DIRT FOUND")
    DIRT_FOUND = True
    return EmptyResponse()

def obstacle_found_cb(req):
    global OBSTACLE_FOUND
    rospy.loginfo("SERVICE TRIGGERED: OBSTACLE FOUND")
    OBSTACLE_FOUND = True
    return EmptyResponse()


# --- Main SMACH Execution Function ---
def main():
    rospy.init_node('vacuum_cleaner_smach')

    # Subscribe to laser scan for obstacle detection
    rospy.Subscriber('/base_scan', LaserScan, laser_callback)

    # Register the event simulation services
    rospy.Service('low_battery_trigger', Empty, low_battery_cb)
    rospy.Service('dirt_found_trigger', Empty, dirt_found_cb)
    rospy.Service('obstacle_trigger', Empty, obstacle_found_cb)
    rospy.loginfo("Services Ready: Call one of the *_trigger services to test transitions.")

    # Create the top-level State Machine Container
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    # Define the fixed goal for the charging station
    charging_station_goal = MoveBaseGoal()
    charging_station_goal.target_pose.header.frame_id = "map"
    charging_station_goal.target_pose.pose.position.x = 5.0
    charging_station_goal.target_pose.pose.position.y = 5.0
    charging_station_goal.target_pose.pose.orientation.w = 1.0

    with sm:
        # MOVING: Primary state, loops on 'succeeded'
        smach.StateMachine.add('MOVING', MovingState(),
                               transitions={'obstacle': 'TURNING',
                                            'low_battery': 'NAVIGATING_TO_CHARGER',
                                            'dirt': 'SPIRALING',
                                            'succeeded': 'MOVING'})

        # TURNING: Handles obstacle recovery
        smach.StateMachine.add('TURNING', TurningState(),
                               transitions={'succeeded': 'MOVING'})
        
        # SPIRALING: Handles cleaning a dirty spot
        smach.StateMachine.add('SPIRALING', SpiralingState(),
                               transitions={'dirt_cleared': 'MOVING'})

        # NAVIGATING_TO_CHARGER: SimpleActionState calling move_base
        smach.StateMachine.add('NAVIGATING_TO_CHARGER',
                               smach_ros.SimpleActionState(
                                   'move_base',
                                   MoveBaseAction,
                                   goal=charging_station_goal
                               ),
                               transitions={'succeeded': 'CHARGING',
                                            'aborted': 'MOVING',
                                            'preempted': 'MOVING'})
        
        # CHARGING: Waits until 'charged'
        smach.StateMachine.add('CHARGING', ChargingState(),
                               transitions={'charged': 'MOVING'})

    # Start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('smach_viewer_server', sm, '/SM_ROOT')
    sis.start()
    
    # Execute the state machine
    rospy.loginfo("Starting vacuum cleaner State Machine...")
    sm.execute()

    # Keep the node alive
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    rospy.Subscriber("/sim/low_battery", EmptyMsg, low_battery_cb)
    rospy.Subscriber("/sim/dirt_found", EmptyMsg, dirt_found_cb)
    rospy.Subscriber("/sim/obstacle_found", EmptyMsg, obstacle_found_cb)
    main()
