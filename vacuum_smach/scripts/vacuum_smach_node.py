#!/usr/bin/env python

import rospy
import smach
import smach_ros
from geometry_msgs.msg import Twist, Vector3
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty as EmptyMsg

from vacuum_smach.msg import WanderAction, WanderGoal, WanderResult

LASER_DATA = None

def laser_callback(scan):
    global LASER_DATA
    LASER_DATA = scan

def wander_result_cb(userdata, status, result):
    rospy.loginfo(f"Wander Action returned result: {result.stop_reason}")
    if result.stop_reason == 'low_battery':
        return 'low_battery'
    elif result.stop_reason == 'dirt':
        return 'dirt'
    elif result.stop_reason == 'obstacle':
        return 'obstacle'
    elif result.stop_reason == 'succeeded':
        return 'succeeded'
    else:
        return 'aborted'

class TurningState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.angular_speed = 1.5
        self.min_angle = 90
        self.max_angle = 135

    def execute(self, _):
        rospy.loginfo("State: TURNING - Analyzing scan to turn safely")
        
        self.pub.publish(Twist())
        rospy.sleep(0.5)

        turn_right = True
        angle = self.min_angle
        corner_mode = False

        if LASER_DATA:
            turn_right, angle, corner_mode = self._analyse_scan()
        else:
            rospy.logwarn("No Laser Data! Turning blindly.")

        self._perform_turn(turn_right, angle, corner_mode)
        return 'succeeded'

    def _analyse_scan(self):
        """Determine safest turn direction and angle from laser scan."""
        scans = LASER_DATA.ranges
        if not scans:
            return True, 90, False

        n = len(scans)
        
        # Define indices for slices
        # Front: Center +/- 30 degrees
        # Left: +30 to +120 degrees
        # Right: -30 to -120 degrees
        center_idx = n // 2
        deg_30_idx = self._samples(30)
        deg_120_idx = self._samples(120)

        front = scans[center_idx - deg_30_idx : center_idx + deg_30_idx]
        left  = scans[deg_30_idx : deg_120_idx]

        right = scans[n - deg_120_idx : n - deg_30_idx]

        fv = self._filter(front)
        lv = self._filter(left)
        rv = self._filter(right)

        f = self._avg(fv, 10.0)
        l = self._avg(lv, 10.0)
        r = self._avg(rv, 10.0)

        front_blocked = f < 0.6
        left_blocked  = l < 0.8
        right_blocked = r < 0.8

        angle = self.min_angle
        turn_right = True
        corner = False

        if front_blocked and (left_blocked or right_blocked):
            corner = True
            angle = self.max_angle
            if left_blocked and right_blocked:
                turn_right = r > l
            else:
                turn_right = not left_blocked
        else:
            turn_right = not (l > r)
            if abs(l - r) > 1.0:
                angle = 110
        
        rospy.loginfo(f"Turn Decision: Right={turn_right}, Angle={angle}, Corner={corner}")
        return turn_right, angle, corner

    def _perform_turn(self, turn_right, angle, corner):
        if corner:
            rospy.loginfo("Corner detected - Backing up first")
            self._backup()

        direction_str = "Right" if turn_right else "Left"
        rospy.loginfo(f"Rotating {direction_str} by {angle} degrees")
        self._rotate(turn_right, angle)

        if corner:
            self._forward()

    def _samples(self, deg):
        if not LASER_DATA or not LASER_DATA.ranges: return 0
        return int(len(LASER_DATA.ranges) * deg / 360.0)

    def _filter(self, arr):
        return [r for r in arr if 0.1 < r < 10.0]

    def _avg(self, seq, fallback):
        return sum(seq)/len(seq) if seq else fallback

    def _backup(self):
        cmd = Twist(Vector3(-0.1, 0, 0), Vector3())
        self._run_for(1.0, cmd)

    def _forward(self):
        cmd = Twist(Vector3(0.2, 0, 0), Vector3())
        self._run_for(1.0, cmd)

    def _rotate(self, right, angle):

        duration = abs((angle * 3.14159 / 180.0) / self.angular_speed)
        angular = -self.angular_speed if right else self.angular_speed
        cmd = Twist(Vector3(), Vector3(0, 0, angular))
        self._run_for(duration, cmd)

    def _run_for(self, seconds, cmd):
        t0 = rospy.Time.now()
        rate = rospy.Rate(10)
        while (rospy.Time.now() - t0).to_sec() < seconds and not rospy.is_shutdown():
            self.pub.publish(cmd)
            rate.sleep()
        self.pub.publish(Twist())


class SpiralingState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['dirt_cleared'])
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.duration = 5.0

    def execute(self, _):
        rospy.loginfo("State: SPIRALING - Spot cleaning dirt")
        t0 = rospy.Time.now()
        rate = rospy.Rate(10)

        linear_v = 0.2
        angular_v = 2.0
        
        while (rospy.Time.now() - t0).to_sec() < self.duration and not rospy.is_shutdown():
            angular_v = max(0.5, angular_v - 0.02)
            cmd = Twist(Vector3(linear_v, 0, 0), Vector3(0, 0, angular_v))
            self.pub.publish(cmd)
            rate.sleep()

        self.pub.publish(Twist())
        rospy.loginfo("Spiraling complete - Dirt cleared")
        return 'dirt_cleared'


class ChargingState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['charged'])
        self.duration = 5.0

    def execute(self, _):
        rospy.loginfo("State: CHARGING - Charging battery...")
        rospy.sleep(self.duration)
        return 'charged'


def main():
    rospy.init_node('vacuum_cleaner_smach')
    

    rospy.Subscriber('/base_scan', LaserScan, laser_callback)

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    dock_goal = MoveBaseGoal()
    dock_goal.target_pose.header.frame_id = "map"
    dock_goal.target_pose.pose.position.x = 5.0
    dock_goal.target_pose.pose.position.y = 5.0
    dock_goal.target_pose.pose.orientation.w = 1.0

    wander_goal = WanderGoal(duration=20.0)

    with sm:
        smach.StateMachine.add('MOVING',
               smach_ros.SimpleActionState('wander', WanderAction, 
                                           goal=wander_goal,
                                           result_cb=wander_result_cb,
                                           outcomes=['succeeded', 'aborted', 'preempted', 'obstacle', 'low_battery', 'dirt']),
               transitions={
                   'obstacle': 'TURNING',
                   'low_battery': 'NAVIGATING_TO_CHARGER',
                   'dirt': 'SPIRALING',
                   'succeeded': 'MOVING',
                   'aborted': 'aborted',
                   'preempted': 'preempted'
               })

        sm.add('TURNING', TurningState(),
               transitions={'succeeded': 'MOVING'})

        sm.add('SPIRALING', SpiralingState(),
               transitions={'dirt_cleared': 'MOVING'})

        sm.add('NAVIGATING_TO_CHARGER',
               smach_ros.SimpleActionState('move_base', MoveBaseAction, goal=dock_goal),
               transitions={
                   'succeeded': 'CHARGING',
                   'aborted': 'MOVING',
                   'preempted': 'MOVING'
               })

        sm.add('CHARGING', ChargingState(),
               transitions={'charged': 'MOVING'})

    sis = smach_ros.IntrospectionServer('viewer', sm, '/SM_ROOT')
    sis.start()
    
    rospy.loginfo("Starting State Machine execution...")
    sm.execute()
    
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()