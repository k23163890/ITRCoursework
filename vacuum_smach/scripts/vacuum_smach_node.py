#!/usr/bin/env python

import rospy
import smach
import smach_ros
from geometry_msgs.msg import Twist, Vector3
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty, EmptyResponse
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty as EmptyMsg

LOW_BATTERY = False
DIRT_FOUND = False
OBSTACLE_FOUND = False
LASER_DATA = None


def laser_callback(scan):
    global LASER_DATA
    LASER_DATA = scan


# ----------------------------------------------------------------------
#  MOVING STATE
# ----------------------------------------------------------------------
class MovingState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['obstacle', 'low_battery', 'dirt', 'succeeded'])

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.move_duration = 5.0
        self.linear_speed = 0.3
        self.angular_speed = 0.0

    def execute(self, _):
        rospy.loginfo("State: MOVING")

        start_time = rospy.Time.now()
        rate = rospy.Rate(10)

        while (rospy.Time.now() - start_time).to_sec() < self.move_duration:
            if self.battery_low():
                return 'low_battery'
            if self.dirt_detected():  
                return 'dirt'
            if self.laser_obstacle(): 
                return 'obstacle'
            if self.obstacle_event(): 
                return 'obstacle'

            forward = Twist(
                Vector3(self.linear_speed, 0, 0),
                Vector3(0, 0, self.angular_speed)
            )
            self.pub.publish(forward)
            rate.sleep()

        self.pub.publish(Twist())
        return 'succeeded'
    

    def battery_low(self):
        global LOW_BATTERY
        if LOW_BATTERY:
            LOW_BATTERY = False
            self.pub.publish(Twist())
            rospy.loginfo("Battery low -> Charging")
            return True

    def dirt_detected(self):
        global DIRT_FOUND
        if DIRT_FOUND:
            DIRT_FOUND = False
            self.pub.publish(Twist())
            rospy.loginfo("Dirt detected -> Spiral Clean")
            return True

    def obstacle_event(self):
        global OBSTACLE_FOUND
        if OBSTACLE_FOUND:
            OBSTACLE_FOUND = False
            self.pub.publish(Twist())
            rospy.loginfo("Obstacle event triggered")
            return True

    def laser_obstacle(self):
        global LASER_DATA
        if LASER_DATA is None:
            return False

        readings = LASER_DATA.ranges
        if not readings:
            return False

        center = len(readings) // 2
        scan_width_deg = 60
        samples_per_deg = len(readings) / 360.0
        span = int(scan_width_deg * samples_per_deg / 2)

        slice_front = readings[center - span:center + span]
        valid = [r for r in slice_front if 0.1 < r < 10.0]

        if valid and min(valid) < 0.5:
            self.pub.publish(Twist())
            rospy.loginfo("Obstacle detected ahead (laser scan)")
            return True

        return False


# ----------------------------------------------------------------------
#  TURNING STATE
# ----------------------------------------------------------------------
class TurningState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.angular_speed = 1.5
        self.min_angle = 90
        self.max_angle = 135

    def execute(self, _):
        rospy.loginfo("State: TURNING")

        global LASER_DATA
        rospy.sleep(0.3)

        turn_right = True
        angle = self.min_angle
        corner_mode = False

        if LASER_DATA:
            turn_right, angle, corner_mode = self._analyse_scan()

        self._perform_turn(turn_right, angle, corner_mode)
        return 'succeeded'

    # ---- Laser-based Observation ----
    def _analyse_scan(self):
        """Determine safest turn direction and angle from laser scan."""
        scans = LASER_DATA.ranges
        n = len(scans)

        front = scans[n//2 - self._samples(30): n//2 + self._samples(30)]
        left  = scans[self._samples(30):self._samples(120)]
        right = scans[n - self._samples(120):n - self._samples(30)]

        fv, lv, rv = self._filter(front), self._filter(left), self._filter(right)

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

        return turn_right, angle, corner

    def _perform_turn(self, turn_right, angle, corner):
        """Rotate by the chosen angle, plus a small backup/forward if cornered."""
        if corner:
            self._backup()

        self._rotate(turn_right, angle)

        if corner:
            self._forward()

    # ---- Motion helpers ----
    def _samples(self, deg):
        return int(len(LASER_DATA.ranges) * deg / 360)

    def _filter(self, arr):
        return [r for r in arr if 0.1 < r < 10]

    def _avg(self, seq, fallback):
        return sum(seq)/len(seq) if seq else fallback

    def _backup(self):
        cmd = Twist(Vector3(-0.1, 0, 0), Vector3())
        self._run_for(0.5, cmd)

    def _forward(self):
        cmd = Twist(Vector3(0.2, 0, 0), Vector3())
        self._run_for(0.5, cmd)

    def _rotate(self, right, angle):
        duration = abs(angle * 3.14159 / 180 / self.angular_speed)
        angular = -self.angular_speed if right else self.angular_speed
        cmd = Twist(Vector3(), Vector3(0, 0, angular))
        self._run_for(duration, cmd)

    def _run_for(self, seconds, cmd):
        t0 = rospy.Time.now()
        rate = rospy.Rate(10)
        while (rospy.Time.now() - t0).to_sec() < seconds:
            self.pub.publish(cmd)
            rate.sleep()
        self.pub.publish(Twist())


# ----------------------------------------------------------------------
#  SPIRALING STATE
# ----------------------------------------------------------------------
class SpiralingState(smach.State):
    """Rotate fast while moving forward slowly to simulate spot cleaning."""
    def __init__(self):
        smach.State.__init__(self, outcomes=['dirt_cleared'])
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.duration = 5.0

    def execute(self, _):
        rospy.loginfo("State: SPIRALING")
        t0 = rospy.Time.now()
        rate = rospy.Rate(10)

        while (rospy.Time.now() - t0).to_sec() < self.duration:
            cmd = Twist(Vector3(0.1, 0, 0), Vector3(0, 0, 2.0))
            self.pub.publish(cmd)
            rate.sleep()

        self.pub.publish(Twist())
        return 'dirt_cleared'


# ----------------------------------------------------------------------
#  CHARGING STATE
# ----------------------------------------------------------------------
class ChargingState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['charged'])
        self.duration = 10.0

    def execute(self, _):
        rospy.loginfo("State: CHARGING")
        rospy.sleep(self.duration)
        return 'charged'


# ----------------------------------------------------------------------
#  Service callbacks
# ----------------------------------------------------------------------
def low_battery_cb(_):
    global LOW_BATTERY
    LOW_BATTERY = True
    return EmptyResponse()

def dirt_found_cb(_):
    global DIRT_FOUND
    DIRT_FOUND = True
    return EmptyResponse()

def obstacle_found_cb(_):
    global OBSTACLE_FOUND
    OBSTACLE_FOUND = True
    return EmptyResponse()


# ----------------------------------------------------------------------
#  MAIN
# ----------------------------------------------------------------------
def main():
    rospy.init_node('vacuum_cleaner_smach')

    rospy.Subscriber('/base_scan', LaserScan, laser_callback)
    rospy.Service('low_battery_trigger', Empty, low_battery_cb)
    rospy.Service('dirt_found_trigger', Empty, dirt_found_cb)
    rospy.Service('obstacle_trigger', Empty, obstacle_found_cb)

    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    dock_goal = MoveBaseGoal()
    dock_goal.target_pose.header.frame_id = "map"
    dock_goal.target_pose.pose.position.x = 5.0
    dock_goal.target_pose.pose.position.y = 5.0
    dock_goal.target_pose.pose.orientation.w = 1.0

    with sm:
        sm.add('MOVING', MovingState(),
               transitions={
                   'obstacle': 'TURNING',
                   'low_battery': 'NAVIGATING_TO_CHARGER',
                   'dirt': 'SPIRALING',
                   'succeeded': 'MOVING'
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
    sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    rospy.Subscriber("/sim/low_battery", EmptyMsg, low_battery_cb)
    rospy.Subscriber("/sim/dirt_found", EmptyMsg, dirt_found_cb)
    rospy.Subscriber("/sim/obstacle_found", EmptyMsg, obstacle_found_cb)
    main()
