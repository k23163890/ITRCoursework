#!/usr/bin/env python

import rospy
import actionlib
import random
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty, EmptyResponse

from vacuum_smach.msg import WanderAction, WanderResult

class WanderActionServer:
    def __init__(self):
        rospy.init_node('wander_server')
        
        self.laser_data = None
        self.low_battery = False
        self.dirt_found = False
        self.obstacle_found = False

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/base_scan', LaserScan, self.laser_cb)
        
        
        rospy.Subscriber("/sim/dirt_found", EmptyMsg, self.dirt_cb)
        rospy.Subscriber("/sim/obstacle_found", EmptyMsg, self.obstacle_cb)

        rospy.Service('/sim/low_battery', Empty, self.battery_service_cb)

        self._as = actionlib.SimpleActionServer("wander", WanderAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        rospy.loginfo("Wander Action Server Started")


    def laser_cb(self, msg): 
        self.laser_data = msg

    def dirt_cb(self, msg): 
        rospy.loginfo("TOPIC RECEIVED: Dirt Found")
        self.dirt_found = True

    def obstacle_cb(self, msg): 
        rospy.loginfo("TOPIC RECEIVED: Manual Obstacle Found")
        self.obstacle_found = True

    def battery_service_cb(self, req):
        rospy.loginfo("SERVICE CALLED: Low Battery")
        self.low_battery = True
        return EmptyResponse()

    def check_laser_obstacle(self):
        if self.laser_data is None: return False
        readings = self.laser_data.ranges
        if not readings: return False
        
        center = len(readings) // 2
        span = int(len(readings) / 360.0 * 30)
        valid = [r for r in readings[center-span:center+span] if 0.1 < r < 10.0]
        
        if valid and min(valid) < 0.5:
            return True
        return False

    def execute_cb(self, goal):
        rospy.loginfo(f"Action: Wandering for {goal.duration} seconds")
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()
        
        self.low_battery = False
        self.dirt_found = False
        self.obstacle_found = False
        
        result = WanderResult()

        while (rospy.Time.now() - start_time).to_sec() < goal.duration and not rospy.is_shutdown():
            if self._as.is_preempt_requested():
                rospy.loginfo('Action Preempted')
                self.pub.publish(Twist()) 
                self._as.set_preempted()
                return

            if self.low_battery:
                self.pub.publish(Twist())
                result.stop_reason = "low_battery"
                self._as.set_aborted(result)
                return
            
            if self.dirt_found:
                self.pub.publish(Twist())
                result.stop_reason = "dirt"
                self._as.set_aborted(result)
                return

            if self.obstacle_found or self.check_laser_obstacle():
                self.pub.publish(Twist())
                result.stop_reason = "obstacle"
                self._as.set_aborted(result)
                return

            cmd = Twist()
            cmd.linear.x = 0.3
            cmd.angular.z = random.uniform(-0.5, 0.5) 
            self.pub.publish(cmd)
            rate.sleep()

        self.pub.publish(Twist())
        result.stop_reason = "succeeded"
        self._as.set_succeeded(result)

if __name__ == '__main__':
    server = WanderActionServer()
    rospy.spin()