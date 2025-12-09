#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import angles
from math import pi, sqrt, atan2

pose  = Pose()

def posenew(msg):
    global pose
    pose = msg


def closed_loop(pub, x, y, th):
    vel = Twist()
    rate = rospy.Rate(40)
    while not rospy.is_shutdown():
        dx = x - pose.x 
        dy = y - pose.y
        dist = sqrt(dx**2 + dy**2)
        if dist < 0.1: break
        vel.linear.x = dist
        vel.angular.z = 4.0 * angles.shortest_angular_distance(pose.theta, atan2(dy, dx))
        pub.publish(vel)
        rate.sleep()
    while not rospy.is_shutdown():
        err = angles.shortest_angular_distance(pose.theta, th)
        if abs(err) < 0.01: break
        vel.linear.x = 0.0
        vel.angular.z = 4.0 * err
        pub.publish(vel)
        rate.sleep()
    pub.publish(Twist())




def open_loop(pub):
    vel = Twist()
    rate = rospy.Rate(20)
    steps = [(1.0, 0.0,80), (0.0, -pi/4, 40), (1.0, 0.0, 40), (0.0, -pi/4, 40)]
    pub.publish(Twist())
    rate.sleep()
    while not rospy.is_shutdown():
        for lin, ang, ticks in steps:
            vel.linear.x = lin
            vel.angular.z = ang
            for n in range(ticks):
                pub.publish(vel)
                rate.sleep()
    pub.publish(Twist())





if __name__ == "__main__":
    rospy.init_node("rectangle_node", anonymous=True)
    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    rospy.Subscriber("/turtle1/pose", Pose, posenew)
    rospy.sleep(1)
    x = 3.5
    y = 6
    th = 0.0
    closed_loop(pub, x,y, th)
    open_loop(pub)