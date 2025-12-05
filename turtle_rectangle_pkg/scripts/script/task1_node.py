#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

pose = Pose()

def pose_msg(msg):
    global pose
    pose = msg

def doMaths(corner_x, corner_y):
    dx = corner_x - pose.x
    dy = corner_y - pose.y
    d = math.hypot(dx, dy)
    ang = math.atan2(dy, dx)
    err = math.atan2(math.sin(ang - pose.theta), math.cos(ang - pose.theta))
    return d, err

def move_forward(pub, rate, speed, dist):
    travelled = 0.0
    while not rospy.is_shutdown() and travelled < dist:
        cmd = Twist()
        cmd.linear.x = speed
        pub.publish(cmd)
        travelled += speed / rate.frequency
        rate.sleep()


def turn_left(pub, rate, turn_speed, angle):
    turned = 0.0
    while not rospy.is_shutdown() and turned < angle:
        cmd = Twist()
        cmd.angular.z = turn_speed
        pub.publish(cmd)
        turned += turn_speed / rate.frequency
        rate.sleep()

def reach_corner(pub, rate):
    while not rospy.is_shutdown():
        err = math.atan2(math.sin(0 - pose.theta), math.cos(0 - pose.theta))
        if abs(err) < 0.01:
            break
        cmd = Twist()
        cmd.angular.z = 4*err
        pub.publish(cmd)
        rate.sleep()



def closed_loop_corner():
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(50)
    corner_x, corner_y = 3.0, 2.0
    while not rospy.is_shutdown():
        vals = doMaths(corner_x,corner_y)
        if vals[0] < 0.01:
            break
        cmd = Twist()
        cmd.linear.x = min(1.0, 2*vals[0])
        cmd.angular.z = 6*vals[1]
        pub.publish(cmd)
        rate.sleep()
    pub.publish(Twist())
    rate.sleep()
    reach_corner(pub, rate)

def open_loop_rectangle():
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(40)
    L = 5.0
    H = 3.0
    speed = 1.0
    turn_speed = 1.5
    # store rate frequency so helper functions can use it
    rate.frequency = 40.0

    while not rospy.is_shutdown():
        move_forward(pub, rate, speed, L)
        turn_left(pub, rate, turn_speed, math.pi / 2)
        move_forward(pub, rate, speed, H)
        turn_left(pub, rate, turn_speed, math.pi / 2)
        move_forward(pub, rate, speed, L)
        turn_left(pub, rate, turn_speed, math.pi / 2)
        move_forward(pub, rate, speed, H)
        turn_left(pub, rate, turn_speed, math.pi / 2)


if __name__ == "__main__":
    rospy.init_node("turtle_rectangle_node")
    rospy.Subscriber("/turtle1/pose", Pose, pose_msg)
    rospy.sleep(0.5)
    closed_loop_corner()
    open_loop_rectangle()
