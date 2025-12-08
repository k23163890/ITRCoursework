#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose2D
from delivery_robot.srv import GetLocation, GetLocationResponse

def handle_get_location(req):
    if req.locationName == "kitchen":
        x = 6.0
        y = 4.3
        th = 0.0
    elif req.locationName == "living_room":
        x = 10.8
        y = 3.13
        th = -2.3
    elif req.locationName == "person":
        x = 6.0
        y = 1.9
        th = 0.0
    else:
        rospy.logwarn("Unknown location")
        x = 0.0
        y = 0.0
        th = 0.0
        
    pos = Pose2D()
    pos.x = x
    pos.y = y
    pos.theta = th

    return GetLocationResponse(pos)


def main():
    rospy.init_node('Service')
    rospy.Service('location', GetLocation, handle_get_location)
    rospy.spin()

if __name__ == '__main__':
    main()