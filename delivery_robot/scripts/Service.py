#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose2D
from delivery_robot.srv import GetLocation, GetLocationResponse

def handler(req):
    if req.locationName == "garage":
        x = 2.1
        y = 9.4
        th = -1.6
    elif req.locationName == "living_room":
        x = 5.8
        y = 6.2
        th = -2.3
    elif req.locationName == "person":
        x = 0.3
        y = 8.9
        th = -3.1
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
    rospy.Service('location', GetLocation, handler)
    rospy.spin()

if __name__ == '__main__':
    main()