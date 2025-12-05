#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose2D
from delivery_bot.srv import GetLocation, GetLocationResponse

def handle_get_location(req):
    if req.location_name == "kitchen":
        rospy.loginfo("Found kitchen coords")
        x = 10.7
        y = 4.3
        th = 0.0
    elif req.location_name == "living_room":
        x = 10.4
        y = 8.9
        th = 1.57
    elif req.location_name == "person":
        x = 1.0
        y = 0.0
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
    rospy.init_node('location_server')
    rospy.Service('get_location', GetLocation, handle_get_location)
    rospy.loginfo("Location Service Server is ready with corrected map coordinates.")
    rospy.spin()

if __name__ == '__main__':
    main()