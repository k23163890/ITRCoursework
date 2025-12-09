#!/usr/bin/env python3
import rospy
import actionlib
from delivery_robot.msg import GoToLocationAction, GoToLocationGoal, LookAtAction, LookAtGoal
from geometry_msgs.msg import Point



def go_to_garage(client):
    rospy.loginfo("Going to Garage")
    client.send_goal(GoToLocationGoal(locationName="garage"))
    client.wait_for_result()
    rospy.loginfo("In Garage")


def look_at_person(look_client):
    rospy.loginfo("Looking at Person")
    look_client.send_goal(LookAtGoal(target = Point(x=0.3, y=8.9, z=0.0)))
    look_client.wait_for_result()
    rospy.loginfo("Looked at Person")


def go_to_living_room(client):
    rospy.loginfo("Going to Living Room")
    client.send_goal(GoToLocationGoal(locationName="living_room"))
    client.wait_for_result()
    rospy.loginfo("In LivingRoom")


def main():
    rospy.init_node('Client')

    client = actionlib.SimpleActionClient('locationServer', GoToLocationAction)
    look_client = actionlib.SimpleActionClient('lookAtServer', LookAtAction)

    client.wait_for_server()
    look_client.wait_for_server()
    rospy.loginfo("Server Request passed")

    go_to_garage(client)

    look_at_person(look_client)

    go_to_living_room(client)

    rospy.loginfo("Finished task")

if __name__ == '__main__':
    main()
