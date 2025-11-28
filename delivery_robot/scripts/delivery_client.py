#!/usr/bin/env python3
import rospy
import actionlib
from geometry_msgs.msg import Point
from delivery_robot.msg import GoToLocationAction, GoToLocationGoal, LookAtAction, LookAtGoal

def main():
    rospy.init_node('delivery_client')

    # Create action clients
    goto_client = actionlib.SimpleActionClient('goto_location', GoToLocationAction)
    look_client = actionlib.SimpleActionClient('look_at', LookAtAction)

    rospy.loginfo("Waiting for action servers...")
    goto_client.wait_for_server()
    look_client.wait_for_server()
    rospy.loginfo("Connected to action servers.")

    #Kichen
    rospy.loginfo("Going to KITCHEN...")
    goto_client.send_goal(GoToLocationGoal(location_name="kitchen"))
    goto_client.wait_for_result()
    rospy.loginfo("IN KITCHEN")


    #Person
    rospy.loginfo("Looking at PERSON...")
    person = Point(x=3.0, y=1.0, z=0.0)
    look_client.send_goal(LookAtGoal(target=person))
    look_client.wait_for_result()
    rospy.loginfo("Looked at Person")

    #Living room
    rospy.loginfo("Going to LIVING ROOM...")
    goto_client.send_goal(GoToLocationGoal(location_name="living_room"))
    goto_client.wait_for_result()

    rospy.loginfo("Delivery task complete!")

if __name__ == '__main__':
    main()
