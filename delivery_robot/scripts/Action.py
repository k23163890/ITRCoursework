#!/usr/bin/env python3
import rospy
import actionlib
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from delivery_robot.msg import GoToLocationAction, GoToLocationResult, LookAtAction, LookAtResult
from delivery_robot.srv import GetLocation


class Action:
    def __init__(self):
        rospy.init_node("Action")
        self.goto_server = actionlib.SimpleActionServer("goto_location", GoToLocationAction, self.GoToLocation, auto_start=False)
        self.lookat_server = actionlib.SimpleActionServer("look_at", LookAtAction, self.lookAt, auto_start=False)
        rospy.wait_for_service("location")
        self.location = rospy.ServiceProxy("location", GetLocation)
        self.moveBase()

        self.goto_server.start()
        self.lookat_server.start()

        rospy.spin()

    def moveBase(self):
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move base action server")

    def computeQuat(self, target, movegoal):
        quat = quaternion_from_euler(0, 0, target.theta)
        movegoal.target_pose.pose.orientation = Quaternion(*quat)

    def move_base_functions(self, movegoal, **kwargs):
        if kwargs.get("send_goal"):
            self.move_base_client.send_goal(movegoal)
            self.move_base_client.wait_for_result()

    def GoToLocation(self, goal):
        response = self.location(goal.locationName)

        target = response.pose
        movegoal = MoveBaseGoal()
        movegoal.target_pose.header.stamp = rospy.Time.now()
        movegoal.target_pose.header.frame_id = "map"
        movegoal.target_pose.pose.position.x = target.x
        movegoal.target_pose.pose.position.y = target.y
        self.computeQuat(target, movegoal)
        self.move_base_functions(movegoal, send_goal=True)


        if self.move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            result = GoToLocationResult(passed=True)
            self.goto_server.set_succeeded(result)
        else:
            result = GoToLocationResult(passed=False)
            self.goto_server.set_aborted(result)

    def lookAt(self, goal):
        rospy.sleep(4)
        result = LookAtResult(passed=True)
        self.lookat_server.set_succeeded(result)


if __name__ == "__main__":
    Action()
