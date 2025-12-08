#!/usr/bin/env python3
import rospy
import actionlib
from geometry_msgs.msg import Pose2D, Quaternion
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from delivery_robot.msg import GoToLocationAction, GoToLocationResult, LookAtAction, LookAtResult
from delivery_robot.srv import GetLocation


class Action:
    def __init__(self):
        rospy.init_node("Action")

        self.subscribe_odom()
        rospy.loginfo("Odom messages subscribed")

        rospy.wait_for_service("location")
        self.location = rospy.ServiceProxy("location", GetLocation)
        rospy.loginfo("Service client prepared")

        self.moveBase()


        self.goto_server = actionlib.SimpleActionServer(
            "goto_location", GoToLocationAction, self.GoToLocation, auto_start=False
        )

        self.lookat_server = actionlib.SimpleActionServer(
            "look_at", LookAtAction, self.lookAt, auto_start=False
        )
        rospy.loginfo("Custom action servers created")
        self.goto_server.start()
        self.lookat_server.start()

        rospy.spin()


    def subscribe_odom(self):
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.current_pose = Pose2D()

    def moveBase(self):
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move base action server")


    def odom_callback(self, msg):
        self.current_pose.x = msg.pose.pose.position.x
        self.current_pose.y = msg.pose.pose.position.y

    def computeQuat(self, target, mb_goal):
        quat = quaternion_from_euler(0, 0, target.theta)
        mb_goal.target_pose.pose.orientation = Quaternion(*quat)


    def move_base_functions(self, mb_goal, **kwargs):
        if kwargs.get("send_goal"):
            self.move_base_client.send_goal(mb_goal)
            self.move_base_client.wait_for_result()


    def GoToLocation(self, goal):
        response = self.location(goal.locationName)

        target = response.pose
        mb_goal = MoveBaseGoal()
        mb_goal.target_pose.header.stamp = rospy.Time.now()
        mb_goal.target_pose.header.frame_id = "map"
        mb_goal.target_pose.pose.position.x = target.x
        mb_goal.target_pose.pose.position.y = target.y
        rospy.loginfo("Built move_base message and filled positions")



        self.computeQuat(target, mb_goal)

        self.move_base_functions(mb_goal, send_goal=True)

        state = self.move_base_client.get_state()

        if state == actionlib.GoalStatus.SUCCEEDED:
            result = GoToLocationResult(passed=True)
            self.goto_server.set_succeeded(result)
        else:
            result = GoToLocationResult(passed=False)
            self.goto_server.set_aborted(result)

    def lookAt(self, goal):
        rospy.sleep(3)
        result = LookAtResult(passed=True)
        self.lookat_server.set_succeeded(result)


if __name__ == "__main__":
    Action()
