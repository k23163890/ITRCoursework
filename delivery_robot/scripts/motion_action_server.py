#!/usr/bin/env python3
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped, Pose2D, Quaternion
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  # New Imports
from delivery_bot.msg import (
    GoToLocationAction, GoToLocationResult,
    LookAtAction, LookAtResult
)
from delivery_bot.srv import GetLocation


class MotionActionServer:
    def __init__(self):
        rospy.init_node("motion_action_server")

        # --- Publishers / Subscribers ---
        # NOTE: We are replacing the simple publisher with an Action Client for move_base
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.current_pose = Pose2D()

        # --- Service Client ---
        rospy.wait_for_service("get_location")
        self.get_location = rospy.ServiceProxy("get_location", GetLocation)

        # --- move_base Action Client ---
        # This client is used to send navigation goals to the ROS move_base node
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("[Action] Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("[Action] Connected to move_base server.")

        # --- Custom Action Servers ---
        self.goto_server = actionlib.SimpleActionServer(
            "goto_location", GoToLocationAction, self.execute_goto, auto_start=False
        )
        self.lookat_server = actionlib.SimpleActionServer(
            "look_at", LookAtAction, self.execute_lookat, auto_start=False
        )

        self.goto_server.start()
        self.lookat_server.start()

        rospy.loginfo("[Action] MotionActionServer running and ready for goals.")
        rospy.spin()

    # --- Callbacks ---
    def odom_callback(self, msg):
        self.current_pose.x = msg.pose.pose.position.x
        self.current_pose.y = msg.pose.pose.position.y
        # Note: You might want to update the orientation here too if you use it.

        def _perform_180_rotation(self):
            rospy.loginfo("[Helper] Executing 180-degree rotation.")

            # 1. Get current position and orientation
            current_x = self.current_pose.x
            current_y = self.current_pose.y
            current_yaw = self.current_pose.theta

            # 2. Calculate the target yaw: current_yaw + 180 degrees (pi radians)
            target_yaw = current_yaw + math.pi

            # Normalize the angle to be within the [-pi, pi] range
            if target_yaw > math.pi:
                target_yaw -= 2 * math.pi
            elif target_yaw < -math.pi:
                target_yaw += 2 * math.pi

            rospy.loginfo(f"[Helper] Current Yaw: {current_yaw:.2f} rad. Target Yaw: {target_yaw:.2f} rad.")

            # 3. Create the MoveBaseGoal
            mb_goal = MoveBaseGoal()
            mb_goal.target_pose.header.stamp = rospy.Time.now()
            mb_goal.target_pose.header.frame_id = "map"

            # Set position to the current position (in-place rotation)
            mb_goal.target_pose.pose.position.x = current_x
            mb_goal.target_pose.pose.position.y = current_y

            # Set orientation to the new 180-degree rotated yaw
            quat = quaternion_from_euler(0, 0, target_yaw)
            mb_goal.target_pose.pose.orientation = Quaternion(*quat)

            # 4. Send goal to move_base and wait for result
            self.move_base_client.send_goal(mb_goal)
            self.move_base_client.wait_for_result()

            state = self.move_base_client.get_state()

            # 5. Return True/False based on the rotation result
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("[Helper] Successfully completed 180-degree rotation.")
                return True
            else:
                rospy.logerr(f"[Helper] Rotation failed or aborted. Final state: {state}")
                return False


    def execute_goto(self, goal):
        # ... (initial checks and service call code remains the same)

        # 3. Send goal to move_base and wait for result
        self.move_base_client.send_goal(mb_goal)
        rospy.loginfo(f"[Action] Sent goal to move_base. Waiting for result...")

        self.move_base_client.wait_for_result()

        state = self.move_base_client.get_state()

        # Check if the goal succeeded (state 3 is SUCCEEDED)
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"[Action] Successfully reached {goal.location_name}. Preparing to rotate...")

            # 👇 NEW CODE: Call the rotation helper
            rotation_success = self._perform_180_rotation()

            if rotation_success:
                result = GoToLocationResult(success=True)
                self.goto_server.set_succeeded(result)
            else:
                # Treat successful travel but failed rotation as an aborted action
                rospy.logwarn("[Action] Navigation succeeded, but 180-degree rotation failed.")
                result = GoToLocationResult(success=False)
                self.goto_server.set_aborted(result)

        else:
            rospy.logerr(f"[Action] Navigation failed or aborted. Final state: {state}")
            result = GoToLocationResult(success=False)
            self.goto_server.set_aborted(result)


    # --- Actions ---
    def execute_goto(self, goal):
        rospy.loginfo(f"[Action] Received request: Go to {goal.location_name}")

        # Check for preemption request
        if self.goto_server.is_preempt_requested():
            self.move_base_client.cancel_goal()
            self.goto_server.set_preempted()
            rospy.loginfo("[Action] GoToLocation preempted.")
            return

        # 1. Get target coordinates from the service
        try:
            resp = self.get_location(goal.location_name)
        except rospy.ServiceException as e:
            rospy.logerr(f"[Action] Failed to call get_location: {e}")
            self.goto_server.set_aborted(GoToLocationResult(success=False))
            return

        target = resp.pose
        rospy.loginfo(f"[Action] Navigating to {goal.location_name} at ({target.x:.2f}, {target.y:.2f})")

        # 2. Create the MoveBaseGoal
        mb_goal = MoveBaseGoal()
        mb_goal.target_pose.header.stamp = rospy.Time.now()
        mb_goal.target_pose.header.frame_id = "map"

        mb_goal.target_pose.pose.position.x = target.x
        mb_goal.target_pose.pose.position.y = target.y

        quat = quaternion_from_euler(0, 0, target.theta)
        mb_goal.target_pose.pose.orientation = Quaternion(*quat)

        # 3. Send goal to move_base and wait for result
        self.move_base_client.send_goal(mb_goal)
        rospy.loginfo(f"[Action] Sent goal to move_base. Waiting for result...")

        # Wait until navigation completes or preemption occurs
        self.move_base_client.wait_for_result()

        state = self.move_base_client.get_state()

        # Check if the goal succeeded (state 3 is SUCCEEDED)
        if state == actionlib.GoalStatus.SUCCEEDED:
            result = GoToLocationResult(success=True)
            self.goto_server.set_succeeded(result)
            rospy.loginfo(f"[Action] Successfully reached {goal.location_name}.")
        else:
            rospy.logerr(f"[Action] Navigation failed or aborted. Final state: {state}")
            result = GoToLocationResult(success=False)
            self.goto_server.set_aborted(result)

    def execute_lookat(self, goal):
        # NOTE: This is a placeholder as you did not implement actual head/base movement
        # to 'look at' the target. It just succeeds immediately.
        rospy.loginfo(f"[Action] Received LookAt target: ({goal.target.x:.2f}, {goal.target.y:.2f})")

        # In a real robot, you would publish a command here to turn the base or the head
        # For now, we simulate success
        rospy.sleep(3)

        result = LookAtResult(success=True)
        self.lookat_server.set_succeeded(result)
        rospy.loginfo("[Action] Finished LookAt action (simulated success).")


if __name__ == "__main__":
    MotionActionServer()