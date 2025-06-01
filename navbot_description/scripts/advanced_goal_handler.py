#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String

class AdvancedGoalHandler:
    def __init__(self):
        rospy.init_node('advanced_goal_handler', anonymous=True)
        self.goal_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        self.light_state_subscriber = rospy.Subscriber('/traffic_light_state', String, self.light_state_callback)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")
        self.current_goal = None
        self.traffic_light_state = "RED"  # Assume the initial state is RED

    def goal_callback(self, msg):
        goal = MoveBaseGoal()
        goal.target_pose.header = msg.header
        goal.target_pose.pose = msg.pose
        self.current_goal = goal  # Store the goal
        if self.traffic_light_state == "GREEN":
            self.send_goal()

    def send_goal(self):
        if self.current_goal is not None:
            rospy.loginfo("Sending goal to move_base")
            self.client.send_goal(self.current_goal, done_cb=self.done_callback, feedback_cb=self.feedback_callback)

    def light_state_callback(self, msg):
        self.traffic_light_state = msg.data
        rospy.loginfo(f"Traffic light state: {self.traffic_light_state}")
        if self.traffic_light_state == "RED":
            rospy.loginfo("Red light detected, canceling current goal.")
            self.client.cancel_all_goals()
        elif self.traffic_light_state == "GREEN" and self.current_goal is not None:
            rospy.loginfo("Green light detected, resending goal.")
            self.send_goal()

    def done_callback(self, status, result):
        rospy.loginfo(f"Goal reached with status: {status}")
        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal successfully completed.")
        else:
            rospy.loginfo("Goal did not complete successfully.")

    def feedback_callback(self, feedback):
        rospy.loginfo(f"Current location: {feedback.base_position.pose.position.x:.2f}, {feedback.base_position.pose.position.y:.2f}")

if __name__ == '__main__':
    try:
        handler = AdvancedGoalHandler()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Advanced goal handler node terminated.")

