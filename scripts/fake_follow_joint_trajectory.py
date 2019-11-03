#!/usr/bin/env python
import rospy
# Brings in the SimpleActionClient
import actionlib

from moveit_msgs.msg import ExecuteTrajectoryAction as moveitETA
from moveit_msgs.msg import ExecuteTrajectoryGoal as moveitETG
from moveit_msgs.msg import RobotTrajectory
from control_msgs.msg import FollowJointTrajectoryAction as ControlFJTA
from control_msgs.msg import FollowJointTrajectoryResult as ControlFJTR

from feedbot_trajectory_logic.srv import MoveToPose

class FakeFollowJointTrajectoryAction:
  def __init__(self):
    self.client = actionlib.SimpleActionClient('/execute_trajectory', moveitETA)
    self.client.wait_for_server()

    self._result = ControlFJTR()
    self._as = actionlib.SimpleActionServer("/follow_joint_trajectory", ControlFJTA, execute_cb=self.execute_cb, auto_start = False)
    rospy.logwarn("Starting callback server\n")
    self._as.start()


  def execute_cb(self, msg):
    rospy.logwarn("Calling Execute Callback for fake followjointtrajectory")
    goal = moveitETG()
    moveit_robot_trajectory = RobotTrajectory();
    moveit_robot_trajectory.joint_trajectory = msg.trajectory
    goal.trajectory = moveit_robot_trajectory 
    self.client.send_goal(goal)
    self.client.wait_for_result()
    # we don't actually use the result for anything...

    self._as.set_succeeded(self._result)
    rospy.logwarn("Done Execute Callback for fake followjointtrajectory")

if __name__=="__main__":
  rospy.init_node('fake_follow_joint_trajectory', anonymous=True)
  server = FakeFollowJointTrajectoryAction()
  rospy.spin()
