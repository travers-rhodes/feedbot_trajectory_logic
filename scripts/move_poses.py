#!/usr/bin/env python
import rospy

import numpy as np
from geometry_msgs.msg import Pose, Quaternion, Point
from feedbot_trajectory_logic.srv import MoveToPose, MoveToJointAngles
from tf.transformations import quaternion_from_euler, quaternion_multiply


class MovePoses:
  def __init__(self):
    rospy.logwarn("waiting for move_to_pose")
    rospy.wait_for_service("move_to_pose")
    self.move_ = rospy.ServiceProxy("move_to_pose", MoveToPose)
    rospy.logwarn("continuing after finding move_to_pose service")
    rospy.logwarn("waiting for move_to_joint_angles")
    rospy.wait_for_service("move_to_joint_angles")
    self.move_to_angles_ = rospy.ServiceProxy("move_to_joint_angles", MoveToJointAngles)
    rospy.logwarn("continuing after finding move_to_joint_angles")
   
    self.move_to_home() 

    self.move_(Pose(Point(0,0.3,0.3),Quaternion(1,0,0,0)))


  def move_to_home(self):
    joint_angles = np.array([0, -np.pi/2, np.pi/2, 0, np.pi/2, 0])
    time = 5.0
    self.move_to_angles_(joint_angles, time) 

if __name__=="__main__":
  rospy.init_node('trajectory_logic_node', anonymous=True)
  s = MovePoses()
