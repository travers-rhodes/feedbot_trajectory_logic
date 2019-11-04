#!/usr/bin/env python
import rospy

import numpy as np
from geometry_msgs.msg import Pose, Quaternion, Point
from feedbot_trajectory_logic.srv import MoveToPose, MoveToJointAngles
from tf.transformations import quaternion_from_euler, quaternion_multiply


class MovePoses:
  def __init__(self):
    self.focal_point = np.array([0, 0.43, 0])
    # quaternion is defined in order x,y,z,w
    self.defaultQuat = np.array([0.5, 0.5, 0.5, 0.5])
    rospy.logwarn("waiting for move_to_pose")
    rospy.wait_for_service("move_to_pose")
    self.move_ = rospy.ServiceProxy("move_to_pose", MoveToPose)
    rospy.logwarn("continuing after finding move_to_pose service")
    rospy.logwarn("waiting for move_to_joint_angles")
    rospy.wait_for_service("move_to_joint_angles")
    self.move_to_angles_ = rospy.ServiceProxy("move_to_joint_angles", MoveToJointAngles)
    rospy.logwarn("continuing after finding move_to_joint_angles")
   
    self.move_to_home() 

    for theta in np.arange(0,4*np.pi, 0.1):
      target = self.focal_point + np.array([0.1 * np.cos(theta), 0.1 * np.sin(theta), 0.2])
      self.move_to_target(target) 

  def move_to_home(self):
    joint_angles = np.array([0, np.pi/4, 0, np.pi / 2.0, 0, - np.pi / 4, -np.pi/2])
    time = 10.0
    self.move_to_angles_(joint_angles, time) 

  def move_to_target(self, point):
    newpose = Pose()
    point_msg = Point()
    point_msg.x = point[0]
    point_msg.y = point[1]
    point_msg.z = point[2]
    delta = np.array(point) - self.focal_point
    epsilon = 0.001
    r = np.sqrt(delta[0]**2 + delta[1]**2)
    if (r < epsilon):
      down_angle = np.pi/4
    else:
      down_angle = np.arctan(delta[2]/r)

    rot = np.arctan2(-delta[1], -delta[0])

    newpose.position = point_msg
    newpose.orientation = quatMsg(quaternion_multiply(quaternion_from_euler(0,down_angle,rot), self.defaultQuat))
    res = self.move_(newpose)
    rospy.logwarn(res)

def quatMsg(tfquat):
    return(Quaternion(tfquat[0],tfquat[1],tfquat[2],tfquat[3]))

if __name__=="__main__":
  rospy.init_node('spoon_feeder', anonymous=True)
  s = MovePoses()
