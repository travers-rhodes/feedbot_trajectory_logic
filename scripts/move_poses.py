#!/usr/bin/env python
import rospy

import numpy as np
from geometry_msgs.msg import Pose, Quaternion, Point
from feedbot_trajectory_logic.srv import MoveToPose, MoveToJointAngles
from tf.transformations import quaternion_from_euler, quaternion_multiply


class MovePoses:
  def __init__(self):
    self.focal_point = np.array([-0.11, 0.43, 0])
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
    # easy hack to keep robot gripper far from table in preparatory move
    # move all the angles (except the first/zeroth one) to their eventual goal. Thus the final move should be primarily rotational and not toward table
    self.move_to_angles_([0, 0.8932028016555025, 0.06301116856960177, 1.3685584384486664, 2.8477393386885352, -1.298948058495412, -1.9756862537838606],10)

    grip_path_radius = 0.12 # in meters


    for theta in np.linspace(0,2*np.pi, 60):
      target = self.focal_point + np.array([grip_path_radius * np.cos(theta), grip_path_radius * np.sin(theta), 0.2])
      self.move_to_target(target) 
      if theta == 0:
        # turn on camera to start recording
        rospy.set_param('/is_recording_active', True)

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
    # compute the down-angle so that the _camera_ points to the focal-point
    # I did this out by hand, and I think we want
    # (z = grip height, B = camera-offset-from-grip-axis, a = down_angle, r = radius distance from grip to focal point)
    # z cos(a) + B = r sin(a)
    # cos(a) = (-2 z B +/- \sqrt((2 z B)^2 - 4 (z^2 + B^2)(B^2 - r^2))) / 2(z^2 + B^2)
    r = np.sqrt(delta[0]**2 + delta[1]**2)
    z = delta[2]
    B = 0.06 # i just guessed this number 
    cosa = (-2 * z * B + np.sqrt((2 * z * B)**2 - 4 * (z**2 + B**2) * (B**2 - r**2))) / (2 * (z**2 + B**2))
    if (cosa < epsilon):
      down_angle = np.pi/4
    else:
      down_angle = np.arccos(cosa)

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
