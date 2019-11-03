#!/usr/bin/env python
import rospy

import numpy as np
from geometry_msgs.msg import Pose, Quaternion, Point
from feedbot_trajectory_logic.srv import MoveToPose


class MovePoses:
  def __init__(self):
    # quaternion is defined in order x,y,z,w
    self.defaultQuat = Quaternion(0.5, 0.5, 0.5, 0.5)
    rospy.logwarn("waiting for move_to_pose")
    rospy.wait_for_service("move_to_pose")
    rospy.logwarn("continuing after finding move_to_pose service")
    self.move_ = rospy.ServiceProxy("move_to_pose", MoveToPose)
    self.move_to_target([0.3 + 0.3,0.05,0.17])

  def move_to_target(self, point):
    newpose = Pose()
    point_msg = Point()
    point_msg.x = point[0]
    point_msg.y = point[1]
    point_msg.z = point[2]
    newpose.position = point_msg
    newpose.orientation = self.defaultQuat
    res = self.move_(newpose)
    rospy.logwarn(res)

if __name__=="__main__":
  rospy.init_node('spoon_feeder', anonymous=True)
  s = MovePoses()
