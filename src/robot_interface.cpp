#include "feedbot_trajectory_logic/robot_interface.h"

RobotInterface::RobotInterface(CustomRobotParams robot_params)
{
  max_joint_angles_ = robot_params.max_joint_angles;
  min_joint_angles_ = robot_params.min_joint_angles;
  joint_names_ = robot_params.joint_names;
}

void
RobotInterface::InitializeConnection()
{
}

bool
RobotInterface::SendTargetAngles(const std::vector<double> &joint_angles, float secs)
{
}
