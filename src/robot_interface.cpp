#include "feedbot_trajectory_logic/robot_interface.h"

RobotInterface::RobotInterface(CustomRobotParams robot_params)
{
  max_joint_angles_ = robot_params.max_joint_angles;
  min_joint_angles_ = robot_params.min_joint_angles;
  initial_joint_values_ = robot_params.initial_joint_values;
  joint_names_ = robot_params.joint_names;
  srdf_group_name_ = robot_params.srdf_group_name;
  end_effector_link_ = robot_params.end_effector_link;
}

void
RobotInterface::InitializeConnection()
{
}

trajectory_msgs::JointTrajectory
RobotInterface::SendTrajectory(const trajectory_msgs::JointTrajectory &joint_trajectory, bool can_speed_up)
{
}

bool
RobotInterface::SendTargetAngles(const std::vector<double> &joint_angles, float secs)
{
}

void
RobotInterface::GetCurrentAngles(std::vector<double> &joint_angles, std::vector<std::string> &joint_names)
{
}
