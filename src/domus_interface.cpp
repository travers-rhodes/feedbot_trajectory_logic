//
// This class connects to DOMUS via a serial connection 
// and can be used to send target angles for DOMUS to move to
//
#include "feedbot_trajectory_logic/domus_interface.h"

const uint8_t REQUEST_JOINT_ANGLES = 136;

std::vector<double> max_joint_angles{ 2.7, 0 + 0.64, 2.1 - 1.35, 2.7, 2.4, 2.5 };
std::vector<double> min_joint_angles{ -2.7, -2.3 + 0.64, 0 - 1.35, -2.7, -2.4, -2.5 };


DomusInterface::DomusInterface()
{
}

void
DomusInterface::InitializeConnection(ros::NodeHandle nh)
{
  ac_ = std::make_shared<actionlib::SimpleActionClient<niryo_one_msgs::RobotMoveAction>>("/niryo_one/commander/robot_action", true);
  ac_->waitForServer();
}

void
DomusInterface::SendTargetAngles(const std::vector<double> &joint_angles)
{
  for (int i = 0; i < joint_angles.size(); i++) {
    if (joint_angles[i] > max_joint_angles[i] || joint_angles[i] < min_joint_angles[i]) 
    {
      ROS_ERROR_STREAM("The requested joint " << i << " was " << joint_angles[i] << " which is past the joint limits.");
      return;
    }
  }

  

  // use a mutex (apparently i should use a unique_lock, but i have no idea why...) to ensure only
  // one command sent at a time 
  niryo_one_msgs::RobotMoveGoal goal;
  niryo_one_msgs::RobotMoveCommand cmd;
  niryo_one_msgs::TrajectoryPlan traj_plan;
  // it would seem weird to me if this were actually used in any way
  moveit_msgs::RobotState traj_start;
  // it would seem weird to me if this were actually used in any way
  std::string group_name;
  // this is pretty much the only actually relevant thing we're filling out
  moveit_msgs::RobotTrajectory trajectory;
  trajectory_msgs::JointTrajectory joint_trajectory;
  std::vector<std::string> joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
  std::vector<trajectory_msgs::JointTrajectoryPoint> points;
  
  trajectory_msgs::JointTrajectoryPoint point;
  point.positions = joint_angles;
  point.time_from_start = ros::Duration(0.1);
  points.push_back(point);

  // now, finally, fill out the structure
  joint_trajectory.joint_names = joint_names;
  joint_trajectory.points = points;
  trajectory.joint_trajectory = joint_trajectory;
  traj_plan.trajectory_start = traj_start;
  traj_plan.group_name = group_name;
  traj_plan.trajectory = trajectory;
  //execute trajectory
  cmd.cmd_type = 7;
  cmd.Trajectory = traj_plan;
  goal.cmd = cmd;

  ac_->sendGoalAndWait(goal);
}
