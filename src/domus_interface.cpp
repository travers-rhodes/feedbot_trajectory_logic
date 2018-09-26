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
  ac_ = std::make_shared<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>("/niryo_one_follow_joint_trajectory_controller/follow_joint_trajectory", true);
  ac_->waitForServer();
}

// move to the target joint_angles and the motion should take you secs seconds.
// return false if we know the robot motion failed
bool
DomusInterface::SendTargetAngles(const std::vector<double> &joint_angles, float secs)
{
  for (int i = 0; i < joint_angles.size(); i++) {
    if (joint_angles[i] > max_joint_angles[i] || joint_angles[i] < min_joint_angles[i]) 
    {
      ROS_ERROR_STREAM("The requested joint " << i << " was " << joint_angles[i] << " which is past the joint limits.");
      return false;
    }
  }

  // use a mutex (apparently i should use a unique_lock, but i have no idea why...) to ensure only
  // one command sent at a time 
  control_msgs::FollowJointTrajectoryGoal goal;
  trajectory_msgs::JointTrajectory joint_trajectory;
  std::vector<std::string> joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
  std::vector<trajectory_msgs::JointTrajectoryPoint> points;

  std::vector<control_msgs::JointTolerance> tols;
  for (int i = 0; i < joint_angles.size(); i++) {
    control_msgs::JointTolerance tol;
    tol.name = joint_names[i];
    tol.position = 5;
    tol.velocity = 5;
    tol.acceleration = 5;
    tols.push_back(tol);
  }
  
  trajectory_msgs::JointTrajectoryPoint point;
  point.positions = joint_angles;
  point.time_from_start = ros::Duration(secs);
  points.push_back(point);

  // now, finally, fill out the structure
  joint_trajectory.joint_names = joint_names;
  joint_trajectory.points = points;
  goal.trajectory = joint_trajectory;
  goal.path_tolerance = tols;
  
  std_msgs::Empty empt;
  ac_->sendGoal(goal);
  return true;
}
