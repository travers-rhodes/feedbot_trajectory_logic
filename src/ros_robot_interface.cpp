//
// This class connects to DOMUS 
// and can be used to send target angles for DOMUS to move to
//
#include "feedbot_trajectory_logic/ros_robot_interface.h"

RosRobotInterface::RosRobotInterface(std::string follow_joint_trajectory_name, std::string joint_state_name, ros::NodeHandle* nh, CustomRobotParams robot_params) : RobotInterface(robot_params)
{
  follow_joint_trajectory_name_ = follow_joint_trajectory_name;
  joint_state_name_ = joint_state_name;
  nh_ = nh;
}

void
RosRobotInterface::InitializeConnection()
{
  ac_ = std::make_shared<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>(follow_joint_trajectory_name_, true);
  ac_->waitForServer();
}
  
void
RosRobotInterface::GetCurrentAngles(std::vector<double> &joint_angles, std::vector<std::string> &joint_names)
{
  //https://answers.ros.org/question/73258/detecting-timeout-from-waitformessage/
  boost::shared_ptr<sensor_msgs::JointState const> js = ros::topic::waitForMessage<sensor_msgs::JointState>(joint_state_name_, ros::Duration(10));
  if (js == NULL) {
    throw;
  }
  std::copy(js->position.begin(), js->position.begin() + joint_names_.size(), std::back_inserter(joint_angles));
  joint_names = joint_names_;
}


// send the robot on a whole biglong path 
void
RosRobotInterface::SendTrajectory(const trajectory_msgs::JointTrajectory &joint_trajectory)
{
  // I should use a mutex (apparently i should use a unique_lock, but i have no idea why...) to ensure only
  // one command sent at a time 
  
  std::vector<control_msgs::JointTolerance> tols;
  for (int i = 0; i < joint_names_.size(); i++) {
    control_msgs::JointTolerance tol;
    tol.name = joint_names_[i];
    tol.position = 5;
    tol.velocity = 5;
    tol.acceleration = 5;
    tols.push_back(tol);
  }
  
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = joint_trajectory;
  goal.path_tolerance = tols;
  ac_->sendGoal(goal);
  ac_->waitForResult();
  boost::shared_ptr<const control_msgs::FollowJointTrajectoryResult> result = ac_->getResult();
  int error_code = result->error_code;
  ROS_INFO_STREAM("The error code from calling the follow joint trajectory action was " << error_code << ".");
  if (error_code)
  {
    ROS_ERROR_STREAM("The error code from calling the follow joint trajectory action was nonzero.");
    throw; 
  }
} 

// move to the target joint_angles and the motion should take you secs seconds.
// return false if we know the robot motion failed
bool
RosRobotInterface::SendTargetAngles(const std::vector<double> &joint_angles, float secs)
{
  for (int i = 0; i < joint_angles.size(); i++) {
    if (joint_angles[i] > max_joint_angles_[i] || joint_angles[i] < min_joint_angles_[i]) 
    {
      ROS_ERROR_STREAM("The requested joint " << i << " was " << joint_angles[i] << " which is past the joint limits.");
      return false;
    }
  }

  trajectory_msgs::JointTrajectory joint_trajectory;
  std::vector<trajectory_msgs::JointTrajectoryPoint> points;

  trajectory_msgs::JointTrajectoryPoint point;
  point.positions = joint_angles;
  point.time_from_start = ros::Duration(secs);
  points.push_back(point);

  // now, finally, fill out the structure
  joint_trajectory.joint_names = joint_names_;
  joint_trajectory.points = points;
  SendTrajectory(joint_trajectory);
  return true;
}
