//
// This class connects to DOMUS via a serial connection 
// and can be used to send target angles for DOMUS to move to
//
#include "feedbot_trajectory_logic/domus_interface.h"
#include "ros/ros.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "std_msgs/String.h"

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
  if (mtx_.try_lock()) {  
    niryo_one_msgs::RobotMoveGoal goal;
    niryo_one_msgs::RobotMoveCommand cmd;
    cmd.cmd_type = 1;
    cmd.joints = joint_angles; 
    goal.cmd = cmd;
    ac_->sendGoalAndWait(goal);
    mtx_.unlock();
  }
}
