//
// This class connects to DOMUS 
// and can be used to send target angles for DOMUS to move to
//
#ifndef ROS_ROBOT_INTERFACE_H_ 
#define ROS_ROBOT_INTERFACE_H_ 

#include "control_msgs/FollowJointTrajectoryAction.h"
#include "feedbot_trajectory_logic/robot_interface.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <memory>
#include <vector>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
class RosRobotInterface : public RobotInterface
{
  public:
    RosRobotInterface(std::string follow_joint_trajectory_name, std::string joint_state_name, ros::NodeHandle* nh, CustomRobotParams robot_params, std::string robot_description_param_name = "robot_description");
    virtual void InitializeConnection();
    virtual bool SendTargetAngles(const std::vector<double> &joint_angles, float secs);
    virtual void GetCurrentAngles(std::vector<double> &joint_angles, std::vector<std::string> &joint_names);
    virtual void SendTrajectory(const trajectory_msgs::JointTrajectory &joint_trajectory);
  private:
    ros::NodeHandle* nh_;
    std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> ac_;
    std::string follow_joint_trajectory_name_;
    std::string joint_state_name_;
    robot_model::RobotModelConstPtr kinematic_model_; 
    robot_state::RobotStatePtr kinematic_state_;
};
#endif
