//
// This class connects to DOMUS 
// and can be used to send target angles for DOMUS to move to
//
#ifndef NIRYO_INTERFACE_H_
#define NIRYO_INTERFACE_H_

#include "feedbot_trajectory_logic/domus_interface.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <mutex>
class NiryoInterface : public DomusInterface
{
  public:
    NiryoInterface();
    virtual void InitializeConnection();
    virtual bool SendTargetAngles(const std::vector<double> &joint_angles, float secs);
  private:
    std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> ac_;
    ros::Publisher reset_pub_;
};
#endif
