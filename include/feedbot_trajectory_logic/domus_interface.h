//
// This class connects to DOMUS via a serial connection 
// and can be used to send target angles for DOMUS to move to
//
#ifndef DOMUS_INTERFACE_H_
#define DOMUS_INTERFACE_H_

#include "control_msgs/FollowJointTrajectoryAction.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <mutex>
class DomusInterface
{
  public:
    DomusInterface();
    virtual void InitializeConnection(ros::NodeHandle nh);
    virtual void SendTargetAngles(const std::vector<double> &joint_angles, float secs);
  private:
    std::shared_ptr<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>> ac_;
    ros::Publisher reset_pub_;
};
#endif
