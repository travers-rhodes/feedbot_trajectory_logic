//
// This class connects to DOMUS via a serial connection 
// and can be used to send target angles for DOMUS to move to
//
#ifndef DOMUS_INTERFACE_H_
#define DOMUS_INTERFACE_H_

#include "niryo_one_msgs/RobotMoveCommand.h"
#include "niryo_one_msgs/RobotMoveAction.h"
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <mutex>
class DomusInterface
{
  public:
    DomusInterface();
    virtual void InitializeConnection(ros::NodeHandle nh);
    virtual void SendTargetAngles(const std::vector<double> &joint_angles);
  private:
    std::shared_ptr<actionlib::SimpleActionClient<niryo_one_msgs::RobotMoveAction>> ac_;
    std::mutex mtx_;
};
#endif
