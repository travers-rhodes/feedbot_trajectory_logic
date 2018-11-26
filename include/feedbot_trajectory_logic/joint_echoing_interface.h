//
// This class connects to DOMUS 
// and can be used to send target angles for DOMUS to move to
//
#ifndef JOINT_ECHOING_INTERFACE_H_
#define JOINT_ECHOING_INTERFACE_H_
#include "feedbot_trajectory_logic/domus_interface.h"
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

class JointEchoingInterface : public DomusInterface
{
  public:
    JointEchoingInterface(ros::NodeHandle* n);
    virtual void InitializeConnection();
    virtual bool SendTargetAngles(const std::vector<double> &joint_angles, float secs);
    std::vector<double> max_joint_angles{ 2.7, 0 + 0.64, 2.1 - 1.35, 2.7, 2.4, 2.5 };
    std::vector<double> min_joint_angles{ -2.7, -2.3 + 0.64, 0 - 1.35, -2.7, -2.4, -2.5 };
  protected:
    void PublishRobotState(const std::vector<double> &joint_value);
  private:
    sensor_msgs::JointState joint_state_;
    ros::Publisher joint_pub_;
};
#endif
