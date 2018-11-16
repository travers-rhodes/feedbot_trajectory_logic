//
// This class connects to DOMUS 
// and can be used to send target angles for DOMUS to move to
//
#ifndef CUSTOM_DOMUS_INTERFACE_H_
#define CUSTOM_DOMUS_INTERFACE_H_

#include "feedbot_trajectory_logic/domus_interface.h"
#include <ros/ros.h>
#include <serial/serial.h>
#include <math.h>

class CustomDomusInterface : public DomusInterface
{
  public:
    CustomDomusInterface();
    virtual void InitializeConnection();
    virtual bool SendTargetAngles(const std::vector<double> &joint_angles, float secs);
  private:
    serial::Serial ser;
};
#endif
