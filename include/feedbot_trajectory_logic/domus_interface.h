//
// This class connects to DOMUS 
// and can be used to send target angles for DOMUS to move to
//
#ifndef DOMUS_INTERFACE_H_
#define DOMUS_INTERFACE_H_
#include <vector>

class DomusInterface
{
  public:
    DomusInterface();
    virtual void InitializeConnection();
    virtual bool SendTargetAngles(const std::vector<double> &joint_angles, float secs);
    std::vector<double> max_joint_angles{ 2.7, 0 + 0.64, 2.1 - 1.35, 2.7, 2.4, 2.5 };
    std::vector<double> min_joint_angles{ -2.7, -2.3 + 0.64, 0 - 1.35, -2.7, -2.4, -2.5 };
};
#endif
