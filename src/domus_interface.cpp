//
// This class connects to DOMUS 
// and can be used to send target angles for DOMUS to move to
//
#include "feedbot_trajectory_logic/domus_interface.h"

DomusInterface::DomusInterface()
{
}

void
DomusInterface::InitializeConnection()
{
}

// move to the target joint_angles and the motion should take you secs seconds.
// return false if we know the robot motion failed
bool
DomusInterface::SendTargetAngles(const std::vector<double> &joint_angles, float secs)
{
}
