#include <vector>
#include <string>

class CustomRobotParams
{
  public:
    std::vector<double> min_joint_angles, max_joint_angles;
    std::vector<std::string> joint_names; 
};

class NiryoRobotParams : public CustomRobotParams
{
  public:
    NiryoRobotParams()
    {
      std::vector<double> max = { 2.7, 0 + 0.64, 2.1 - 1.35, 2.7, 2.4, 2.5 };
      std::vector<double> min = { -2.7, -2.3 + 0.64, 0 - 1.35, -2.7, -2.4, -2.5 };
      std::vector<std::string> names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
      max_joint_angles = max;
      min_joint_angles = min;
      joint_names = names;
    };
};
