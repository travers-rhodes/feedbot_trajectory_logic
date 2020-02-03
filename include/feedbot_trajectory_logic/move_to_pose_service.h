#include <ros/ros.h>
#include <feedbot_trajectory_logic/jacobian_controller.h>
#include "feedbot_trajectory_logic/MoveToPose.h"
#include "feedbot_trajectory_logic/MoveToJointAngles.h"
#include "feedbot_trajectory_logic/joint_echoing_interface.h"
#include "feedbot_trajectory_logic/ros_robot_interface.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "geometry_msgs/Pose.h"

class MoveToPoseService 
{
  public:
    MoveToPoseService(double step_size_meters, RobotInterface* robot_interface, ros::NodeHandle*, std::string link_prefix, std::string robot_description_param_name);
    bool move_to_pose(feedbot_trajectory_logic::MoveToPose::Request &req,
           feedbot_trajectory_logic::MoveToPose::Response &res);
    bool move_to_joint_angles(feedbot_trajectory_logic::MoveToJointAngles::Request &req,
           feedbot_trajectory_logic::MoveToJointAngles::Response &res);
 
  private:
    JacobianController controller_;
    RobotInterface* robot_interface_;
};

