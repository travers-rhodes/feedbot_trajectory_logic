//
// This ros node starts a server at "track_pose_server" which exports a service
// "update_pose_target". The API of that service call is, if "stopMotion" is set
// on the request, stop the robot
// and otherwise, start the robot moving toward the given target pose
// this service also periodically publishes to "/distance_to_target"
//
#include <ros/ros.h>
#include <feedbot_trajectory_logic/jacobian_controller.h>
#include "feedbot_trajectory_logic/MoveToPose.h"
#include "feedbot_trajectory_logic/MoveToJointAngles.h"
#include "feedbot_trajectory_logic/joint_echoing_interface.h"
#include "feedbot_trajectory_logic/ros_robot_interface.h"
#include "feedbot_trajectory_logic/custom_domus_interface.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "geometry_msgs/Pose.h"

class MoveToPoseService 
{
  public:
    MoveToPoseService(double step_size_meters, RobotInterface* robot_interface, ros::NodeHandle*, std::string link_prefix);
    bool move_to_pose(feedbot_trajectory_logic::MoveToPose::Request &req,
           feedbot_trajectory_logic::MoveToPose::Response &res);
    bool move_to_joint_angles(feedbot_trajectory_logic::MoveToJointAngles::Request &req,
           feedbot_trajectory_logic::MoveToJointAngles::Response &res);
 
  private:
    JacobianController controller_;
    RobotInterface* robot_interface_;
};

