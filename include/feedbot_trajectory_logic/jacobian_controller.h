//
// This class computes the robot jacobian and can be use to move the robot along a straight-line path
// (in cylindrical coordinates) to the target pose (in cartesian coordinates)
//
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <feedbot_trajectory_logic/robot_interface.h>
#include <feedbot_trajectory_logic/transform_helpers.h>

// this could probably be a struct, no? shrugs.
class JointUpdateResult
{
  public:
    JointUpdateResult(std::vector<double> jp, Eigen::Affine3d eefp, double qd, double td, bool at) {
      joint_positions = jp;
      eef_pose = eefp;
      quat_dist = qd;
      trans_dist = td;
      at_target = at;
    }
    JointUpdateResult(const JointUpdateResult& jur) {
      joint_positions = jur.joint_positions;
      eef_pose = jur.eef_pose;
      quat_dist = jur.quat_dist;
      trans_dist = jur.trans_dist;
      at_target = jur.at_target;
    }
    std::vector<double> joint_positions;
    Eigen::Affine3d eef_pose;
    double quat_dist;
    double trans_dist;
    bool at_target;
};

class JacobianController
{
  public:
    JacobianController(double trans_step_size_meters, RobotInterface* robot_interface, ros::NodeHandle* n, std::string robot_description_param_name);
    JointUpdateResult plan_step_to_target_pose(const std::vector<double> joint_angles, const geometry_msgs::Pose &target_pose);
    double make_step_to_target_pose(const geometry_msgs::Pose &target_pose);

  private:
    void scale_down_step(double step_scale, Eigen::Vector3d &trans_diff, double &rot_angle, Eigen::Vector3d &cylindrical_diff);
    Eigen::VectorXd get_joint_delta(Eigen::Vector3d cylindrical_diff, double rot_angle, Eigen::Vector3d rot_axis);
    Eigen::MatrixXd get_cylindrical_jacobian();

    Eigen::Affine3d current_pose_;
    robot_state::RobotStatePtr kinematic_state_;
    const robot_state::JointModelGroup* joint_model_group_;
    robot_model::RobotModelPtr kinematic_model_;
    robot_model_loader::RobotModelLoader robot_model_loader_;
    sensor_msgs::JointState joint_state_;
    RobotInterface* robot_interface_;
    float _trans_step_size_meters;
    std::string link_prefix_;
    int num_joints_;
};
