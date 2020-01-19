#include <feedbot_trajectory_logic/move_to_pose_service.h>

MoveToPoseService::MoveToPoseService(double step_size_meters, RobotInterface* robot_interface, ros::NodeHandle* n, std::string link_prefix, std::string robot_description_param_name) : controller_(step_size_meters, robot_interface, n, link_prefix, robot_description_param_name)
{
  robot_interface_ = robot_interface;
}

bool MoveToPoseService::move_to_joint_angles(feedbot_trajectory_logic::MoveToJointAngles::Request &req, feedbot_trajectory_logic::MoveToJointAngles::Response &res)
{
  std::vector<double> joint_positions;
  std::vector<std::string> joint_names;
  robot_interface_->GetCurrentAngles(joint_positions, joint_names);
  
  trajectory_msgs::JointTrajectory jt;
  float cumulative_time_secs(0);
  trajectory_msgs::JointTrajectoryPoint start_point;
  start_point.positions = joint_positions; 
  ros::Duration start_time(cumulative_time_secs);
  start_point.time_from_start = start_time;
  jt.points.push_back(start_point);

  trajectory_msgs::JointTrajectoryPoint point;
  point.positions = req.joint_positions; 
  cumulative_time_secs += req.target_time;
  ros::Duration cur_time(cumulative_time_secs);
  point.time_from_start = cur_time;
  jt.points.push_back(point);
  
  jt.joint_names = joint_names;
  res.joint_trajectory = jt;
  bool can_speed_up = false;
  robot_interface_->SendTrajectory(jt, can_speed_up);
  return true;
}

bool MoveToPoseService::move_to_pose(feedbot_trajectory_logic::MoveToPose::Request &req, feedbot_trajectory_logic::MoveToPose::Response &res)
{
  std::vector<double> joint_positions;
  std::vector<std::string> joint_names;
  robot_interface_->GetCurrentAngles(joint_positions, joint_names);
  
  bool at_goal = false;
  trajectory_msgs::JointTrajectory jt;
  float cumulative_time_secs(0);
  trajectory_msgs::JointTrajectoryPoint start_point;
  start_point.positions = joint_positions; 
  ros::Duration start_time(cumulative_time_secs);
  start_point.time_from_start = start_time;
  jt.points.push_back(start_point);
  while (!at_goal)
  {
    //std::cout << "running tracking!" << std::endl;
    try
    {
      JointUpdateResult jur = controller_.plan_step_to_target_pose(joint_positions, req.target);
      joint_positions = jur.joint_positions;
      trajectory_msgs::JointTrajectoryPoint point;
      point.positions = jur.joint_positions; 
      cumulative_time_secs += jur.step_time;
      ros::Duration cur_time(cumulative_time_secs);
      point.time_from_start = cur_time;
      jt.points.push_back(point);
      at_goal = jur.at_target;
    }
    catch(...)
    {
      std::cout << "You hit an error!";
      throw;
    }
  }
  
  jt.joint_names = joint_names;
  // in this case, our timings are fuzzy at best, so fine for the code to speed up the calculated timings above
  bool can_speed_up = true;
  res.joint_trajectory = robot_interface_->SendTrajectory(jt, can_speed_up);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "track_pose_server");
  ros::NodeHandle n;
  bool is_simulation, is_simulate_spoon;
  double step_size_meters;
  std::string robot_type, link_prefix, robot_description_param_name;
  ros::param::param<double>("~step_size_meters", step_size_meters, 0.01);
  ros::param::param<std::string>("~robot_type", robot_type, "niryo");
  ros::param::param<std::string>("~link_prefix", link_prefix, "");
  ros::param::param<std::string>("~robot_description_param_name", robot_description_param_name, "robot_description");

  RobotInterface* robot_interface;
  NiryoRobotParams robot_params;
  if (robot_type == "ur5") { 
    std::cout << "Running code on a UR5 robot";
    UR5RobotParams ur5_robot_params;
    robot_interface = new RosRobotInterface("arm_controller/follow_joint_trajectory", "joint_states", &n, ur5_robot_params);
  } else if (robot_type == "sim") { 
    std::cout << "Echoing joints on rviz gen3 robot";
    UR5RobotParams ur5_robot_params;
    robot_interface = new RosRobotInterface("/execute_trajectory", "/joint_states", &n, ur5_robot_params, robot_description_param_name);
  }
  std::cout << "Waiting 5 sec for RobotInterface in case it's slow to come up";
  ros::Duration(5).sleep();
  std::cout << "Done waiting 5 sec for DomusInterface in case it's slow to come up";
  MoveToPoseService move_to_pose_service(step_size_meters, robot_interface, &n, link_prefix, robot_description_param_name);
  ros::ServiceServer service = n.advertiseService("move_to_pose", &MoveToPoseService::move_to_pose, &move_to_pose_service);
  ros::ServiceServer angle_service = n.advertiseService("move_to_joint_angles", &MoveToPoseService::move_to_joint_angles, &move_to_pose_service);
  ros::spin();
  delete robot_interface;
  return 0;
}
