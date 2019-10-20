#include <feedbot_trajectory_logic/move_to_pose_service.h>

MoveToPoseService::MoveToPoseService(double step_size_meters, RobotInterface* robot_interface, ros::NodeHandle* n, std::string link_prefix) : controller_(step_size_meters, robot_interface, n, link_prefix)
{
  robot_interface_ = robot_interface;
}

bool MoveToPoseService::move_to_pose(feedbot_trajectory_logic::MoveToPose::Request &req, feedbot_trajectory_logic::MoveToPose::Response &res)
{
  std::vector<double> joint_positions;
  std::vector<std::string> joint_names;
  robot_interface_->GetCurrentAngles(joint_positions, joint_names);
  std::vector<std::vector<double>> joint_locs = plan_path(joint_positions, req.target);
  trajectory_msgs::JointTrajectory jt;
  jt.joint_names = joint_names;
  for (int i = 0; i < joint_locs.size(); i++) {
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = joint_locs[i];
    jt.points.push_back(point);
  }
  res.joint_trajectory = jt;
  return true;
}

std::vector<std::vector<double>> MoveToPoseService::plan_path(std::vector<double> joint_positions, geometry_msgs::Pose target_pose)
{
  bool at_goal = false;
  std::vector<std::vector<double>> points;
  while (!at_goal)
  {
    //std::cout << "running tracking!" << std::endl;
    try
    {
      JointUpdateResult jur = controller_.plan_step_to_target_pose(joint_positions, target_pose);
      joint_positions = jur.joint_positions;
      points.push_back(joint_positions);
      at_goal = jur.at_target;
    }
    catch(...)
    {
      std::cout << "You hit an error!";
      throw;
    }
  }
  return points;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "track_pose_server");
  ros::NodeHandle n;
  bool is_simulation, is_simulate_spoon;
  double step_size_meters;
  std::string robot_type, link_prefix;
  // how frequently do we send a (possibly new) target to the jacobian_controller
  // (which itself has a timer for how frequently to send commands to domus)
  // don't forget to set a default value for these, in case you start from the command line! :)
  ros::param::param<double>("~step_size_meters", step_size_meters, 0.01);
  ros::param::param<std::string>("~robot_type", robot_type, "niryo");
  ros::param::param<std::string>("~link_prefix", link_prefix, "");

  RobotInterface* robot_interface;
  NiryoRobotParams robot_params;
  if (robot_type == "niryo")
  { 
    std::cout << "Running code on a standard Niryo robot";
    robot_interface = new RosRobotInterface("niryo_one_follow_joint_trajectory_controller/follow_joint_trajectory", "joint_states", &n, robot_params);
  } else if (robot_type == "ur5") { 
    std::cout << "Running code on a UR5 robot";
    UR5RobotParams ur5_robot_params;
    robot_interface = new RosRobotInterface("arm_controller/follow_joint_trajectory", "joint_states", &n, ur5_robot_params);
  } else if (robot_type == "gen3") { 
    std::cout << "Running code on a gen3 robot";
    Gen3RobotParams gen3_robot_params;
    robot_interface = new RosRobotInterface("/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory", "my_gen3/joint_states", &n, gen3_robot_params);
  } else if (robot_type == "gen3_rviz") { 
    std::cout << "Echoing joints on rviz gen3 robot";
    Gen3RobotParams gen3_robot_params;
    robot_interface = new JointEchoingInterface(&n, gen3_robot_params);
  } else if (robot_type == "custom_domus") {
    ROS_ERROR_STREAM("Disabled code for custom Domus robot");
    //robot_interface = new CustomDomusInterface(&n, robot_params);
  } else {
    std::cout << "Simulating code without connecting to any robot";
    robot_interface = new JointEchoingInterface(&n, robot_params);
  }
  std::cout << "Waiting 5 sec for DomusInterface in case it's slow to come up";
  ros::Duration(5).sleep();
  std::cout << "Done waiting 5 sec for DomusInterface in case it's slow to come up";
  MoveToPoseService trackPoseService(step_size_meters, robot_interface, &n, link_prefix);
  std::cout << "Waiting for trackPoseService in case it's slow to come up" << std::endl;
  ros::Duration(5).sleep();
  ros::ServiceServer service = n.advertiseService("move_to_pose", &MoveToPoseService::move_to_pose, &trackPoseService);
  ros::spin();
  delete robot_interface;
  return 0;
}
