//
// This class connects to DOMUS 
// and can be used to send target angles for DOMUS to move to
//
#include "feedbot_trajectory_logic/ros_robot_interface.h"
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

RosRobotInterface::RosRobotInterface(std::string follow_joint_trajectory_name, std::string joint_state_name, ros::NodeHandle* nh, CustomRobotParams robot_params, std::string robot_description_param_name) : RobotInterface(robot_params)
{
  follow_joint_trajectory_name_ = follow_joint_trajectory_name;
  joint_state_name_ = joint_state_name;
  nh_ = nh;
  robot_model_loader::RobotModelLoader robot_model_loader_(robot_description_param_name);
  kinematic_model_ = robot_model_loader_.getModel();
  kinematic_state_ = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model_));
}

void
RosRobotInterface::InitializeConnection()
{
  ac_ = std::make_shared<actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction>>(follow_joint_trajectory_name_, true);
  ac_->waitForServer();
}
  
void
RosRobotInterface::GetCurrentAngles(std::vector<double> &joint_angles, std::vector<std::string> &joint_names)
{
  //https://answers.ros.org/question/73258/detecting-timeout-from-waitformessage/
  boost::shared_ptr<sensor_msgs::JointState const> js = ros::topic::waitForMessage<sensor_msgs::JointState>(joint_state_name_, ros::Duration(10));
  if (js == NULL) {
    throw;
  }
  std::copy(js->position.begin(), js->position.begin() + joint_names_.size(), std::back_inserter(joint_angles));
  joint_names = joint_names_;
}

trajectory_msgs::JointTrajectory
interpolate_trajectory(const trajectory_msgs::JointTrajectory& joint_trajectory, double speedup_factor, double t_delta = 0.001, bool pos_only = false)
{
  double epsilon = 0.00001;

  int num_waypoints = joint_trajectory.points.size();
  double total_duration = joint_trajectory.points[num_waypoints-1].time_from_start.toSec();

  trajectory_msgs::JointTrajectory hyper_joint_trajectory;
  std::vector<trajectory_msgs::JointTrajectoryPoint> hyper_points;
  int past_index = 0;
  int fut_index = 1;
  int max_index = num_waypoints - 1;
  std::vector<double> past_positions = joint_trajectory.points[past_index].positions;
  std::vector<double> fut_positions = joint_trajectory.points[fut_index].positions;
  std::vector<double> past_vel, fut_vel, past_acc, fut_acc;
  if (!pos_only)
  {
    past_vel = joint_trajectory.points[past_index].velocities;
    fut_vel = joint_trajectory.points[fut_index].velocities;
    past_acc = joint_trajectory.points[past_index].accelerations;
    fut_acc = joint_trajectory.points[fut_index].accelerations;
  }
  double past_time = joint_trajectory.points[past_index].time_from_start.toSec();
  double fut_time = joint_trajectory.points[fut_index].time_from_start.toSec();
  // hypersample to get approximate time-stemps, then re-compute time-stamps
  // and then subsample to get our desired interpolation result
  for (double dur = 0; (dur * speedup_factor) < total_duration; dur += t_delta)
  {
    while ((dur * speedup_factor) > fut_time && past_index < max_index)
    {
      past_index++;
      fut_index = std::min(past_index + 1, max_index);
      past_positions = joint_trajectory.points[past_index].positions;
      fut_positions = joint_trajectory.points[fut_index].positions;
      if (!pos_only)
      {
        past_vel = joint_trajectory.points[past_index].velocities;
        fut_vel = joint_trajectory.points[fut_index].velocities;
        past_acc = joint_trajectory.points[past_index].accelerations;
        fut_acc = joint_trajectory.points[fut_index].accelerations;
      }
      past_time = joint_trajectory.points[past_index].time_from_start.toSec();
      fut_time = joint_trajectory.points[fut_index].time_from_start.toSec();
    }
    double t = 0; 
    if ((fut_time - past_time) > epsilon)
    {
      t = ((dur * speedup_factor) - past_time) / (fut_time - past_time);
    }

    trajectory_msgs::JointTrajectoryPoint point;
    for (int j = 0; j < joint_trajectory.joint_names.size(); j++)
    {
      point.positions.push_back(past_positions[j] + t * (fut_positions[j] - past_positions[j]));
      if (!pos_only)
      {
        point.velocities.push_back(past_vel[j] + t * (fut_vel[j] - past_vel[j]));
        point.accelerations.push_back(past_acc[j] + t * (fut_acc[j] - past_acc[j]));
      }
    }
    
    ros::Duration tfs(dur);
    point.time_from_start = tfs;
    hyper_points.push_back(point);
  }
  
  // now, finally, fill out the structure
  hyper_joint_trajectory.joint_names = joint_trajectory.joint_names;
  hyper_joint_trajectory.points = hyper_points;

  return(hyper_joint_trajectory);
}


// send the robot on a whole biglong path 
// can_speed_up parameter indicates whether it's OK for the robot (if it thinks it's safe) to perform the trajectory faster than the time parameterization given by the user
trajectory_msgs::JointTrajectory
RosRobotInterface::SendTrajectory(const trajectory_msgs::JointTrajectory &raw_joint_trajectory, bool can_speed_up)
{
  if (raw_joint_trajectory.points.size() < 2)
  {
    ROS_ERROR_STREAM("You need at least two points in your trajectory to have a vaiid trajectory");
  }
  double original_trajectory_duration = raw_joint_trajectory.points.back().time_from_start.toSec();
  ROS_INFO_STREAM("Raw:");
  ROS_INFO_STREAM("First timestamp after 0 is " << raw_joint_trajectory.points[1].time_from_start << ".");
  ROS_INFO_STREAM("Last timestamp is " << raw_joint_trajectory.points.back().time_from_start << ".");

  // need to set velocity/acceleration points for kinova robot :(
  //trajectory_processing::IterativeParabolicTimeParameterization tp;
  trajectory_processing::TimeOptimalTrajectoryGeneration tp;
  robot_trajectory::RobotTrajectory robot_traj(kinematic_model_, srdf_group_name_);
  std::vector<double> joint_angles;
  std::vector<std::string> joint_names;
  GetCurrentAngles(joint_angles, joint_names);
  kinematic_state_->setJointGroupPositions(srdf_group_name_, joint_angles);  
  robot_traj.setRobotTrajectoryMsg(*kinematic_state_, raw_joint_trajectory);

  if (can_speed_up)
  {
    // if we're allowed to speed up the trajectory, then we should retime first, since
    // hypersampling can result in an artifical boundary on how sped-up the trajectory can go
    tp.computeTimeStamps(robot_traj);
  }
  moveit_msgs::RobotTrajectory robot_traj_msg;
  robot_traj.getRobotTrajectoryMsg(robot_traj_msg);
  
  // comment taken from robotiq_2f_85_move_it_config/launch/ompl_planning_pipeline.launch.xml
  // <!-- PreComputedJointTrajectory Action Server needs 1msec timesteps between waypoints  -->
  double t_delta = 0.001;
  
  // FIRST, we oversample (including extra samples to account for speedup factor), but we do not actually speed up trajectory yet)
  double speedup_factor;
  ros::param::param<double>("~speedup_factor", speedup_factor, 1.0); 
  ROS_INFO_STREAM("the speedup factor is " << speedup_factor<< ".");
  
  bool pos_only = true;
  trajectory_msgs::JointTrajectory hyper_joint_trajectory = interpolate_trajectory(robot_traj_msg.joint_trajectory, 1.0, t_delta / speedup_factor, pos_only);
  robot_traj.setRobotTrajectoryMsg(*kinematic_state_, hyper_joint_trajectory);
  ROS_INFO_STREAM("Hypersampled");
  ROS_INFO_STREAM("First timestamp after 0 is " << hyper_joint_trajectory.points[1].time_from_start << ".");
  ROS_INFO_STREAM("Last timestamp is " << hyper_joint_trajectory.points.back().time_from_start << ".");

  // THEN, we re-time (which may slow down the trajectory, if the original trajectory was dangerously fast)
  tp.computeTimeStamps(robot_traj);
  robot_traj.getRobotTrajectoryMsg(robot_traj_msg);
  ROS_INFO_STREAM("Retimed");
  ROS_INFO_STREAM("First timestamp after 0 is " << robot_traj_msg.joint_trajectory.points[1].time_from_start << ".");
  ROS_INFO_STREAM("Last timestamp is " << robot_traj_msg.joint_trajectory.points.back().time_from_start << ".");

  // note: at this point we have third-party timing instead of our original timing 
  // only look at the the third-party timing if it was slower than our desired timing, or if the user has said we can speed up our timing
  double current_trajectory_duration = robot_traj_msg.joint_trajectory.points.back().time_from_start.toSec();
  double residual_speedup_factor;
  if (can_speed_up || current_trajectory_duration >= original_trajectory_duration) 
  {
    // speedup_factor just applies to whatever the third-party timing returned
    residual_speedup_factor = speedup_factor;
  }
  else
  {
    // otherwise, slow down the trajectory so that the total duration matches our desired 
    residual_speedup_factor = current_trajectory_duration / original_trajectory_duration * speedup_factor;
  }
  ROS_INFO_STREAM("Residual speedup factor is " << residual_speedup_factor << ".");
   
  // FINALLY, we subsample at the desired times.
  pos_only = false;
  trajectory_msgs::JointTrajectory final_joint_trajectory = interpolate_trajectory(robot_traj_msg.joint_trajectory, residual_speedup_factor, t_delta, pos_only);
  ROS_INFO_STREAM("Final");
  ROS_INFO_STREAM("First timestamp after 0 is " << final_joint_trajectory.points[1].time_from_start << ".");
  ROS_INFO_STREAM("Last timestamp is " << final_joint_trajectory.points.back().time_from_start << ".");

  robot_traj.setRobotTrajectoryMsg(*kinematic_state_, final_joint_trajectory);
  robot_traj.getRobotTrajectoryMsg(robot_traj_msg);

  // I should use a mutex (apparently i should use a unique_lock, but i haven't looked into why...) to ensure only
  // one command sent at a time 
  
  moveit_msgs::ExecuteTrajectoryGoal goal;
  goal.trajectory = robot_traj_msg;
  ac_->sendGoal(goal);
  ac_->waitForResult();
  boost::shared_ptr<const moveit_msgs::ExecuteTrajectoryResult> result = ac_->getResult();
  int error_code = result->error_code.val;

  ROS_INFO_STREAM("RosRobotInterface: The error code from calling the follow joint trajectory action was " << error_code << ".");
  if (error_code != moveit_msgs::MoveItErrorCodes::SUCCESS)
  {
    ROS_ERROR_STREAM("RosRobotInterface: The error code from calling the follow joint trajectory action was not SUCCESS.");
  }
  return(goal.trajectory.joint_trajectory);
} 

// move to the target joint_angles and the motion should take you secs seconds.
// return false if we know the robot motion failed
bool
RosRobotInterface::SendTargetAngles(const std::vector<double> &joint_angles, float secs)
{
  for (int i = 0; i < joint_angles.size(); i++) {
    if (joint_angles[i] > max_joint_angles_[i] || joint_angles[i] < min_joint_angles_[i]) 
    {
      ROS_ERROR_STREAM("RosRobotInterface: The requested joint " << i << " was " << joint_angles[i] << " which is past the joint limits.");
      return false;
    }
  }

  trajectory_msgs::JointTrajectory joint_trajectory;
  std::vector<trajectory_msgs::JointTrajectoryPoint> points;

  trajectory_msgs::JointTrajectoryPoint point;
  point.positions = joint_angles;
  point.time_from_start = ros::Duration(secs);
  points.push_back(point);

  // now, finally, fill out the structure
  joint_trajectory.joint_names = joint_names_;
  joint_trajectory.points = points;
  // This method was given a desired time-from-start, so don't adjust the timing.
  bool can_speed_up = false;
  SendTrajectory(joint_trajectory, can_speed_up);
  return true;
}
