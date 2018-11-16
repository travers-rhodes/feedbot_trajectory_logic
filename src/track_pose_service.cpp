#include <feedbot_trajectory_logic/track_pose_service.h>

TrackPoseService::TrackPoseService(double update_rate_hz, double step_size_meters, DomusInterface* domus_interface, ros::NodeHandle* n) : controller(step_size_meters, domus_interface, n), _update_rate_hz(update_rate_hz)
{
  dist_pub_ = n->advertise<std_msgs::Float64>("/distance_to_target", 1);

  is_active = false;
}

// This method only returns when !ros::ok()
// it continually moves the robot in a step toward the last requested target
// (or doesn't, if the last request was to stop the robot)
void TrackPoseService::run_tracking()
{
  ros::Rate loop_rate(_update_rate_hz);
  while (ros::ok())
  {
    //std::cout << "running tracking!" << std::endl;
    try
    {
      std_msgs::Float64 msg;
      if(is_active)
      {
        msg.data = controller.make_step_to_target_pose(target_pose);
      }
      else
      {
        //if we aren't moving, then we will never arrive
        msg.data = 1.0;
      }
      dist_pub_.publish(msg); 
    }
    catch(...)
    {
      std::cout << "You hit an error!";
      throw;
    }
    loop_rate.sleep();
  }
}

// this method updates the target for the service to move the robot to
bool TrackPoseService::handle_target_update(feedbot_trajectory_logic::TrackPose::Request &req,
                          feedbot_trajectory_logic::TrackPose::Response &res)
{
  if(req.stopMotion)
  {
    is_active = false;
  }
  else
  {
    geometry_msgs::Pose target_copy(req.target);
    target_pose = target_copy;
    is_active = true;
  }
  res.success = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "track_pose_server");
  ros::NodeHandle n;
  bool is_simulation, is_simulate_spoon;
  double update_rate_hz, step_size_meters;
  // how frequently do we send a (possibly new) target to the jacobian_controller
  // (which itself has a timer for how frequently to send commands to domus)
  // don't forget to set a default value for these, in case you start from the command line! :)
  ros::param::param<double>("~update_rate_hz", update_rate_hz, 10);
  ros::param::param<double>("~step_size_meters", step_size_meters, 0.01);

  DomusInterface* domus_interface;
  domus_interface = new NiryoInterface();
  ros::AsyncSpinner spinner(1); // use 1 thread async for callbacks
  spinner.start();
  std::cout << "Waiting for DomusInterface in case it's slow to come up";
  ros::Duration(5).sleep();
  TrackPoseService trackPoseService(update_rate_hz, step_size_meters, domus_interface, &n);
  std::cout << "Waiting for trackPoseService in case it's slow to come up" << std::endl;
  ros::Duration(5).sleep();
  ros::ServiceServer service = n.advertiseService("update_pose_target", &TrackPoseService::handle_target_update, &trackPoseService);
  trackPoseService.run_tracking();

  delete domus_interface;
  return 0;
}
