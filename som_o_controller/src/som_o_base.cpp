/******************************
 *  Som-O Base Driver node
 *  Obodroid Corporation Co,.Ltd
 *  Author : Theppasith Nisitsukcharoen
 *  Date : 20-Jan-2017
 *******************************/
#include "som_o_controller/som_o_hardware.h"
#include "controller_manager/controller_manager.h"
#include "ros/callback_queue.h"
#include <boost/chrono.hpp>

typedef boost::chrono::steady_clock time_source;

/**
* Control loop for Husky, not realtime safe
*/
void controlLoop(som_o_base::SomOHardware &som_o,
                 controller_manager::ControllerManager &cm,
                 time_source::time_point &last_time)
{

  // Calculate monotonic time difference
  time_source::time_point this_time = time_source::now();
  boost::chrono::duration<double> elapsed_duration = this_time - last_time;
  ros::Duration elapsed(elapsed_duration.count());
  last_time = this_time;

  // Process control loop
  
  som_o.updateJointsFromHardware();
  cm.update(ros::Time::now(), elapsed);
  husky.writeCommandsToHardware();
}

/**
* Diagnostics loop for Husky, not realtime safe
*/
void diagnosticLoop(husky_base::HuskyHardware &husky)
{
  husky.updateDiagnostics();
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "husky_base");
  ros::NodeHandle nh, private_nh("~");

  double control_frequency, diagnostic_frequency;
  private_nh.param<double>("control_frequency", control_frequency, 10.0);

  // Initialize robot hardware and link to controller manager
  husky_base::HuskyHardware husky(nh, private_nh, control_frequency);
  controller_manager::ControllerManager cm(&husky, nh);

  // Setup separate queue and single-threaded spinner to process timer callbacks
  // that interface with Husky hardware - libhorizon_legacy not threadsafe. This
  // avoids having to lock around hardware access, but precludes realtime safety
  // in the control loop.
  ros::CallbackQueue husky_queue;
  ros::AsyncSpinner husky_spinner(1, &husky_queue);

  time_source::time_point last_time = time_source::now();
  ros::TimerOptions control_timer(
    ros::Duration(1 / control_frequency),
    boost::bind(controlLoop, boost::ref(husky), boost::ref(cm), boost::ref(last_time)),
    &husky_queue);
  ros::Timer control_loop = nh.createTimer(control_timer);

  ros::TimerOptions diagnostic_timer(
    ros::Duration(1 / diagnostic_frequency),
    boost::bind(diagnosticLoop, boost::ref(husky)),
    &husky_queue);
  ros::Timer diagnostic_loop = nh.createTimer(diagnostic_timer);

  husky_spinner.start();

  // Process remainder of ROS callbacks separately, mainly ControlManager related
  ros::spin();

  return 0;
}
Contact GitHub API Training Shop Blog About
© 2017 GitHub, Inc. Terms Privacy Security Status Help