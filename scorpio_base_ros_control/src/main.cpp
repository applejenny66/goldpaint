#include "scorpio_base_ros_control/hardware_transmission_interface.h"
#include <ros/ros.h>
#include <string>
#include <boost/thread/thread.hpp>
#include <controller_manager/controller_manager.h>

int main(int argc, char **argv)
{
  // initialize ros
  ros::init(argc, argv, "scorpio_ros_control");
  ros::NodeHandle nh;

  // Set joint names (according to urdf)
  std::vector<std::string> jnt_names;
  jnt_names.push_back("right_wheel_joint");
  jnt_names.push_back("left_wheel_joint");

  // Set update frequency (Hz)
  int update_freq = 10;
  
  // Hardware transmission interface
  HwTmIntf scorpio(jnt_names, update_freq);

  // Controller manager
  controller_manager::ControllerManager cm(&scorpio, nh);

#if 1
  ros::Rate rate(update_freq);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while(ros::ok())
  {
    scorpio.update();
    cm.update(scorpio.get_time(), scorpio.get_period());
    rate.sleep();
  }

  spinner.stop();

  return 0;
#endif
}
