#include <ros/ros.h>
#include <vector>
#include <string>
#include <iostream>
#include <controller_manager/controller_manager.h>
#include <std_msgs/String.h>

#include "scorpio_arm_ros_control/common.h"
#include "scorpio_arm_ros_control/hardware_transmission_interface.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scorpio_arm_ros_control");
  ros::NodeHandle nh;

  int update_freq;
  std::vector<int> gear_ratios;
  std::vector<std::string> jnt_names;

  get_rosparam(nh, update_freq, gear_ratios, jnt_names);
  std::cout << "==========update freq is " << update_freq << "==========" << std::endl;

  HwTmIntf scorpio_arm(gear_ratios, jnt_names, update_freq);
  controller_manager::ControllerManager cm(&scorpio_arm, nh);
  scorpio_arm.TorqueAdvertise(nh);

  ros::Rate rate(update_freq);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while(ros::ok())
  {
    scorpio_arm.update();
    cm.update(scorpio_arm.get_time(), scorpio_arm.get_period());
    rate.sleep();
  }

  spinner.stop();

  return 0;
}
