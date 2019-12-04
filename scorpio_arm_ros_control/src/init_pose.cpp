#include <ros/ros.h>
#include <vector>
#include <string>
#include <iostream>
#include <controller_manager/controller_manager.h>

#include "scorpio_arm_ros_control/common.h"
#include "scorpio_arm_ros_control/hardware_transmission_interface.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "scorpio_motor_stuck_fix");
	ros::NodeHandle nh;

	int update_freq;
	std::vector<int> gear_ratios;
	std::vector<std::string> jnt_names;

	get_rosparam(nh, update_freq, gear_ratios, jnt_names);
	std::cout << "==================" << update_freq << std::endl;

	HwTmIntf scorpio_arm(gear_ratios, jnt_names, update_freq);
	scorpio_arm.InitPoseFake();

	return 0;
}
