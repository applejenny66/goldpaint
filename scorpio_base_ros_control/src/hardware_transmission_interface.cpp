#include "scorpio_base_ros_control/hardware_transmission_interface.h"
#include <sstream>
#include <stdlib.h>
#include <ros/ros.h>

#define PI 3.1415926

#define G_GEAR_RATIO_R 20.25
#define G_GEAR_RATIO_L 20.25
#define ENC_FULL_R 4096 
#define ENC_FULL_L 4096 

HwTmIntf::HwTmIntf(
  std::vector<std::string> jnt_names,
  int update_freq)
  : update_freq_(update_freq)
{
  jnt_names_ = jnt_names;

  // Initialize Gear Ratios
  //for(size_t i = 0; i < 2; i++)
  tm_data_.trans_.emplace_back(G_GEAR_RATIO_R, 0.0);
  tm_data_.trans_.emplace_back(G_GEAR_RATIO_L, 0.0);
  
  // Initialize hardware data
  hw_data_init_();

  // Initialize transmission data
  tm_data_init_();

  // Register hardware data
  hw_register_();

  // Wrap transmission data
  tm_wrap_();

  // Register transmission data
  tm_register_();

  // Initialize serial port
  scorpio_port.set_port("/dev/ttyS0");
  scorpio_port.set_baudRate(9600);
  scorpio_port.set_dataBits(8);
  scorpio_port.set_stopBit(1);
  scorpio_port.set_parity(false);
  scorpio_port.set_hardwareFlowControl(false);

  scorpio_port.homing(1);
  scorpio_port.homing(2);
  scorpio_port.switchOn(1);
  scorpio_port.switchOn(2);
}


HwTmIntf::~HwTmIntf()
{
  scorpio_port.switchOff(1);
  scorpio_port.switchOff(2);
}


void HwTmIntf::update()
{
  static std::stringstream ss_cmd_vel_;
  static std::stringstream ss_curr_vel_;
  static bool init = true;


  tm_data_.act_to_jnt_state_.propagate();
  tm_data_.jnt_to_act_state_.propagate();

#if 0
  std::cout << "Write : " << std::endl;
  std::cout << "1v" << hw_data_.act_cmd_vel_[0] * (30 / PI)<< " rpm" << std::endl;
  std::cout << "2v" << hw_data_.act_cmd_vel_[1] * (30 / PI)<< " rpm" << std::endl;
  std::cout << "=======================================\n" << std::endl;
#endif

  // Write data to motors
  //scorpio_port.update((hw_data_.act_cmd_vel_[0] * (30 / PI)), -(hw_data_.act_cmd_vel_[1] * (30 / PI)));
  scorpio_port.writeSpeed(1,  (hw_data_.act_cmd_vel_[0] * (30 / PI))); // Right wheel (node 1)(rpm)
  scorpio_port.writeSpeed(2, -(hw_data_.act_cmd_vel_[1] * (30 / PI))); // Left  wheel (node 2)(rpm)

  // Read data from motors 
  //scorpio_port.readEnc(1);
  //scorpio_port.readEnc(2);
  try
  {
    hw_data_.act_curr_pos_[0] =  ((float)scorpio_port.readEnc(1) / ENC_FULL_R) * (2 * PI);
    hw_data_.act_curr_pos_[1] = -((float)scorpio_port.readEnc(2) / ENC_FULL_L) * (2 * PI);
  }
  catch(const std::runtime_error &e)
  {
    ROS_WARN("Encoder response timeout.");
    //std::cerr << e.what() << std::endl;
  }


  // Fake reading
  //hw_data_.act_curr_pos_[0] += hw_data_.act_cmd_vel_[0] * this->get_period().toSec(); 
  //hw_data_.act_curr_pos_[1] += hw_data_.act_cmd_vel_[1] * this->get_period().toSec(); 

#if 0
  std::cout << "Read rwheel position: " << hw_data_.act_curr_pos_[0] << " rad" << std::endl;
  std::cout << "Read lwheel position: " << hw_data_.act_curr_pos_[1] << " rad" << std::endl;
  std::cout << "=======================================\n" << std::endl;
#endif

}


ros::Time HwTmIntf::get_time() const
{
  return ros::Time::now();
}


ros::Duration HwTmIntf::get_period() const
{
  return ros::Duration(1.0 / update_freq_);
}

