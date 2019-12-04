#ifndef HARDEARE_TRANSMISSION_INTERFACE
#define HARDEARE_TRANSMISSION_INTERFACE

#include "scorpio_base_ros_control/hardware_data.h"
#include "scorpio_base_ros_control/transmission_data.h"
#include "scorpio_uart/scorpio_uart.h"
#include <hardware_interface/robot_hw.h>

class HwTmIntf: public hardware_interface::RobotHW
{
  public:
    HwTmIntf(std::vector<std::string>, int);
    ~HwTmIntf();
    void update();
    ros::Time get_time() const;
    ros::Duration get_period() const;

  private:
    void hw_data_init_();
    void tm_data_init_();
    void hw_register_();
    void tm_register_();
    void tm_wrap_();
    void read_odom_once_();

    HwData hw_data_;
    TmData tm_data_;
    SerialHandle scorpio_port;

    std::vector<std::string> jnt_names_;
    int update_freq_;
};

#endif
