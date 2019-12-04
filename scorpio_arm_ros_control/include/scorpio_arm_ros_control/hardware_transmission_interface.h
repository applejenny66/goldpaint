#ifndef HARDEARE_TRANSMISSION_INTERFACE
#define HARDEARE_TRANSMISSION_INTERFACE

#include "scorpio_arm_ros_control/hardware_data.h"
#include "scorpio_arm_ros_control/transmission_data.h"
#include "scorpio_uart/scorpio_uart.h"
#include <hardware_interface/robot_hw.h>

#include "MMCPP/OS_PlatformDependSetting.hpp"
#include "MMCPP/MMCPPlib.hpp"
#include "MMC_APP/MMC_definitions.h"

#include "std_msgs/Bool.h"
#include <moveit/move_group_interface/move_group_interface.h>

class HwTmIntf: public hardware_interface::RobotHW
{
  public:
    HwTmIntf(std::vector<int>, std::vector<std::string>, int);
    ~HwTmIntf();
    void update();
    void set_n_dof();
    ros::Time get_time() const;
    ros::Duration get_period() const;
    ELMO_INT32 Init_Connection(void);

    void EnableAll();
    void DisableAll();
    void ResetAll();
    void ChangeOpModeAll();
    void InitPose();
    void InitPoseFake();
    void MotorStuckFix();
    void TorqueAdvertise(ros::NodeHandle);

  private:
    void hw_data_init_();
    void tm_data_init_();
    void hw_register_();
    void tm_register_();
    void tm_wrap_();
    //ros::Time get_time_() const;
    //ros::Duration get_period_(int) const;

    HwData hw_data_;
    TmData tm_data_;

    std::vector<int> gear_ratios_;
    std::vector<std::string> jnt_names_;
    int n_dof_;
    int update_freq_;

    int jnt_num;
    int start_jnt;
    int bypass_count;
    //int base_ignore_count;
    ros::Publisher TorquePub;
    double torque_average[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double count_average[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double torque_threshold = 5.0;
    double count_threshold = 0.05;
    MMC_CONNECT_HNDL gConnHndl ;
    CMMCConnection gConn ;
    CMMCSingleAxis cAxis[6];
    const int cAxisBias[6]  = {21189, 15872, 106947, 98955, 60213, 72376};
    const int cAxisInit[6]  = {21189, 32144, 74888, 98970, 57546, 72376};
    const int cAxisVel[6]  = {10000, 5000, 10000, 20000, 10000, 20000};
    CMMCGroupAxis cGrpRef;
    //CMMCHostComm cHost;
    //MMC_MOTIONPARAMS_SINGLE stSingleDefault ;
    MMC_MOTIONPARAMS_GROUP G_Mparam;

    RTE_CLBKP pRTEClbk;

    // For base
    SerialHandle scorpio_port;
};

#endif
