#include "scorpio_arm_ros_control/hardware_transmission_interface.h"

#define PI 3.1415926
//#define ENC_FULL 8388608
#define ENC_FULL 131072
#define ENC_FULL_R 4096
#define ENC_FULL_L 4096

ELMO_INT32 OnRunTimeError(
    const ELMO_INT8* msg,
    MMC_CONNECT_HNDL uiConnHndl,
    ELMO_UINT16 usAxisRef,
    ELMO_INT16 sErrorID,
    ELMO_UINT16 usStatus)
{

    printf("\n  %s, Axis_ref=%d, ErrId=%d, status=%d \n", msg, usAxisRef, sErrorID, usStatus);
    MMC_CloseConnection(uiConnHndl);

    exit(0);
}

void Exception(CMMCException exp)
{
    std::cout << "function " << exp.what() << std::endl;
    std::cout << "axis " << exp.axisRef() << " error: " << exp.error() << std::endl;
    exit(0);
}

HwTmIntf::HwTmIntf(
    std::vector<int> gear_ratios,
    std::vector<std::string> jnt_names,
    int update_freq)
    : n_dof_(jnt_names.size()),
      update_freq_(update_freq)
{
    gear_ratios_ = gear_ratios;
    jnt_names_ = jnt_names;

    // Initialize gear ratios
    for(size_t i = 0; i < gear_ratios.size(); i++)
        tm_data_.trans_.emplace_back(gear_ratios_[i], 0.0);

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

    //**** For arm ****//
    // Start ethercat master
    //Initial_StartFieldBus();
    jnt_num = 2;
    jnt_num = 6;
    start_jnt = jnt_num-1;
  	start_jnt = 0;
  	bypass_count = 0;
    //base_ignore_count = 0;

#if 1
    Init_Connection();

    ResetAll();
    ChangeOpModeAll();
    EnableAll();
	  //MotorStuckFix();
    //InitPose();
    std::cout << "ALL DONE" << std::endl;
#endif
#if 0
    //**** For base ****//
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
#endif
}

HwTmIntf::~HwTmIntf()
{
    //**** For base ****//
    //scorpio_port.switchOff(1);
    //scorpio_port.switchOff(2);

	//**** For arm ****//
    DisableAll();
}

ELMO_INT32 HwTmIntf::Init_Connection(void)
{
    ELMO_INT32 iEventMask = 0x7FFFFFFF;
    const ELMO_PINT8 cHostIP= (char*)"192.168.1.5";
    const ELMO_PINT8 cDestIP= (char*)"192.168.1.3";

    // Set Try-Catch flag Enable\Disable
    CMMCPPGlobal::Instance()->SetThrowFlag(true,false);
    CMMCPPGlobal::Instance()->SetThrowWarningFlag(false);

    //create connection
    gConnHndl = gConn.ConnectRPCEx(cHostIP, cDestIP, iEventMask, (MMC_MB_CLBK)NULL);
    gConn.GetVersion();

    // Register Run Time Error Callback function
    pRTEClbk = (RTE_CLBKP)OnRunTimeError;
    CMMCPPGlobal::Instance()->RegisterRTE(pRTEClbk);

    // Axes initialization
  	cAxis[0].InitAxisData("a01", gConnHndl);
  	cAxis[1].InitAxisData("a02", gConnHndl);
  	cAxis[2].InitAxisData("a03", gConnHndl);
  	cAxis[3].InitAxisData("a04", gConnHndl);
  	cAxis[4].InitAxisData("a05", gConnHndl);
  	cAxis[5].InitAxisData("a06", gConnHndl);

#if 0
  	// Group initialization
  	//cGrpRef.InitAxisData("v01",gConnHndl);

  	// Set default params
  	for(int id = this->start_jnt; id < this->jnt_num; id++)
  		G_Mparam.dEndPoint[id] = cAxisBias[id];

  	G_Mparam.eBufferMode = MC_ABORTING_MODE;
  	G_Mparam.eTransitionMode = MC_TM_NONE_MODE;
  	G_Mparam.eCoordSystem = MC_ACS_COORD;
  	//G_Mparam.fTransitionParameter[0] = 1;
  	//G_Mparam.fTransitionParameter[1] = 1;
  	G_Mparam.fVelocity = 10000;         //TODO
  	G_Mparam.fAcceleration = 100000;    //TODO
  	G_Mparam.fDeceleration = 100000;    //TODO
  	G_Mparam.fJerk = 2000000;
  	//G_Mparam.iExecDelayMs = 0;
  	//G_Mparam.bExecute = 1;

    try
    {
	    //cGrpRef.SetDefaultParams(G_Mparam);
    }
    catch(CMMCException exp)
    {
        Exception(exp);
    }
#endif

    return 0;
}

void HwTmIntf::EnableAll()
{
    try
    {
        for(int id = start_jnt; id < jnt_num; id++)
        {
            std::cout << "axis " << id << " status is "
                      << (cAxis[id].ReadStatus()& NC_AXIS_STAND_STILL_MASK) << "(STAND_STILL) and "
                      << (cAxis[id].ReadStatus()& NC_AXIS_DISCRETE_MOTION_MASK) << "(DISCRETE_MOTION)" << std::endl;
            if (!(cAxis[id].ReadStatus()& NC_AXIS_STAND_STILL_MASK))
            {
                cAxis[id].PowerOn();
                while (!(cAxis[id].ReadStatus()& NC_AXIS_STAND_STILL_MASK));
            }
            std::cout << "axis " << id << " status is "
                      << (cAxis[id].ReadStatus()& NC_AXIS_STAND_STILL_MASK) << "(STAND_STILL) and "
                      << (cAxis[id].ReadStatus()& NC_AXIS_DISCRETE_MOTION_MASK) << "(DISCRETE_MOTION)" << std::endl;
        }

        //cGrpRef.GroupEnable();
        //while (!(cGrpRef.ReadStatus() & NC_GROUP_STANDBY_MASK));

        std::cout << "EnableAll Done" << std::endl;
    }
    catch(CMMCException exp)
    {
        Exception(exp);
    }
}

void HwTmIntf::DisableAll()
{
	for(int id = start_jnt; id < jnt_num; id++)
    {
        try
        {
            if (!(cAxis[id].ReadStatus()& NC_AXIS_STAND_STILL_MASK))
            {
                cAxis[id].Stop();
                while (!(cAxis[id].ReadStatus()& NC_AXIS_STAND_STILL_MASK));
            }
			if (!(cAxis[id].ReadStatus()& NC_AXIS_DISABLED_MASK))
            {
                cAxis[id].PowerOff();
                while (!(cAxis[id].ReadStatus()& NC_AXIS_DISABLED_MASK));
            }
        }
		catch(CMMCException exp)
    	{
        	if (cAxis[id].ReadStatus() & NC_AXIS_ERROR_STOP_MASK)
            {
                cAxis[id].Reset();
                while( !(cAxis[id].ReadStatus() & NC_AXIS_DISABLED_MASK));
            }
    	}

        //cGrpRef.GroupDisable();
        //while (!(cGrpRef.ReadStatus() & NC_GROUP_DISABLED_MASK));

	}
	std::cout << "DisableAll Done" << std::endl;
}

void HwTmIntf::ResetAll()
{
	try
    {
		std::cout << "ResetAll" << std::endl;
        for(int id = start_jnt; id < jnt_num; id++)
        {
            if (cAxis[id].ReadStatus() & NC_AXIS_ERROR_STOP_MASK)
            {
                cAxis[id].Reset();
                while( !(cAxis[id].ReadStatus() & NC_AXIS_DISABLED_MASK));
            }
        }

#if 0
        if (cGrpRef.ReadStatus() & NC_GROUP_ERROR_STOP_MASK)
        {
            cGrpRef.GroupReset();
            while (!(cGrpRef.ReadStatus() & NC_GROUP_DISABLED_MASK));
        }
#endif

        std::cout << "ResetAll Done" << std::endl;
    }
    catch(CMMCException exp)
    {
        Exception(exp);
    }
}

void HwTmIntf::ChangeOpModeAll()
{
	try
    {
        for(int id = start_jnt; id < jnt_num; id++)
        {
            //axis not in profile position mode - change it to profile position
            if (cAxis[id].GetOpMode() != OPM402_PROFILE_POSITION_MODE)
            {
                cAxis[id].SetOpMode(OPM402_PROFILE_POSITION_MODE);

                //wait till operation mode is profile position
                while( cAxis[id].GetOpMode() != OPM402_PROFILE_POSITION_MODE);
            }
        }

        std::cout << "ChangeOpModeAll Done" << std::endl;
    }
    catch(CMMCException exp)
    {
        Exception(exp);
    }
}


void HwTmIntf::update()
{
#if 1
    static ELMO_DOUBLE pos[6];
    tm_data_.act_to_jnt_state_.propagate();
    tm_data_.jnt_to_act_state_.propagate();
    //std::cout << bypass_count << std::endl;
    //**** For arm ****//
    //std::cout << "======= Write ========" << std::endl;
    // Write data to motors
    try
    {
        if (bypass_count++ > 100)
        {
            for(int id = start_jnt; id < jnt_num; id++)
            {
                double targetPos = cAxisBias[id] + hw_data_.act_cmd_pos_[id] * (ENC_FULL / (2 * PI));
                //std::cout << "axis" << id << " to " << targetPos << std::endl;
                //std::cout << "bias " << cAxisBias[id] << " cmd_pos " << hw_data_.act_cmd_pos_[id] * (ENC_FULL / (2 * PI)) << std::endl;
                //G_Mparam.dEndPoint[id] = targetPos;

                //std::cout << "Ready to move" << std::endl;
                cAxis[id].MoveAbsolute(targetPos, cAxisVel[id], 20000, 20000, 2000000, MC_ABORTING_MODE);
            }
        }
#if 0
          if ((cGrpRef.GroupReadStatus() & NC_GROUP_STANDBY_MASK))
          {
              std::cout << "move" << std::endl;
              cGrpRef.MoveLinearAbsolute(G_Mparam.fVelocity, G_Mparam.dEndPoint);
          }
          else
          {
              //std::cout << "don't move" << std::endl;
          }
#endif
    }
    catch(CMMCException exp)
    {
        //Skip Maestro warning 1008
        //This is because destination is too close to origin
        //Exception(exp);
    }
    //std::cout << "======= Read ========" << std::endl;
    //cGrpRef.GroupReadActualPosition(MC_ACS_COORD, pos);
    // Read data from motors

      //cout << "axis" << id << " at " << pos[id] << std::endl;
      //hw_data_.act_curr_pos_[id] = pos[id] * (2 * PI / ENC_FULL);
      //cout << "axis" << id << " at " << cAxis[id].GetActualPosition() << std::endl;
      //cout << "axis" << id << " tor " << cAxis[id].GetActualTorque() << std::endl;
#if 1
    for(int id = start_jnt; id < 3; id++)
    {
      //std::cout << "axis" << id << std::endl;
      //std::cout << cAxis[id].GetActualTorque() << std::endl;
      //std::cout << torque_average[id] <<std::endl;
      //std::cout << torque_threshold << std::endl;
      //std::cout << "axis" << id <<" touched!" << abs(cAxis[id].GetActualTorque() - torque_average[id]) << " " << count_average[id] << std::endl;
      double touched = 0.0;
      if (abs(cAxis[id].GetActualTorque() - torque_average[id]) > torque_threshold)
      {
        std::cout << "axis" << id <<" touched!" << abs(cAxis[id].GetActualTorque() - torque_average[id]) << " " << count_average[id] << std::endl;
        touched = 1.0;
        if (count_average[id] > count_threshold)
        {
          std_msgs::Bool msg;
          msg.data = true;
          TorquePub.publish(msg);
        }
      }
      //std::cout << "axis" << id << std::endl;
      torque_average[id] = (cAxis[id].GetActualTorque() + 49 * torque_average[id]) / 50;
      count_average[id] = (touched + 9 * count_average[id]) / 10;
    }
#endif
    for(int id = start_jnt; id < jnt_num; id++)
    {
      hw_data_.act_curr_pos_[id] = (cAxis[id].GetActualPosition() - cAxisBias[id]) * (2 * PI / ENC_FULL);
    }

    // Read actuator encoder (fake)
    //for(int i = 0; i < jnt_names_.size(); i++)
        //hw_data_.act_curr_pos_[i] = hw_data_.act_cmd_pos_[i];
#endif

#if 0
    //**** For base ****//
    //if(base_ignore_count++ > 4){
      scorpio_port.writeSpeed(1,  (hw_data_.act_cmd_vel_[n_dof_ - 2] * (30 / PI))); // Right wheel (node 1)(rpm)
      scorpio_port.writeSpeed(2, -(hw_data_.act_cmd_vel_[n_dof_ - 1] * (30 / PI))); // Left  wheel (node 2)(rpm)

      try
      {
        hw_data_.act_curr_pos_[n_dof_ - 2] =  ((float)scorpio_port.readEnc(1) / ENC_FULL_R) * (2 * PI);
        hw_data_.act_curr_pos_[n_dof_ - 1] = -((float)scorpio_port.readEnc(2) / ENC_FULL_L) * (2 * PI);
      }
      catch(const std::runtime_error &e)
      {
        ROS_WARN("Encoder response timeout.");
        //std::cerr << e.what() << std::endl;
      }
      //base_ignore_count = 0;
    //}
#endif
}

void HwTmIntf::MotorStuckFix()
{
	for(int id = start_jnt; id < jnt_num; id++)
	{
		std::cout << "Motor stuck test: axis" << id << std::endl;
		try
		{
			cAxis[id].MoveRelative(1000, 5000, 100000, 100000, 2000000, MC_BUFFERED_MODE);
			while (!(cAxis[id].ReadStatus()& NC_AXIS_STAND_STILL_MASK));
			cAxis[id].MoveRelative(-1000, 5000, 100000, 100000, 2000000, MC_BUFFERED_MODE);
			while (!(cAxis[id].ReadStatus()& NC_AXIS_STAND_STILL_MASK));
		}
		catch(CMMCException exp)
		{
        	Exception(exp);
			cAxis[id].MoveRelative(1000, 5000, 100000, 100000, 2000000, MC_BUFFERED_MODE);
			while (!(cAxis[id].ReadStatus()& NC_AXIS_STAND_STILL_MASK));
			cAxis[id].MoveRelative(-1000, 5000, 100000, 100000, 2000000, MC_BUFFERED_MODE);
			while (!(cAxis[id].ReadStatus()& NC_AXIS_STAND_STILL_MASK));
		}
		ros::Duration(1, 0).sleep();
	}
}

void HwTmIntf::InitPoseFake()
{
    try
    {
		  for(int id = start_jnt; id < jnt_num; id++)
		  {
			  cAxis[id].MoveAbsolute(cAxisInit[id], cAxisVel[id], 20000, 20000, 2000000, MC_ABORTING_MODE);
			}
		}
    catch(CMMCException exp)
    {
        //Skip Maestro warning 1008
        //This is because destination is too close to origin
        Exception(exp);
    }
    for(int id = start_jnt; id < jnt_num; id++)
    {
      std::cout << "axis" << id << " stand still" << std::endl;
      while (!(cAxis[id].ReadStatus()& NC_AXIS_STAND_STILL_MASK));
    }
}

void HwTmIntf::TorqueAdvertise(ros::NodeHandle nh)
{
    TorquePub = nh.advertise<std_msgs::Bool>("/Collid", 100);
}

ros::Time HwTmIntf::get_time() const
{
    return ros::Time::now();
}

ros::Duration HwTmIntf::get_period() const
{
    return ros::Duration(1.0 / update_freq_);
}
