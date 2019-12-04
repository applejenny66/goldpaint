#ifndef _ADLINK_ERROR_CODE_DEF_H
#define _ADLINK_ERROR_CODE_DEF_H

#define ERR_NoError						(0)		//No Error	

// System Error ( -1 ~ -1000 )
#define ERR_OSVersion					(-1)	// Operation System type mismatched
#define ERR_OpenDriverFailed			(-2)	// Open device driver failed - Create driver interface failed
#define ERR_InsufficientMemory			(-3)	// System memory insufficiently
#define ERR_DeviceNotInitial			(-4)	// Cards not be initialized
#define ERR_NoDeviceFound				(-5)	// Cards not found(No card in your system)
#define ERR_CardIdDuplicate				(-6)	// Cards' ID is duplicated. 
#define ERR_DeviceAlreadyInitialed		(-7)	// Cards have been initialed 
#define ERR_InterruptNotEnable			(-8)	// Cards' interrupt events not enable or not be initialized
#define ERR_TimeOut						(-9)	// Function time out
#define ERR_ParametersInvalid			(-10)	// Function input parameters are invalid
#define ERR_SetEEPROM					(-11)	// Set data to EEPROM (or nonvolatile memory) failed
#define ERR_GetEEPROM					(-12)	// Get data from EEPROM (or nonvolatile memory) failed
#define ERR_FunctionNotAvailable		(-13)	// Function is not available in this step, The device is not support this function or Internal process failed
#define ERR_FirmwareError				(-14)   // Firmware error, please reboot the system
#define ERR_CommandInProcess			(-15)	// Previous command is in process
#define ERR_AxisIdDuplicate				(-16)	// Axes' ID is duplicated.
#define ERR_ModuleNotFound				(-17)   // Slave module not found.
#define ERR_InsufficientModuleNo		(-18)	// System ModuleNo insufficiently
#define ERR_HandShakeFailed				(-19)   // HandSake with the DSP out of time.
#define ERR_FILE_FORMAT					(-20)	// Config file format error.(cannot be parsed)
#define ERR_ParametersReadOnly			(-21)	// Function parameters read only.
#define ERR_DistantNotEnough			(-22)	// Distant is not enough for motion.
#define ERR_FunctionNotEnable			(-23)	// Function is not enabled.
#define ERR_ServerAlreadyClose		    (-24)	// Server already closed.
#define ERR_DllNotFound					(-25)	// Related dll is not found, not in correct path.
//Following are added for AI16AO2 calibration
#define ERR_TrimDAC_Channel				(-26)
#define ERR_Satellite_Type				(-27)
#define	ERR_Over_Voltage_Spec			(-28)
#define ERR_Over_Current_Spec			(-29)
#define ERR_SlaveIsNotAI				(-30)
#define ERR_Over_AO_Channel_Scope		(-31)
#define ERR_DllFuncFailed				(-32)	// Failed to invoke dll function. Extension Dll version is wrong.
#define ERR_FeederAbnormalStop		    (-33) //Feeder abnormal stop, External stop or feeding stop
#define ERR_AlreadyClose				(-34)	// Already closed by others
#define ERR_NullObject					(-35)	// Null object is detected
#define ERR_PreMoveErr					(-36)	// last move is on error stop
#define ERR_PreMoveNotDone			    (-37)	// last move not be done
#define ERR_MismatchState				(-38)	// there is a mismatch state
#define ERR_Read_ModuleType_Dismatch	(-39)
#define ERR_DoubleOverflow			    (-40) // Double format parameter is overflow
#define ERR_SlaveNumberErr			    (-41)
#define ERR_SlaveStatusErr			    (-42)
#define ERR_MapPDOOffset_TimeOut        (-43)
#define ERR_Fifo_Access_Fail            (-44)
#define ERR_KernelVerifyError			(-45)   // Kernel verify error, please download kernel again



//for EMX-100
#define ERR_Emx_Offset					(-899)
#define ERR_Smp_App_Existed				(-900)
#define ERR_Init_Error					(-901)
#define ERR_No_Init						(-902)
#define ERR_No_Setup					(-903)
#define ERR_Input_Error					(-904)
#define ERR_Status_Not_Ready			(-905)
#define ERR_Axis_Busy					(-906)
#define ERR_Network_Error				(-907)
#define ERR_Network_Time_Out			(-908)
#define ERR_Crc_Fail					(-909)
#define ERR_Param_Invalid 				(-910)
#define ERR_No_Servo_On					(-911)
#define ERR_Api_Timeout					(-912)
#define ERR_Load_Xml_Mismatch 			(-915)




#define ERR_Fifo_Access_Success         (1)

//For EtherCAT, PCIe-8334_8 ( -1001 ~ -1999 )
#define ERR_NoENIFile						(-1001)
#define ERR_TimeOut_SetVoltageEnable		(-1002)
#define ERR_TimeOut_SetReadyToSwitch		(-1003)
#define ERR_TimeOut_SetShutdown				(-1004)
#define ERR_TimeOut_SetSwitchOn				(-1005)
#define ERR_TimeOut_SetOperationEnable		(-1006)
#define ERR_RegistryPath					(-1007)
#define ERR_MasterNotOPState				(-1008)
#define ERR_SlaveNotOPState					(-1009)
#define ERR_SlaveTotalAxisNumber			(-1010) // The scanned number of EtherCAT slaves' axes exceeds the number of max.
#define ERR_MissESIFileOrMissENIPath		(-1011) // No ESI file or ESI file path miss result .

//For Kernel update
#define ERR_KernelUpdateError	(-50)
//for general functions
#define ERR_KernelGeneralFunc	(-51) 

#define ERR_Win32Error					(-1000) // No such INT number, or WIN32_API error, contact with ADLINK's FAE staff.
#define ERR_LinuxError                  (-1500) // No such INT number, or LINUX_API error, contact with ADLINK's FAE staff.

// DSP Error ( -2001 ~ -3000 ) //Defined in manual
#define ERR_DspStart					(-2000) // The base for DSP error

#endif  //_ADLINK_ERROR_CODE_DEF_H
