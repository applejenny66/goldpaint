////////////////////////////////////////////////////////////////////////////////
/// \file MMC_general_API.h
/// Name        : MMC_general_API.h
/// Author      : Barak R
/// Created on  : April 19, 20010
/// Version     : 0.0.1
///               0.2.0 Updated 20Jan2015 Haim H. native Data Types (names), for supporting 64B OS.
///               0.3.0 updated 12Aug2015 Haim H.
///				  0.4.0 Updated 11Sep2017 Haim H.
/// Copyright   : Your copyright notice
/// Description : This file contain definitions for ...
////////////////////////////////////////////////////////////////////////////////

#ifndef MMC_GENERAL_API_H_
#define MMC_GENERAL_API_H_

//moving to g++ @YL 4-10-2010
#ifdef __cplusplus
 extern "C" {
#endif
// General Recording definitions
#define NC_REC_TRIGGER_TYPE_MASK        0x0000ffff
#define NC_REC_TRIGGER_PARAM_MASK       0xffff0000

#define NC_REC_TRIGGERG_STATE_MASK      0x00ff
#define NC_REC_BUF_STATE_MASK           0xff00

#define NC_SCOPE_BITS_NONE_BUF_READY    0x000
#define NC_SCOPE_BITS_BUFFER1_READY     0x100
#define NC_SCOPE_BITS_BUFFER2_READY     0x200

#define NC_NORMAL_TRIGGER               0x10000
#define NC_AUTO_TRIGGER                 0x20000
#define NC_SINGLE_TRIGGER               0x30000



// Recording Parameters - "RP[i]"
#define TG_RECORDING_SPARE                  0
#define TG_RECORDING_TRIGGER_VALUE          1
#define TG_RECORDING_PRE_TRIGGER_LENGTH     2
#define TG_RECORDING_TRIGGER_TYPE           3
#define TG_RECORDING_TRIGGER_LEVEL_1        4
#define TG_RECORDING_TRIGGER_LEVEL_2        5
#define TG_RECORDING_TRIGGER_POLARITY       6
#define TG_RECORDING_TRIGGER_IN_MASK        7
//
// Trigger Modes
//
#define TG_RECORDING_TRIGGER_TYPE_NO_TRIGGER            0   ///< Edge    : Rising           (Positive Slope Over            : TRIGVAL >= Level#1)
#define TG_RECORDING_TRIGGER_TYPE_EDGE_Rise             1   ///< Edge    : Rising           (Positive Slope Over            : TRIGVAL >= Level#1)
#define TG_RECORDING_TRIGGER_TYPE_EDGE_Fall             2   ///< Edge    : Falling          (Negative Slope over            : TRIGVAL <= Level#1)
#define TG_RECORDING_TRIGGER_TYPE_EDGE_WindowIn         3   ///< Edge  : Window In      (Into the Window defined by     : Level#2 <= TRIGVAL <= LEVEL#1)
#define TG_RECORDING_TRIGGER_TYPE_EDGE_WindowOut        4   ///< Edge  : Window Out     (Out Of the Window defined by   : Level#2 <= TRIGVAL <= LEVEL#1)
#define TG_RECORDING_TRIGGER_TYPE_LEVEL_GE              5   ///< Level : >=             (GreaterEqual Then              : TRIGVAL >= Level#1)
#define TG_RECORDING_TRIGGER_TYPE_LEVEL_SE              6   ///< Level : <=             (SmallerEqual Then              : TRIGVAL <= Level#1)
#define TG_RECORDING_TRIGGER_TYPE_LEVEL_WindowInside    7   ///< Level : Inside  Window (Inside of Window defined by    : Level#2 <= TRIGVAL <= LEVEL#1)
#define TG_RECORDING_TRIGGER_TYPE_LEVEL_WindowOutside   8   ///< Level : outside Window (Outside of Window defined by   : Level#2 <= TRIGVAL <= LEVEL#1)
#define TG_RECORDING_TRIGGER_TYPE_EDGE_Rise_Mask        9   // Rising-Edge  + MASK      (Positive Slope Over            : (TRIGVAL & MASK) == MASK )
#define TG_RECORDING_TRIGGER_TYPE_EDGE_Fall_Mask        10  // Falling-Edge + MASK      (Negative Slope Over            : (TRIGVAL & MASK) != MASK )
#define TG_RECORDING_TRIGGER_TYPE_LEVEL_GE_Mask         11  // Grater-Equal + Mask      (Equal TO                       : (TRIGVAL & MASK) == MASK )
#define TG_RECORDING_TRIGGER_TYPE_LEVEL_SE_Mask         12  // Smaller-Equal+ Mask      (Not Equal TO                   : (TRIGVAL & MASK) != MASK )
#define TG_RECORDING_TRIGGER_TYPE_BEGIN_MOTION_Mask     13  // Begin Motion

///////////////////////////////////////////////////////////////////////////////
/// TYPES
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
/// \enum NC_REC_PARAM_NAMES_ENUM
///////////////////////////////////////////////////////////////////////////////
typedef enum
{
    NC_REC_DESIRED_POS_LOW_PARAM        = 0,
    NC_REC_DESIRED_POS_HIGH_PARAM,
    NC_REC_DESIRED_VEL_LOW_PARAM,
    NC_REC_DESIRED_VEL_HIGH_PARAM,
    NC_REC_GROUP_VEL_LOW_PARAM,
    NC_REC_GROUP_VEL_HIGH_PARAM,
    NC_REC_GROUP_ACDC_LOW_PARAM,
    NC_REC_GROUP_ACDC_HIGH_PARAM,
    NC_REC_GROUP_DC_LOW_PARAM,
    NC_REC_GROUP_DC_HIGH_PARAM,
    NC_REC_AXIS_AC_DC_LOW_PARAM,
    NC_REC_AXIS_AC_DC_HIGH_PARAM,
    NC_REC_JERK_LOW_PARAM,
    NC_REC_JERK_HIGH_PARAM,
    NC_REC_SMOOTH_FACTOR_AC_LOW_PARAM,
    NC_REC_SMOOTH_FACTOR_AC_HIGH_PARAM,
    NC_REC_SMOOTH_FACTOR_DC_LOW_PARAM,
    NC_REC_SMOOTH_FACTOR_DC_HIGH_PARAM,
    NC_REC_POS_INCR_LOW_PARAM,
    NC_REC_POS_INCR_HIGH_PARAM,
    NC_REC_CYCLE_CNT_PARAM,
    NC_REC_TARGET_POS_PARAM,
    NC_REC_TARGET_VEL_PARAM,
    NC_REC_F_POS_PARAM,
    NC_REC_F_VEL_PARAM,
    NC_DBG1_PARAM,
    NC_DBG2_PARAM,
    NC_DBG3_PARAM,
    NC_DBG4_PARAM,
    NC_DBG5_PARAM,
    NC_REC_ACTUAL_POS_PARAM,
    NC_REC_ACTUAL_VEL_PARAM,
    NC_REC_AXIS_STATUS_PARAM,
    NC_REC_ACTUAL_TORQUE_PARAM,
    NC_REC_I_USER_1_PARAM,
    NC_REC_I_USER_AUX_1_PARAM,
    NC_REC_F_USER_1_PARAM,
    NC_REC_F_USER_AUX_1_PARAM,
    NC_REC_I_USER_2_PARAM,
    NC_REC_I_USER_AUX_2_PARAM,
    NC_REC_F_USER_2_PARAM,
    NC_REC_F_USER_AUX_2_PARAM,
    NC_REC_I_USER_3_PARAM,
    NC_REC_I_USER_AUX_3_PARAM,
    NC_REC_F_USER_3_PARAM,
    NC_REC_F_USER_AUX_3_PARAM,
    NC_REC_I_USER_4_PARAM,
    NC_REC_I_USER_AUX_4_PARAM,
    NC_REC_F_USER_4_PARAM,
    NC_REC_F_USER_AUX_4_PARAM,
    NC_REC_POS_FOLLOWING_ERR_PARAM,
    NC_REC_DIGITAL_INPUTS_PARAM,
    NC_REC_DIGITAL_OUTPUTS_PARAM,
    NC_REC_TRACKING_ERROR_LOW_PARAM,
    NC_REC_TRACKING_ERROR_HIGH_PARAM,
    NC_REC_ERROR_CORRECTION_POS_PARAM,
    NC_REC_ACTUAL_HW_POSITION_PARAM,
    NC_REC_CONTROL_WORD_PARAM,
    NC_REC_STATUS_WORD_PARAM,
    NC_REC_MOTION_MODE_PARAM,
    NC_REC_DI_LOW_PARAM,
    NC_REC_DI_HIGH_PARAM,
    NC_REC_DO_LOW_PARAM,
    NC_REC_DO_HIGH_PARAM,
    NC_REC_AXIS_COMM_ERROR_PARAM,
    NC_REC_AXIS_LAST_EMCY_CODE_PARAM,
    NC_STATUS_REGISTER,
    NC_MCS_LIMIT_REGISTER,
    NC_REC_DESIRED_PCS_X_POS_LOW_PARAM,
    NC_REC_DESIRED_PCS_X_POS_HIGH_PARAM,
    NC_REC_DESIRED_PCS_Y_POS_LOW_PARAM,
    NC_REC_DESIRED_PCS_Y_POS_HIGH_PARAM,
    NC_REC_DESIRED_PCS_Z_POS_LOW_PARAM,
    NC_REC_DESIRED_PCS_Z_POS_HIGH_PARAM,
    NC_REC_DESIRED_PCS_U_POS_LOW_PARAM,
    NC_REC_DESIRED_PCS_U_POS_HIGH_PARAM,
    NC_REC_DESIRED_PCS_V_POS_LOW_PARAM,
    NC_REC_DESIRED_PCS_V_POS_HIGH_PARAM,
    NC_REC_DESIRED_PCS_W_POS_LOW_PARAM,
    NC_REC_DESIRED_PCS_W_POS_HIGH_PARAM,
    NC_REC_DESIRED_MCS_N1_POS_LOW_PARAM,
    NC_REC_DESIRED_MCS_N1_POS_HIGH_PARAM,
    NC_REC_DESIRED_MCS_N2_POS_LOW_PARAM,
    NC_REC_DESIRED_MCS_N2_POS_HIGH_PARAM,
    NC_REC_DESIRED_MCS_N3_POS_LOW_PARAM,
    NC_REC_DESIRED_MCS_N3_POS_HIGH_PARAM,
    NC_REC_DESIRED_MCS_N4_POS_LOW_PARAM,
    NC_REC_DESIRED_MCS_N4_POS_HIGH_PARAM,
    NC_REC_DESIRED_MCS_N5_POS_LOW_PARAM,
    NC_REC_DESIRED_MCS_N5_POS_HIGH_PARAM,
    NC_REC_DESIRED_MCS_N6_POS_LOW_PARAM,
    NC_REC_DESIRED_MCS_N6_POS_HIGH_PARAM,
    NC_REC_DESIRED_MCS_N7_POS_LOW_PARAM,
    NC_REC_DESIRED_MCS_N7_POS_HIGH_PARAM,
    NC_REC_DESIRED_MCS_N8_POS_LOW_PARAM,
    NC_REC_DESIRED_MCS_N8_POS_HIGH_PARAM,
    NC_REC_DESIRED_MCS_N9_POS_LOW_PARAM,
    NC_REC_DESIRED_MCS_N9_POS_HIGH_PARAM,
    NC_REC_DESIRED_MCS_S_POS_LOW_PARAM,
    NC_REC_DESIRED_MCS_S_POS_HIGH_PARAM,
    NC_REC_DESIRED_PCS_X_VEL_LOW_PARAM,
    NC_REC_DESIRED_PCS_X_VEL_HIGH_PARAM,
    NC_REC_DESIRED_PCS_Y_VEL_LOW_PARAM,
    NC_REC_DESIRED_PCS_Y_VEL_HIGH_PARAM,
    NC_REC_DESIRED_PCS_Z_VEL_LOW_PARAM,
    NC_REC_DESIRED_PCS_Z_VEL_HIGH_PARAM,
    NC_REC_DESIRED_PCS_U_VEL_LOW_PARAM,
    NC_REC_DESIRED_PCS_U_VEL_HIGH_PARAM,
    NC_REC_DESIRED_PCS_V_VEL_LOW_PARAM,
    NC_REC_DESIRED_PCS_V_VEL_HIGH_PARAM,
    NC_REC_DESIRED_PCS_W_VEL_LOW_PARAM,
    NC_REC_DESIRED_PCS_W_VEL_HIGH_PARAM,
    NC_REC_DESIRED_MCS_N1_VEL_LOW_PARAM,
    NC_REC_DESIRED_MCS_N1_VEL_HIGH_PARAM,
    NC_REC_DESIRED_MCS_N2_VEL_LOW_PARAM,
    NC_REC_DESIRED_MCS_N2_VEL_HIGH_PARAM,
    NC_REC_DESIRED_MCS_N3_VEL_LOW_PARAM,
    NC_REC_DESIRED_MCS_N3_VEL_HIGH_PARAM,
    NC_REC_DESIRED_MCS_N4_VEL_LOW_PARAM,
    NC_REC_DESIRED_MCS_N4_VEL_HIGH_PARAM,
    NC_REC_DESIRED_MCS_N5_VEL_LOW_PARAM,
    NC_REC_DESIRED_MCS_N5_VEL_HIGH_PARAM,
    NC_REC_DESIRED_MCS_N6_VEL_LOW_PARAM,
    NC_REC_DESIRED_MCS_N6_VEL_HIGH_PARAM,
    NC_REC_DESIRED_MCS_N7_VEL_LOW_PARAM,
    NC_REC_DESIRED_MCS_N7_VEL_HIGH_PARAM,
    NC_REC_DESIRED_MCS_N8_VEL_LOW_PARAM,
    NC_REC_DESIRED_MCS_N8_VEL_HIGH_PARAM,
    NC_REC_DESIRED_MCS_N9_VEL_LOW_PARAM,
    NC_REC_DESIRED_MCS_N9_VEL_HIGH_PARAM,
    NC_REC_DESIRED_MCS_S_VEL_LOW_PARAM,
    NC_REC_DESIRED_MCS_S_VEL_HIGH_PARAM,
    NC_REC_DESIRED_PCS_X_AC_DC_LOW_PARAM,
    NC_REC_DESIRED_PCS_X_AC_DC_HIGH_PARAM,
    NC_REC_DESIRED_PCS_Y_AC_DC_LOW_PARAM,
    NC_REC_DESIRED_PCS_Y_AC_DC_HIGH_PARAM,
    NC_REC_DESIRED_PCS_Z_AC_DC_LOW_PARAM,
    NC_REC_DESIRED_PCS_Z_AC_DC_HIGH_PARAM,
    NC_REC_DESIRED_PCS_U_AC_DC_LOW_PARAM,
    NC_REC_DESIRED_PCS_U_AC_DC_HIGH_PARAM,
    NC_REC_DESIRED_PCS_V_AC_DC_LOW_PARAM,
    NC_REC_DESIRED_PCS_V_AC_DC_HIGH_PARAM,
    NC_REC_DESIRED_PCS_W_AC_DC_LOW_PARAM,
    NC_REC_DESIRED_PCS_W_AC_DC_HIGH_PARAM,
    NC_REC_DESIRED_MCS_N1_AC_DC_LOW_PARAM,
    NC_REC_DESIRED_MCS_N1_AC_DC_HIGH_PARAM,
    NC_REC_DESIRED_MCS_N2_AC_DC_LOW_PARAM,
    NC_REC_DESIRED_MCS_N2_AC_DC_HIGH_PARAM,
    NC_REC_DESIRED_MCS_N3_AC_DC_LOW_PARAM,
    NC_REC_DESIRED_MCS_N3_AC_DC_HIGH_PARAM,
    NC_REC_DESIRED_MCS_N4_AC_DC_LOW_PARAM,
    NC_REC_DESIRED_MCS_N4_AC_DC_HIGH_PARAM,
    NC_REC_DESIRED_MCS_N5_AC_DC_LOW_PARAM,
    NC_REC_DESIRED_MCS_N5_AC_DC_HIGH_PARAM,
    NC_REC_DESIRED_MCS_N6_AC_DC_LOW_PARAM,
    NC_REC_DESIRED_MCS_N6_AC_DC_HIGH_PARAM,
    NC_REC_DESIRED_MCS_N7_AC_DC_LOW_PARAM,
    NC_REC_DESIRED_MCS_N7_AC_DC_HIGH_PARAM,
    NC_REC_DESIRED_MCS_N8_AC_DC_LOW_PARAM,
    NC_REC_DESIRED_MCS_N8_AC_DC_HIGH_PARAM,
    NC_REC_DESIRED_MCS_N9_AC_DC_LOW_PARAM,
    NC_REC_DESIRED_MCS_N9_AC_DC_HIGH_PARAM,
    NC_REC_DESIRED_MCS_S_AC_DC_LOW_PARAM,
    NC_REC_DESIRED_MCS_S_AC_DC_HIGH_PARAM,
    NC_REC_END_MOTION_REASON_PARAM,
    NC_REC_ANALOG_INPUT_PARAM,
    NC_REC_DESIRED_MCS_X_POS_LOW_PARAM,
    NC_REC_DESIRED_MCS_X_POS_HIGH_PARAM,
    NC_REC_DESIRED_MCS_Y_POS_LOW_PARAM,
    NC_REC_DESIRED_MCS_Y_POS_HIGH_PARAM,
    NC_REC_DESIRED_MCS_Z_POS_LOW_PARAM,
    NC_REC_DESIRED_MCS_Z_POS_HIGH_PARAM,
    NC_REC_DESIRED_MCS_U_POS_LOW_PARAM,
    NC_REC_DESIRED_MCS_U_POS_HIGH_PARAM,
    NC_REC_DESIRED_MCS_V_POS_LOW_PARAM,
    NC_REC_DESIRED_MCS_V_POS_HIGH_PARAM,
    NC_REC_DESIRED_MCS_W_POS_LOW_PARAM,
    NC_REC_DESIRED_MCS_W_POS_HIGH_PARAM,
    NC_REC_DESIRED_ACS_A1_POS_LOW_PARAM,
    NC_REC_DESIRED_ACS_A1_POS_HIGH_PARAM,
    NC_REC_DESIRED_ACS_A2_POS_LOW_PARAM,
    NC_REC_DESIRED_ACS_A2_POS_HIGH_PARAM,
    NC_REC_DESIRED_ACS_A3_POS_LOW_PARAM,
    NC_REC_DESIRED_ACS_A3_POS_HIGH_PARAM,
    NC_REC_DESIRED_ACS_A4_POS_LOW_PARAM,
    NC_REC_DESIRED_ACS_A4_POS_HIGH_PARAM,
    NC_REC_DESIRED_ACS_A5_POS_LOW_PARAM,
    NC_REC_DESIRED_ACS_A5_POS_HIGH_PARAM,
    NC_REC_DESIRED_ACS_A6_POS_LOW_PARAM,
    NC_REC_DESIRED_ACS_A6_POS_HIGH_PARAM,
    NC_REC_DESIRED_MCS_X_VEL_LOW_PARAM,
    NC_REC_DESIRED_MCS_X_VEL_HIGH_PARAM,
    NC_REC_DESIRED_MCS_Y_VEL_LOW_PARAM,
    NC_REC_DESIRED_MCS_Y_VEL_HIGH_PARAM,
    NC_REC_DESIRED_MCS_Z_VEL_LOW_PARAM,
    NC_REC_DESIRED_MCS_Z_VEL_HIGH_PARAM,
    NC_REC_DESIRED_MCS_U_VEL_LOW_PARAM,
    NC_REC_DESIRED_MCS_U_VEL_HIGH_PARAM,
    NC_REC_DESIRED_MCS_V_VEL_LOW_PARAM,
    NC_REC_DESIRED_MCS_V_VEL_HIGH_PARAM,
    NC_REC_DESIRED_MCS_W_VEL_LOW_PARAM,
    NC_REC_DESIRED_MCS_W_VEL_HIGH_PARAM,
    NC_REC_DESIRED_ACS_A1_VEL_LOW_PARAM,
    NC_REC_DESIRED_ACS_A1_VEL_HIGH_PARAM,
    NC_REC_DESIRED_ACS_A2_VEL_LOW_PARAM,
    NC_REC_DESIRED_ACS_A2_VEL_HIGH_PARAM,
    NC_REC_DESIRED_ACS_A3_VEL_LOW_PARAM,
    NC_REC_DESIRED_ACS_A3_VEL_HIGH_PARAM,
    NC_REC_DESIRED_ACS_A4_VEL_LOW_PARAM,
    NC_REC_DESIRED_ACS_A4_VEL_HIGH_PARAM,
    NC_REC_DESIRED_ACS_A5_VEL_LOW_PARAM,
    NC_REC_DESIRED_ACS_A5_VEL_HIGH_PARAM,
    NC_REC_DESIRED_ACS_A6_VEL_LOW_PARAM,
    NC_REC_DESIRED_ACS_A6_VEL_HIGH_PARAM,
    NC_REC_DESIRED_MCS_X_AC_DC_LOW_PARAM,
    NC_REC_DESIRED_MCS_X_AC_DC_HIGH_PARAM,
    NC_REC_DESIRED_MCS_Y_AC_DC_LOW_PARAM,
    NC_REC_DESIRED_MCS_Y_AC_DC_HIGH_PARAM,
    NC_REC_DESIRED_MCS_Z_AC_DC_LOW_PARAM,
    NC_REC_DESIRED_MCS_Z_AC_DC_HIGH_PARAM,
    NC_REC_DESIRED_MCS_U_AC_DC_LOW_PARAM,
    NC_REC_DESIRED_MCS_U_AC_DC_HIGH_PARAM,
    NC_REC_DESIRED_MCS_V_AC_DC_LOW_PARAM,
    NC_REC_DESIRED_MCS_V_AC_DC_HIGH_PARAM,
    NC_REC_DESIRED_MCS_W_AC_DC_LOW_PARAM,
    NC_REC_DESIRED_MCS_W_AC_DC_HIGH_PARAM,
    NC_REC_DESIRED_ACS_A1_AC_DC_LOW_PARAM,
    NC_REC_DESIRED_ACS_A1_AC_DC_HIGH_PARAM,
    NC_REC_DESIRED_ACS_A2_AC_DC_LOW_PARAM,
    NC_REC_DESIRED_ACS_A2_AC_DC_HIGH_PARAM,
    NC_REC_DESIRED_ACS_A3_AC_DC_LOW_PARAM,
    NC_REC_DESIRED_ACS_A3_AC_DC_HIGH_PARAM,
    NC_REC_DESIRED_ACS_A4_AC_DC_LOW_PARAM,
    NC_REC_DESIRED_ACS_A4_AC_DC_HIGH_PARAM,
    NC_REC_DESIRED_ACS_A5_AC_DC_LOW_PARAM,
    NC_REC_DESIRED_ACS_A5_AC_DC_HIGH_PARAM,
    NC_REC_DESIRED_ACS_A6_AC_DC_LOW_PARAM,
    NC_REC_DESIRED_ACS_A6_AC_DC_HIGH_PARAM,
    NC_REC_SPEED_OVERRIDE_PARAM,
    NC_REC_TARGET_TORQUE_RC_PARAM,
    NC_REC_AUXILARY_POS_PARAM,
    NC_REC_TARGET_TORQUE_UU_LOW_PARAM,
    NC_REC_TARGET_TORQUE_UU_HIGH_PARAM,
    NC_REC_TORQUE_VELOCITY_UU_LOW_PARAM,
    NC_REC_TORQUE_VELOCITY_UU_HIGH_PARAM,
    NC_REC_TORQUE_ACCELERATION_UU_LOW_PARAM,
    NC_REC_TORQUE_ACCELERATION_UU_HIGH_PARAM,
    NC_REC_ACTUAL_POS_UU_LOW_PARAM,
    NC_REC_ACTUAL_POS_UU_HIGH_PARAM,
    NC_REC_D_ACTUAL_POS_CNT_LOW_PARAM,
    NC_REC_D_ACTUAL_POS_CNT_HIGH_PARAM,
    NC_REC_TARGET_POS_UU_LOW_PARAM,
    NC_REC_TARGET_POS_UU_HIGH_PARAM,
    NC_REC_D_TARGET_POS_CNT_LOW_PARAM,
    NC_REC_D_TARGET_POS_CNT_HIGH_PARAM,
    NC_REC_D_ACTUAL_HW_POS_CNT_LOW_PARAM,
    NC_REC_D_ACTUAL_HW_POS_CNT_HIGH_PARAM,
    NC_REC_ACTUAL_VEL_UU_LOW_PARAM,
    NC_REC_ACTUAL_VEL_UU_HIGH_PARAM,
    NC_REC_TARGET_VEL_UU_LOW_PARAM,
    NC_REC_TARGET_VEL_UU_HIGH_PARAM,
    NC_REC_D_TARGET_VEL_CNT_LOW_PARAM,
    NC_REC_D_TARGET_VEL_CNT_HIGH_PARAM,
    NC_REC_D_ACDC_CNT_LOW_PARAM,
    NC_REC_D_ACDC_CNT_HIGH_PARAM,
    NC_REC_TARGET_MOD_POS_UU_LOW_PARAM,
    NC_REC_TARGET_MOD_POS_UU_HIGH_PARAM,
    NC_REC_TARGET_POS_TOTAL_UU_LOW_PARAM,
    NC_REC_TARGET_POS_TOTAL_UU_HIGH_PARAM,
    NC_DBG6_LOW_PARAM,
    NC_DBG6_HIGH_PARAM,
    NC_USR_607A_PARAM,//271
    NC_USR_60FF_PARAM,
    NC_USR_60B1_PARAM,
    NC_USR_6071_PARAM,
    NC_USR_60B2_PARAM,//275
    NC_MB1_PARAM,
    NC_MB2_PARAM,
    NC_USR_CW_PARAM,
    NC_REC_CPLD_ANALOG_CH_A_INPUT_1_LOW,
    NC_REC_CPLD_ANALOG_CH_A_INPUT_1_HIGH,//280
    NC_REC_CPLD_ANALOG_CH_A_INPUT_2_LOW,
    NC_REC_CPLD_ANALOG_CH_A_INPUT_2_HIGH,
    NC_REC_CPLD_ANALOG_CH_A_INPUT_3_LOW,
    NC_REC_CPLD_ANALOG_CH_A_INPUT_3_HIGH,
    NC_REC_CPLD_ANALOG_CH_A_INPUT_4_LOW,//285
    NC_REC_CPLD_ANALOG_CH_A_INPUT_4_HIGH,
    NC_REC_CPLD_ANALOG_CH_B_INPUT_1_LOW,
    NC_REC_CPLD_ANALOG_CH_B_INPUT_1_HIGH,
    NC_REC_CPLD_ANALOG_CH_B_INPUT_2_LOW,
    NC_REC_CPLD_ANALOG_CH_B_INPUT_2_HIGH,// 290
    NC_REC_CPLD_ANALOG_CH_B_INPUT_3_LOW,
    NC_REC_CPLD_ANALOG_CH_B_INPUT_3_HIGH,
    NC_REC_CPLD_ANALOG_CH_B_INPUT_4_LOW,
    NC_REC_CPLD_ANALOG_CH_B_INPUT_4_HIGH,
    NC_REC_CPLD_ANALOG_CH_A_OUTPUT_1,// 295
    NC_REC_CPLD_ANALOG_CH_A_OUTPUT_2,
    NC_REC_CPLD_ANALOG_CH_A_OUTPUT_3,
    NC_REC_CPLD_ANALOG_CH_A_OUTPUT_4,
    NC_REC_CPLD_ANALOG_CH_B_OUTPUT_1,
    NC_REC_CPLD_ANALOG_CH_B_OUTPUT_2,// 300
    NC_REC_CPLD_ANALOG_CH_B_OUTPUT_3,
    NC_REC_CPLD_ANALOG_CH_B_OUTPUT_4,
    NC_REC_CPLD_EXTENDEDIO1_INPUT,
    NC_REC_CPLD_EXTENDEDIO1_OUTPUT,
    NC_REC_CPLD_EXTENDEDIO2_OUTPUT,// 305
    NC_REC_CPLD_EXTENDEDIO3_OUTPUT,
    NC_REC_CPLD_ENCODER_QUAD_1_POS_LONGLONG_LOW,
    NC_REC_CPLD_ENCODER_QUAD_1_POS_LONGLONG_HIGH,
    NC_REC_CPLD_ENCODER_QUAD_1_POS_DOUBLE_LOW,
    NC_REC_CPLD_ENCODER_QUAD_1_POS_DOUBLE_HIGH, //310
    NC_REC_CPLD_ENCODER_QUAD_1_VEL_LOW,
    NC_REC_CPLD_ENCODER_QUAD_1_VEL_HIGH,
    NC_REC_CPLD_ENCODER_QUAD_2_POS_LONGLONG_LOW,
    NC_REC_CPLD_ENCODER_QUAD_2_POS_LONGLONG_HIGH,
    NC_REC_CPLD_ENCODER_QUAD_2_POS_DOUBLE_LOW,// 315
    NC_REC_CPLD_ENCODER_QUAD_2_POS_DOUBLE_HIGH,
    NC_REC_CPLD_ENCODER_QUAD_2_VEL_LOW,
    NC_REC_CPLD_ENCODER_QUAD_2_VEL_HIGH,
    NC_REC_CPLD_ENCODER_ABS_1_POS_LONGLONG_LOW,
    NC_REC_CPLD_ENCODER_ABS_1_POS_LONGLONG_HIGH,//320
    NC_REC_CPLD_ENCODER_ABS_1_POS_DOUBLE_LOW,
    NC_REC_CPLD_ENCODER_ABS_1_POS_DOUBLE_HIGH,
    NC_REC_CPLD_ENCODER_ABS_1_VEL_LOW,
    NC_REC_CPLD_ENCODER_ABS_1_VEL_HIGH,
    NC_REC_CPLD_ENCODER_ABS_2_POS_LONGLONG_LOW,//325
    NC_REC_CPLD_ENCODER_ABS_2_POS_LONGLONG_HIGH,
    NC_REC_CPLD_ENCODER_ABS_2_POS_DOUBLE_LOW,
    NC_REC_CPLD_ENCODER_ABS_2_POS_DOUBLE_HIGH,
    NC_REC_CPLD_ENCODER_ABS_2_VEL_LOW,
    NC_REC_CPLD_ENCODER_ABS_2_VEL_HIGH, //330
    NC_REC_MAX_NUM_PARAM,
}NC_REC_PARAM_NAMES_ENUM;



typedef enum {
    MMC_UNKNOWN_PARAM               = 0,
    MMC_AXIS_MODE_PARAM             = 1,
    MMC_AXIS_OP_MOTION_MODE_PARAM   = 2,
    MMC_AXIS_STATE_PARAM            = 3,
    MMC_AXIS_GROUP_ID_PARAM         = 4,
    MMC_DRIVE_ID_PARAM              = 5,
    MMC_MOVEMENT_TYPE_PARAM         = 6,
    MMC_POSITION_PERIOD_PARAM       = 7,
    MMC_CONTROLLER_MODE             = 8,
    MMC_REAL_CONTROLLER_MODE_PARAM  = 9,
    MMC_SET_POSITION_PARAM          = 10,
    MMC_ACT_POSITION_PARAM          = 11,
    MMC_AIM_POSITION_PARAM          = 12,
    MMC_SET_VELOCITY_PARAM          = 13,
    MMC_ACT_VELOCITY_PARAM          = 14,
    MMC_MAX_VELOCITY_PARAM          = 15,
    MMC_SW_MAX_VELOCITY_PARAM       = 16,
    MMC_CONSTANT_VELOCITY_PARAM     = 17,
    MMC_SET_ACCELERATION_PARAM      = 18,
    MMC_MAX_ACCELERATION_PARAM      = 19,
    MMC_SW_MAX_ACCELERATION_PARAM   = 20,
    MMC_ACCELERATING_PARAM          = 21,
    MMC_SET_DECELERATION_PARAM      = 22,
    MMC_MAX_DECELERATION_PARAM      = 23,
    MMC_SW_MAX_DECELERATION_PARAM   = 24,
    MMC_DECELERATING_PARAM          = 25,
    MMC_MAX_JERK_PARAM              = 26,
    MMC_SW_MAX_JERK_PARAM           = 27,
    MMC_SPATIAL_OPTION_PARAM        = 28,
    MMC_CYCLE_TIME_PARAM            = 29,
    MMC_RES_ID_PARAM                = 30,
    MMC_SW_LIMIT_HIGH_POS_PARAM     = 31,
    MMC_SW_LIMIT_LOW_POS_PARAM      = 32,
    MMC_I_COMM_EV_USR_1_PARAM       = 33,   /// PDO transmission From Drive (TPDO)
    MMC_I_COMM_EV_USR_1_AUX_PARAM   = 34,   /// PDO transmission From Drive (TPDO)
    MMC_F_COMM_EV_USR_1_PARAM       = 35,   /// PDO transmission From Drive (TPDO)
    MMC_F_COMM_EV_USR_1_AUX_PARAM   = 36,   /// PDO transmission From Drive (TPDO)
    MMC_I_COMM_EV_USR_2_PARAM       = 37,   /// PDO transmission From Drive (TPDO)
    MMC_I_COMM_EV_USR_2_AUX_PARAM   = 38,   /// PDO transmission From Drive (TPDO)
    MMC_F_COMM_EV_USR_2_PARAM       = 39,   /// PDO transmission From Drive (TPDO)
    MMC_F_COMM_EV_USR_2_AUX_PARAM   = 40,   /// PDO transmission From Drive (TPDO)
    MMC_I_COMM_EV_USR_3_PARAM       = 41,   /// PDO transmission TO Drive (RPDO)
    MMC_I_COMM_EV_USR_3_AUX_PARAM   = 42,   /// PDO transmission TO Drive (RPDO)
    MMC_F_COMM_EV_USR_3_PARAM       = 43,   /// PDO transmission TO Drive (RPDO)
    MMC_F_COMM_EV_USR_3_AUX_PARAM   = 44,   /// PDO transmission TO Drive (RPDO)
    MMC_I_COMM_EV_USR_4_PARAM       = 45,   /// PDO transmission TO Drive (RPDO)
    MMC_I_COMM_EV_USR_4_AUX_PARAM   = 46,   /// PDO transmission TO Drive (RPDO)
    MMC_F_COMM_EV_USR_4_PARAM       = 47,   /// PDO transmission TO Drive (RPDO)
    MMC_F_COMM_EV_USR_4_AUX_PARAM   = 48,   /// PDO transmission TO Drive (RPDO)
    MMC_CONNECTION_TYPE_PARAM       = 49,   /// Connection type - CAN/EtherCAT
    MMC_ERROR_CORRECTION_PARAM      = 50,
    MMC_S_FACTOR                    = 51,
    MMC_MOTOR_ON_MAX_TIMEOUT_MSEC   = 52,
    MMC_SUPPORT_BLENDED_FB_PENDING  = 53,   /// Are we allowed to insert an ARC when transitioning from 1 to 0 (execute).
    MMC_FB_CNT_RECALC_FROM_ACTIVE   = 54,   /// Number of FB's to recalc from active. Def: 0.
    MMC_EST_TIME_TO_BE_ACTIVEFB_THRSHLD = 55,   /// 
    MMC_MAX_CURRENT_PARAM                   = 56,
    MMC_LIMIT_STOP_DECELERATION             = 57,
    MMC_LIMIT_STOP_JERK                     = 58,
    MMC_SET_VECTOR_VELOCITY_PARAM           = 59,
    MMC_MCS_SW_LIMIT_LOW_POS_ARRAY          = 60,
    MMC_MCS_SW_LIMIT_HIGH_POS_ARRAY         = 61,
    MMC_MCS_S_DIRECTION                     = 62,
    MMC_TARGET_RADIUS                       = 63,
    MMC_TARGET_TIME                         = 64,
    MMC_PROFILE_TIME                        = 65,
    MMC_OVERALL_MOTION_TIME                 = 66,
    MMC_MAX_TRACKING_ERROR_POSITION         = 67,
    MMC_MAX_TRACKING_ERROR_TIME             = 68,
    MMC_DRIVE_TRACKING_ERROR                = 69,
    MMC_GMAS_TRACKING_ERROR                 = 70,
    MMC_END_MOTION_REASON                   = 71,
    MMC_AXIS_ERROR_MASK                     = 72,
    MMC_LAST_DRIVE_EMERGENCY                = 73,
    MMC_AXIS_ASYNC_ERROR_CODE               = 74,
    MMC_FB_DEPTH                            = 75,
    MMC_ANALOG_INPUT                        = 76,
    MMC_EMCY_SET_ERR                        = 77,
    MMC_ETHERCAT_DRV_OUTPUT                 = 78,
    MMC_DIGITAL_INPUT_LOGIC                 = 79,
    MMC_IS_FATAL_ERROR                      = 80,
    MMC_LAST_SYSTEM_ERROR                   = 81,
    MMC_LAST_NODE_INIT_ERROR                = 82,
    MMC_LAST_SDO_ABORT_RAW_DATA             = 83,
    MMC_SPEED_OVERRIDE                      = 84,
    MMC_FAST_REFERENCE                      = 85,
    MMC_SET_ACDC_PARAM                      = 86,
    MMC_DIGITAL_INPUT_PARAM                 = 87,
    MMC_CAN_DRV_OUTPUT                      = 88,
    MMC_DS402_CONTROL_WORD                  = 89,
    MMC_DS402_STATUS_WORD                   = 90,
    MMC_STATUS_REGISTER                     = 91,
    MMC_MCS_LIMIT_REGISTER                  = 92,
    MMC_ErCr_AUTOLOAD                       = 93,
    MMC_MAX_DESIRED_TORQUE_PARAM            = 94,
    MMC_MAX_TORQUE_VELOCITY_PARAM           = 95,
    MMC_MAX_TORQUE_ACCELERATION_PARAM       = 96,
    MMC_USER_UNITS_POSITION                 = 97,
    MMC_USER_UNITS_VELOCITY                 = 98,
    MMC_POSITION_OFFSET                     = 99,
    MMC_TARGET_POS_UU                       = 100,
    MMC_ACTUAL_POS_UU                       = 101,
    MMC_MODULO_AXIS                         = 102,
    MMC_MODULO_LOW                          = 103,
    MMC_MODULO_HIGH                         = 104,
    MMC_MODULO_ACTUAL_CYCLE                 = 105,
    MMC_MODULO_TARGET_CYCLE                 = 106,
    MMC_AUXILIARY_POSITION                  = 107,
    MMC_UU_ENUM_POS                         = 108,
    MMC_UU_ENUM_VEL                         = 109,
    MMC_FAST_REFERENCE_MODE                 = 110,
    MMC_GMAS_INIT_STATE                     = 111,
    MMC_TOUCH_PROB_POS                      = 112,
    MMC_TOUCH_PROB_EVT_CNT                  = 113,
    MMC_PI_NUM_OF_INPUTS                    = 114,
    MMC_PI_NUM_OF_OUTPUTS                   = 115,
    MMC_ERROR_STOP_DECELERATION             = 116,
    MMC_ERROR_STOP_JERK                     = 117,
    MMC_STATE_TRANS_CYCLES                  = 118,
    MMC_AL_CYCLES                           = 119,
    MMC_POLICY_FAILED_STATE                 = 120,
    MMC_POLICY_FAIL_CODE                    = 121,
    MMC_PHY_ERR_POLICY                      = 122,
    MMC_PHY_ERR_THRESHOLD                   = 123,
    MMC_CYC_ERR_POLICY                      = 124,
    MMC_CYC_ERR_THRESHOLD                   = 125,
    MMC_MISS_FRAME_POLICY                   = 126,
    MMC_MISS_FRAME_THRESHOLD                = 127,
    MMC_AL_ERR_POLICY                       = 128,
    MMC_UNEX_MO_POLICY                      = 129,
    MMC_FAULT_POLICY                        = 130,
    MMC_QSTOP_POLICY                        = 131,
    MMC_HBT_POLICY                          = 132,
    MMC_EMCY_POLICY                         = 133,
    MMC_FB_POLICY                           = 134,
    MMC_CYCLE_TIME_COUNTER                  = 135,
    MMC_IUSER607A                           = 136,
    MMC_UCUSER607A_SRC                      = 137,
    MMC_IUSER60FF                           = 138,
    MMC_UCUSER60FF_SRC                      = 139,
    MMC_IUSER60B1                           = 140,
    MMC_UCUSER60B1_SRC                      = 141,
    MMC_IUSER6071                           = 142,
    MMC_UCUSER6071_SRC                      = 143,
    MMC_IUSERMB1                            = 144,
    MMC_IUSERMB2                            = 145,
    MMC_IUSER60B2                           = 146,
    MMC_UCUSER60B2_SRC                      = 147,
    MMC_IEC_RUN_ON_STARTUP                  = 148,
    MMC_TORQEMATORCRATIO                    = 149,
    MMC_MOTORRATEDCURRENT                   = 150,
    MMC_USERCW                              = 151,
    MMC_USERCW_SRC                          = 152,
    MMC_BURN_CPLD_DURING_DF                 = 153,
    MMC_CPLD_VERSION                        = 154,
    MMC_ANALOG_INPUT_VAL_CH_A               = 155,
    MMC_ANALOG_INPUT_VAL_CH_B               = 156,
    MMC_ANALOG_INPUT_OFFSET                 = 157,
    MMC_ANALOG_INPUT_GAIN                   = 158,
    MMC_ANALOG_INPUT_DEAD_BAND              = 159,
    MMC_ANALOG_INPUT_FILTER_FREQ            = 160,
    MMC_ANALOG_INPUT_FILTER_DAMP            = 161,
    MMC_ANALOG_OUTPUT_VAL_CH_A              = 162,
    MMC_ANALOG_OUTPUT_VAL_CH_B              = 163,
    MMC_CPLD_EXTENDEDIO1_INPUT_VAL          = 164,
    MMC_CPLD_EXTENDEDIO1_OUTPUT_CONFIG      = 165,
    MMC_CPLD_EXTENDEDIO1_OUTPUT_VAL         = 166,
    MMC_CPLD_EXTENDEDIO2_OUTPUT_VAL         = 167,
    MMC_CPLD_EXTENDEDIO3_OUTPUT_VAL         = 168,
    MMC_CPLD_ENCODER_QUAD_1_ACTIVE          = 169,
    MMC_CPLD_ENCODER_QUAD_1_POS_LONGLONG    = 170,
    MMC_CPLD_ENCODER_QUAD_1_POS_DOUBLE      = 171,
    MMC_CPLD_ENCODER_QUAD_1_VELOCITY        = 172,
    MMC_CPLD_ENCODER_QUAD_1_POS_UU          = 173,
    MMC_CPLD_ENCODER_QUAD_1_POS_UU_TYPE     = 174,
    MMC_CPLD_ENCODER_QUAD_1_VEL_UU          = 175,
    MMC_CPLD_ENCODER_QUAD_1_VEL_UU_TYPE     = 176,
    MMC_CPLD_ENCODER_QUAD_1_GILTCH_FILTER   = 177,
    MMC_CPLD_ENCODER_QUAD_1_VEL_MODE        = 178,
    MMC_CPLD_ENCODER_QUAD_1_VEL_THRESHOLD   = 179,
    MMC_CPLD_ENCODER_QUAD_1_QUAD_MODE       = 180,
    MMC_CPLD_ENCODER_QUAD_2_ACTIVE          = 181,
    MMC_CPLD_ENCODER_QUAD_2_POS_LONGLONG    = 182,
    MMC_CPLD_ENCODER_QUAD_2_POS_DOUBLE      = 183,
    MMC_CPLD_ENCODER_QUAD_2_VELOCITY        = 184,
    MMC_CPLD_ENCODER_QUAD_2_POS_UU          = 185,
    MMC_CPLD_ENCODER_QUAD_2_POS_UU_TYPE     = 186,
    MMC_CPLD_ENCODER_QUAD_2_VEL_UU          = 187,
    MMC_CPLD_ENCODER_QUAD_2_VEL_UU_TYPE     = 188,
    MMC_CPLD_ENCODER_QUAD_2_GILTCH_FILTER   = 189,
    MMC_CPLD_ENCODER_QUAD_2_VEL_MODE        = 190,
    MMC_CPLD_ENCODER_QUAD_2_VEL_THRESHOLD   = 191,
    MMC_CPLD_ENCODER_QUAD_2_QUAD_MODE       = 192,
    MMC_CPLD_ENCODER_ABS_1_ACTIVE           = 193,
    MMC_CPLD_ENCODER_ABS_1_POS_LONGLONG     = 194,
    MMC_CPLD_ENCODER_ABS_1_POS_DOUBLE       = 195,
    MMC_CPLD_ENCODER_ABS_1_VELOCITY         = 196,
    MMC_CPLD_ENCODER_ABS_1_POS_UU           = 197,
    MMC_CPLD_ENCODER_ABS_1_POS_UU_TYPE      = 198,
    MMC_CPLD_ENCODER_ABS_1_VEL_UU           = 199,
    MMC_CPLD_ENCODER_ABS_1_VEL_UU_TYPE      = 200,
    MMC_CPLD_ENCODER_ABS_2_ACTIVE           = 201,
    MMC_CPLD_ENCODER_ABS_2_POS_LONGLONG     = 202,
    MMC_CPLD_ENCODER_ABS_2_POS_DOUBLE       = 203,
    MMC_CPLD_ENCODER_ABS_2_VELOCITY         = 204,
    MMC_CPLD_ENCODER_ABS_2_POS_UU           = 205,
    MMC_CPLD_ENCODER_ABS_2_POS_UU_TYPE      = 206,
    MMC_CPLD_ENCODER_ABS_2_VEL_UU           = 207,
    MMC_CPLD_ENCODER_ABS_2_VEL_UU_TYPE      = 208,
    MMC_CPLD_RS485_FUNCTIONALITY            = 209,
    MMC_CPLD_RS485_PORT_OUTPUT              = 210,
    MMC_CPLD_RS485_PORT_OUT_GPIO            = 211,
    MMC_CPLD_RS485_PORT_INPUT               = 212,
    MMC_CPLD_TIME_CAPTURE_IO                = 213,
    MMC_CPLD_TIME_CAPTURE_MODE              = 214,
    MMC_CPLD_TIME_CAPTURE_TIME_LATCH        = 215,
    MMC_CPLD_TIME_CAPTURE_EVENTS_COUNTER    = 216,
    MMC_KIN_TYPE                            = 217,
    MMC_KIN_PRM_ARR1                        = 218,
    MMC_KIN_PRM_ARR2                        = 219,
    MMC_KIN_PRM_ARR3                        = 220,
    MMC_KIN_PRM_ARR4                        = 221,
    MMC_KIN_PRM_ARR5                        = 222,
    MMC_CPLD_CONFIG                         = 223,
    MMC_UU_LINEAR_POS                       = 224,
    MMC_UU_LINEAR_VEL                       = 225,
    MMC_UU_ROTARY_POS                       = 226,
    MMC_LICENSE_ARR                         = 227,
    MMC_PROFILER_TYPE                       = 228,
    MMC_MAX_PARAMETERS_NUM
}MMC_PARAMETER_LIST_ENUM;


//@ML CPLD
///////////////////////////////////////////////////////////////////////////////
/// \enum MMC_CONFIG_CPLD_FUNCTIONALITY_ENUM
/// \brief  CPLD configuration type.
///////////////////////////////////////////////////////////////////////////////
typedef enum
{
    eNC_CONFIG_CPLD_FUNC_NONE   = 0,
    eNC_CONFIG_CPLD_TIME_CAPTURE= 0x01,
    eNC_CONFIG_CPLD_EXTENDED_IO = 0x02,
    eNC_CONFIG_CPLD_RS485       = 0x04,
    eNC_CONFIG_CPLD_MAX_FUNCTION= 0x08

}NC_CONFIG_CPLD_FUNCTIONALITY;


///////////////////////////////////////////////////////////////////////////////
/// \enum MMC_GET_ERROR_DESCRIPTION_ENUM
/// \brief Down load configuration file's type.
///////////////////////////////////////////////////////////////////////////////
typedef enum
{
    MMC_GET_ERROR_DESCRIPTION_GMAS = 1,
    MMC_GET_ERROR_DESCRIPTION_DRIVE,
    MMC_GET_ERROR_DESCRIPTION_DRIVE_ABORT
}MMC_GET_ERROR_DESCRIPTION_ENUM;


///////////////////////////////////////////////////////////////////////////////
/// \enum MC_CONDITIONFB_OPERATION_TYPE
/// \brief the type of the operation in waituntilconditionfb.
///////////////////////////////////////////////////////////////////////////////
typedef enum {
    MC_CONDITIONFB_OP_NONE = 0,
    MC_CONDITIONFB_OP_EQUAL,
    MC_CONDITIONFB_OP_LOWER,
    MC_CONDITIONFB_OP_HIGHER,
    MC_CONDITIONFB_OP_LOWER_EQUAL,
    MC_CONDITIONFB_OP_HIGHER_EQUAL,
    MC_CONDITIONFB_OP_MASK_AND,
    MC_CONDITIONFB_OP_LAST
}MC_CONDITIONFB_OPERATION_TYPE;


///////////////////////////////////////////////////////////////////////////////
/// \enum 
/// \brief GMAS initalization states.
///////////////////////////////////////////////////////////////////////////////

enum
{
    eGMAS_STATE_NOT_INITIALIZED = 0,
    eGMAS_STATE_INITIALIZED     = 1,
    eGMAS_STATE_INITIALIZING    = 3,
};

///////////////////////////////////////////////////////////////////////////////
/// \enum Direction
/// \brief two possible PI variable directions, output or input
///////////////////////////////////////////////////////////////////////////////
typedef enum pi_directions
{
    ePI_INPUT       = 0,
    ePI_OUTPUT      = 1,
    ePI_NONE        = 3,
} PI_DIRECTIONS;

///////////////////////////////////////////////////////////////////////////////
/// \enum command opertaion
/// \brief opertaion available to execute user commadn
///////////////////////////////////////////////////////////////////////////////
typedef enum command_operatioin
{
    eMMC_COMMAND_OPERATION_START =0,
    eMMC_COMMAND_OPERATION_STOP =1,
    eMMC_COMMAND_OPERATION_ISRUNNING =2,
    eMMC_COMMAND_OPERATION_ISEXIST =3,
    eMMC_COMMAND_OPERATION_REMOVE =4,
    eMMC_COMMAND_OPERATION_ISEXECUTABLERUNNING =5,
    eMMC_COMMAND_OPERATION_ENABLEPERMISSION =6,
}MC_COMMAND_OPERATION;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READPARAMETER_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    MMC_PARAMETER_LIST_ENUM eParameterNumber;
    ELMO_INT32 iParameterArrIndex;
    ELMO_UINT8 ucEnable;
}MMC_READPARAMETER_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READPARAMETER_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_DOUBLE  dbValue;
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
}MMC_READPARAMETER_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READBOOLPARAMETER_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    MMC_PARAMETER_LIST_ENUM eParameterNumber;
    ELMO_INT32 iParameterArrIndex;
    ELMO_UINT8 ucEnable;
}MMC_READBOOLPARAMETER_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READBOOLPARAMETER_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_LINT32 lValue;
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
}MMC_READBOOLPARAMETER_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEPARAMETER_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_DOUBLE dbValue;
    MMC_PARAMETER_LIST_ENUM eParameterNumber;
    ELMO_INT32 iParameterArrIndex;
    ELMO_UINT8 ucEnable;
}MMC_WRITEPARAMETER_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEPARAMETER_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT16 usStatus;
    ELMO_INT16 usErrorID;
}MMC_WRITEPARAMETER_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEBOOLPARAMETER_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_LINT32 lValue;
    MMC_PARAMETER_LIST_ENUM eParameterNumber;
    ELMO_INT32  iParameterArrIndex;
    ELMO_UINT8  ucEnable;
}MMC_WRITEBOOLPARAMETER_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEBOOLPARAMETER_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
}MMC_WRITEBOOLPARAMETER_OUT;



///////////////////////////////////////////////////////////////////////////////
/// \struct MC_EXECUTION_MODE
/// \brief execution mode of the "write group of parameters" function
///////////////////////////////////////////////////////////////////////////////
typedef enum
{
    eMMC_EXECUTION_MODE_IMMEDIATE = 0,
    eMMC_EXECUTION_MODE_QUEUED
}MC_EXECUTION_MODE;


///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READGROUPOFPARAMETERSMEMBER
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readgroupofparametersmember
{
    MMC_PARAMETER_LIST_ENUM eParameterNumber;
    ELMO_INT32  iParameterIndex;
    ELMO_UINT16 usAxisRef;
    ELMO_UINT16 usPadding;
}MMC_READGROUPOFPARAMETERSMEMBER;


///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEGROUPOFPARAMETERSMEMBER
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writegroupofparametersmember
{
    ELMO_DOUBLE  dbValue;
    MMC_PARAMETER_LIST_ENUM eParameterNumber;
    ELMO_INT32  iParameterIndex;
    ELMO_UINT16 usAxisRef;
    ELMO_UINT16 usPadding1;
    ELMO_UINT16 usPadding2;
    ELMO_UINT16 usPadding3;
}MMC_WRITEGROUPOFPARAMETERSMEMBER;


#define GROUP_OF_PARAMETERS_MAXIMUM_SIZE 5

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READGROUPOFPARAMETERS_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readgroupofparameters_in
{
    MMC_READGROUPOFPARAMETERSMEMBER sParameters[GROUP_OF_PARAMETERS_MAXIMUM_SIZE];
    ELMO_UINT8  ucNumberOfParameters;
} MMC_READGROUPOFPARAMETERS_IN;


///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READGROUPOFPARAMETERS_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readgroupofparameters_out
{
    ELMO_DOUBLE  dbValue[GROUP_OF_PARAMETERS_MAXIMUM_SIZE];
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
    ELMO_UINT8  ucProblematicEntry;
} MMC_READGROUPOFPARAMETERS_OUT;


///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEGROUPOFPARAMETERS_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writegroupofparameters_in
{
    MMC_WRITEGROUPOFPARAMETERSMEMBER sParameters[GROUP_OF_PARAMETERS_MAXIMUM_SIZE];
    MC_EXECUTION_MODE   eExecutionMode;
    ELMO_UINT8          ucNumberOfParameters;
    ELMO_UINT8          ucMode;
    ELMO_UINT8          ucExecute;
} MMC_WRITEGROUPOFPARAMETERS_IN;


///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEGROUPOFPARAMETERS_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writegroupofparameters_out
{
    ELMO_UINT32 uiHndl;
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
    ELMO_UINT8  ucProblematicEntry;
} MMC_WRITEGROUPOFPARAMETERS_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEGROUPOFPARAMETERSMEMBEREX
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writegroupofparametersmemberex
{
    ELMO_DOUBLE              dbValue;
    MMC_PARAMETER_LIST_ENUM eParameterNumber;
    ELMO_INT32              iParameterIndex;
    ELMO_UINT16             usAxisRef;
    ELMO_UINT16             usPIVarOffset;
    ELMO_UINT8              ucPiDirection;
    ELMO_UINT8              pPadding[3];
}MMC_WRITEGROUPOFPARAMETERSMEMBEREX;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEGROUPOFPARAMETERSEX_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writegroupofparametersex_in
{
    MMC_WRITEGROUPOFPARAMETERSMEMBEREX sParameters[GROUP_OF_PARAMETERS_MAXIMUM_SIZE];
    MC_EXECUTION_MODE   eExecutionMode;
    ELMO_UINT8          ucNumberOfParameters;
    ELMO_UINT8          ucMode;
    ELMO_UINT8          ucExecute;
} MMC_WRITEGROUPOFPARAMETERSEX_IN;


///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEGROUPOFPARAMETERSEX_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writegroupofparametersex_out
{
    ELMO_UINT32 uiHndl;
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
    ELMO_UINT8  ucProblematicEntry;
} MMC_WRITEGROUPOFPARAMETERSEX_OUT;


#define ETHERCAT_MEMORY_READ_MAX_SIZE      1400
#define ETHERCAT_MEMORY_WRITE_MAX_SIZE      8
///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEMEMORYRANGE_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writememoryrange_in
{
    ELMO_UINT16 usRegAddr;
    ELMO_UINT8  ucLength;
    ELMO_UINT8  pData[ETHERCAT_MEMORY_WRITE_MAX_SIZE];
} MMC_WRITEMEMORYRANGE_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEMEMORYRANGE_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writememoryrange_out
{
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
} MMC_WRITEMEMORYRANGE_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READMEMORYRANGE_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readmemoryrange_in
{
    ELMO_UINT16 usRegAddr;
    ELMO_UINT8  ucLength;
} MMC_READMEMORYRANGE_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READMEMORYRANGE_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readmemoryrange_out
{
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
    ELMO_UINT8  pData[ETHERCAT_MEMORY_READ_MAX_SIZE];
} MMC_READMEMORYRANGE_OUT;


#define PI_LARGE_VAR_SIZE       1400
#define PI_ALIASING_LENGTH      16
#define PI_SHORT_BITSIZE        16
#define PI_INT_BITSIZE          32
#define PI_LONG_LONG_BITSIZE    64
#define PI_REG_VAR_SIZE         4
#define MAX_PI_VARIABLES_NUM    50

///////////////////////////////////////////////////////////////////////////////
/// \enum PI_VAR_TYPES
/// \brief PI variables types
///////////////////////////////////////////////////////////////////////////////
enum
{
    ePI_BOOL                    = 0,
    ePI_SIGNED_CHAR             = 1,
    ePI_UNSIGNED_CHAR           = 2,
    ePI_SIGNED_SHORT            = 3,
    ePI_UNSIGNED_SHORT          = 4,
    ePI_SIGNED_INT              = 5,
    ePI_UNSIGNED_INT            = 6,
    ePI_SIGNED_LONG_LONG        = 7,
    ePI_UNSIGNED_LONG_LONG      = 8,
    ePI_FLOAT                   = 9,
    ePI_DOUBLE                  = 10,
    ePI_BITWISE                 = 11,
    ePI_8MULTIPLE               = 12,
    ePI_INVALID
};

///////////////////////////////////////////////////////////////////////////////
/// \union UN_PI_VAR
/// \brief this is the input\output union for PI variable <= 32 bit
///////////////////////////////////////////////////////////////////////////////
typedef union unPIVar
{
    ELMO_UINT8      pRawData[PI_REG_VAR_SIZE];
    ELMO_INT8       cData;
    ELMO_UINT8      ucData;
    ELMO_INT16      sData;
    ELMO_UINT16     usData;
    ELMO_INT32      iData;
    ELMO_UINT32     uiData;
    ELMO_FLOAT       fData;
} UN_PI_VAR;

///////////////////////////////////////////////////////////////////////////////
/// \union UN_LARGE_PI_VAR
/// \brief this is the input\output union for PI variable > 32 bit
///////////////////////////////////////////////////////////////////////////////
typedef union unLargePIVar
{
    ELMO_UINT8      pData[PI_LARGE_VAR_SIZE];
    ELMO_DOUBLE      dbVal;
    ELMO_INT64      llVal;
    ELMO_UINT64     ullVal;
} UN_LARGE_PI_VAR;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READPIVAR_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readpivar_in
{
    ELMO_UINT16     usIndex;
    ELMO_UINT8      ucDirection;
}
MMC_READPIVAR_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READPIVAR_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readpivar_out
{
    UN_PI_VAR       unVal;
    ELMO_UINT16     usStatus;
    ELMO_INT16      usErrorID;
} MMC_READPIVAR_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEPIVAR_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writepivar_in
{
    UN_PI_VAR       unVal;
    ELMO_UINT16     usIndex;
} MMC_WRITEPIVAR_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEPIVAR_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writepivar_out
{
    ELMO_UINT16     usStatus;
    ELMO_INT16      usErrorID;
}MMC_WRITEPIVAR_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READLARGEPIVAR_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readlargepivar_in
{
    ELMO_UINT16     usIndex;
    ELMO_UINT8      ucDirection;
} MMC_READLARGEPIVAR_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READLARGEPIVAR_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readlargepivar_out
{
    UN_LARGE_PI_VAR unVal;
    ELMO_UINT16     usStatus;
    ELMO_INT16      usErrorID;
} MMC_READLARGEPIVAR_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITELARGEPIVAR_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writelargepivar_in
{
    UN_LARGE_PI_VAR unVal;
    ELMO_UINT16     usIndex;
} MMC_WRITELARGEPIVAR_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITELARGEPIVAR_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writelargepivar_out
{
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
} MMC_WRITELARGEPIVAR_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READPIVARBOOL_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readpivarbool_in
{
    ELMO_UINT16 usIndex;
    ELMO_UINT8  ucDirection;
} MMC_READPIVARBOOL_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READPIVARBOOL_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readpivarbool_out
{
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
    ELMO_UINT8  ucBOOL;
} MMC_READPIVARBOOL_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READPIVARCHAR_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readpivarsignedchar_in
{
    ELMO_UINT16 usIndex;
    ELMO_UINT8  ucDirection;
} MMC_READPIVARCHAR_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READPIVARCHAR_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readpivarsignedchar_out
{
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
    ELMO_INT8   cData;
} MMC_READPIVARCHAR_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READPIVARUSCHAR_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readpivarunsignedchar_in
{
    ELMO_UINT16 usIndex;
    ELMO_UINT8  ucDirection;
} MMC_READPIVARUCHAR_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READPIVARUSCHAR_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readpivarunsignedchar_out
{
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
    ELMO_UINT8  ucData;
} MMC_READPIVARUCHAR_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READPIVARSHORT_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readpivarsignedshort_in
{
    ELMO_UINT16 usIndex;
    ELMO_UINT8  ucDirection;
} MMC_READPIVARSHORT_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READPIVARSHORT_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readpivarsignedshort_out
{
    ELMO_INT16  sData;
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
} MMC_READPIVARSHORT_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READPIVARUSSHORT_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readpivarunsignedshort_in
{
    ELMO_UINT16 usIndex;
    ELMO_UINT8  ucDirection;
} MMC_READPIVARUSHORT_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READPIVARUSSHORT_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readpivarunsignedshort_out
{
    ELMO_UINT16 usData;
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
} MMC_READPIVARUSHORT_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READPIVARINT_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readpivarsignedint_in
{
    ELMO_UINT16 usIndex;
    ELMO_UINT8  ucDirection;
} MMC_READPIVARINT_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READPIVARINT_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readpivarsignedint_out
{
    ELMO_INT32  iData;
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
} MMC_READPIVARINT_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READPIVARUINT_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readpivarunsignedint_in
{
    ELMO_UINT16 usIndex;
    ELMO_UINT8  ucDirection;
} MMC_READPIVARUINT_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READPIVARUINT_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readpivarunsignedint_out
{
    ELMO_UINT32 uiData;
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
} MMC_READPIVARUINT_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READPIVARFLOAT_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readpivarfloat_in
{
    ELMO_UINT16 usIndex;
    ELMO_UINT8  ucDirection;
} MMC_READPIVARFLOAT_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READPIVARFLOAT_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readpivarfloat_out
{
    ELMO_FLOAT   fData;
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
} MMC_READPIVARFLOAT_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READPIVARLONGLONG_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readpivarsignedlonglong_in
{
    ELMO_UINT16 usIndex;
    ELMO_UINT8  ucDirection;
} MMC_READPIVARLONGLONG_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READPIVARLONGLONG_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readpivarsignedlonglong_out
{
    ELMO_INT64  llData;
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
} MMC_READPIVARLONGLONG_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READPIVARUSLONGLONG_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readpivarunsignedlonglong_in
{
    ELMO_UINT16 usIndex;
    ELMO_UINT8  ucDirection;
} MMC_READPIVARULONGLONG_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READPIVARULONGLONG_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readpivarunsignedlonglong_out
{
    ELMO_UINT64 ullData;
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
} MMC_READPIVARULONGLONG_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READPIVARDOUBLE_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readpivardouble_in
{
    ELMO_UINT16 usIndex;
    ELMO_UINT8  ucDirection;
} MMC_READPIVARDOUBLE_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READPIVARDOUBLE_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readpivardouble_out
{
    ELMO_DOUBLE  dbData;
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
} MMC_READPIVARDOUBLE_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READPIVARRAW_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readpivarraw_in
{
    ELMO_UINT16 usIndex;
    ELMO_UINT8  ucDirection;
    ELMO_UINT8  ucByteLength;
} MMC_READPIVARRAW_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READPIVARRAW_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readpivarraw_out
{
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
    ELMO_UINT8  pRawData[PI_REG_VAR_SIZE];
} MMC_READPIVARRAW_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READLARGEPIVARRAW_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readlargepivarraw_in
{
    ELMO_UINT16 usIndex;
    ELMO_UINT16 usByteLength; 
    ELMO_UINT8  ucDirection;
} MMC_READLARGEPIVARRAW_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READLARGEPIVARRAW_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readlargepivarraw_out
{
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
    ELMO_UINT8  pRawData[PI_LARGE_VAR_SIZE];
} MMC_READLARGEPIVARRAW_OUT;


///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEPIVARBOOL_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writepivarbool_in
{
    ELMO_UINT16 usIndex;
    ELMO_UINT8  ucBOOL;
} MMC_WRITEPIVARBOOL_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEPIVARBOOL_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writepivarbool_out
{
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
}MMC_WRITEPIVARBOOL_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEPIVARCHAR_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writepivarsignedchar_in
{
    ELMO_UINT16 usIndex;
    ELMO_INT8   cData;  
} MMC_WRITEPIVARCHAR_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEPIVARCHAR_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writepivarsignedchar_out
{
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
}MMC_WRITEPIVARCHAR_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEPIVARUSCHAR_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writepivarunsignedchar_in
{
    ELMO_UINT16 usIndex;
    ELMO_UINT8  ucData;
} MMC_WRITEPIVARUCHAR_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEPIVARUSCHAR_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writepivarunsignedchar_out
{
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
} MMC_WRITEPIVARUCHAR_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEPIVARSHORT_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writepivarsignedshort_in
{
    ELMO_INT16  sData;
    ELMO_UINT16 usIndex;
} MMC_WRITEPIVARSHORT_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEPIVARSHORT_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writepivarsignedshort_out
{
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
} MMC_WRITEPIVARSHORT_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEPIVARUSHORT_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writepivarunsignedshort_in
{
    ELMO_UINT16 usData;
    ELMO_UINT16 usIndex;
} MMC_WRITEPIVARUSHORT_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEPIVARUSHORT_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writepivarunsignedshort_out
{
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
} MMC_WRITEPIVARUSHORT_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEPIVARINT_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writepivarsignedint_in
{
    ELMO_INT32  iData;
    ELMO_UINT16 usIndex;
} MMC_WRITEPIVARINT_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEPIVARINT_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writepivarsignedint_out
{
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
} MMC_WRITEPIVARINT_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEPIVARUINT_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writepivarunsignedint_in
{
    ELMO_UINT32 uiData;
    ELMO_UINT16 usIndex;
} MMC_WRITEPIVARUINT_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEPIVARUINT_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writepivarunsignedint_out
{
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
} MMC_WRITEPIVARUINT_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEPIVARFLOAT_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writepivarfloat_in
{
    ELMO_FLOAT   fData;
    ELMO_UINT16  usIndex;
} MMC_WRITEPIVARFLOAT_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEPIVARFLOAT_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writepivarfloat_out
{
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
} MMC_WRITEPIVARFLOAT_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEPIVARLONGLONG_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writelargepivarsignedlonglong_in
{
    ELMO_INT64  llData;
    ELMO_UINT16 usIndex;
} MMC_WRITEPIVARLONGLONG_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEPIVARLONGLONG_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writelargepivarsignedlonglong_out
{
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
} MMC_WRITEPIVARLONGLONG_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEPIVARULONGLONG_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writelargepivarunsignedlonglong_in
{
    ELMO_UINT64 ullData;
    ELMO_UINT16 usIndex;
} MMC_WRITEPIVARULONGLONG_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEPIVARULONGLONG_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writelargepivarunsignedlonglong_out
{
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
} MMC_WRITEPIVARULONGLONG_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEPIVARDOUBLE_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writelargepivardouble_in
{
    ELMO_DOUBLE  dbVal;
    ELMO_UINT16 usIndex;
} MMC_WRITEPIVARDOUBLE_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEPIVARDOUBLE_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writelargepivardouble_out
{
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
} MMC_WRITEPIVARDOUBLE_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEPIVARRAW_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writepivarraw_in
{
    ELMO_UINT16 usIndex;
    ELMO_UINT8  ucByteLength;
    ELMO_UINT8  pRawData[PI_REG_VAR_SIZE];
} MMC_WRITEPIVARRAW_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEPIVARRAW_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writepivarraw_out
{
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
} MMC_WRITEPIVARRAW_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEPIVARRAWLARGE_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writelargepivarraw_in
{
    ELMO_UINT16 usIndex;
    ELMO_UINT16 usByteLength;
    ELMO_UINT8  pRawData[PI_LARGE_VAR_SIZE];
} MMC_WRITELARGEPIVARRAW_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEPIVARRAWLARGE_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writelargepivarraw_out
{
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
} MMC_WRITELARGEPIVARRAW_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct NC_PI_ENTRY
/// \brief this struct describes an entry on the PI variables table
///////////////////////////////////////////////////////////////////////////////
typedef struct pientry
{
    ELMO_UINT32 uiBitSize;
    ELMO_UINT32 uiBitOffset;
    ELMO_UINT16 usCanOpenIndex;
    ELMO_UINT8  ucCanOpenSubIndex;
    ELMO_UINT8  ucVarType;
    ELMO_INT8   pAliasing[PI_ALIASING_LENGTH];
} NC_PI_ENTRY;

///////////////////////////////////////////////////////////////////////////////
/// \struct NC_PI_INFO_BY_ALIAS
/// \brief this struct describes an entry on the PI variables table
///////////////////////////////////////////////////////////////////////////////
typedef struct piinfobyalias
{
    ELMO_UINT32 uiBitSize;
    ELMO_UINT32 uiBitOffset;
    ELMO_UINT16 usCanOpenIndex;
    ELMO_UINT16 usPIVarOffset;
    ELMO_UINT8  ucCanOpenSubIndex;
    ELMO_UINT8  ucVarType;
    ELMO_UINT8  ucDirection;
    ELMO_UINT8  ucPadding;
} NC_PI_INFO_BY_ALIAS;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_GETPIVARINFO_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_getvarinfo_in
{
    ELMO_UINT16 usPIVarIndex;
    ELMO_UINT8  ucDirection;
} MMC_GETPIVARINFO_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_GETPIVARINFO_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_getpivarinfo_out
{
    NC_PI_ENTRY VarInfo;
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
} MMC_GETPIVARINFO_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_GETPIVARINFOBYALIAS_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////

typedef struct mmc_getpivarinfobyalias_in
{
    ELMO_INT8 pAliasing[PI_ALIASING_LENGTH];
} MMC_GETPIVARINFOBYALIAS_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_GETPIVARINFOBYALIAS_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_getpivarinfobyalias_out
{
    NC_PI_INFO_BY_ALIAS VarInfo;
    ELMO_UINT16         usStatus;
    ELMO_INT16          usErrorID;
} MMC_GETPIVARINFOBYALIAS_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_GETPIVARSRANGEINFO_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_getvarsrangeinfo_in
{
    ELMO_UINT16 usFirstIndex;
    ELMO_UINT16 usLastIndex;
    ELMO_UINT8  ucDirection;
} MMC_GETPIVARSRANGEINFO_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_GETPIVARSRANGEINFO_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_getvarsrangeinfo_out
{
    NC_PI_ENTRY pVarInfo[MAX_PI_VARIABLES_NUM];
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
} MMC_GETPIVARSRANGEINFO_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \enum COMM_TYPES
/// \brief network communication types
///////////////////////////////////////////////////////////////////////////////
enum COMM_TYPES
{
    eCOMM_TYPE_ETHERCAT = 1,
    eCOMM_TYPE_CAN = 2,
};

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_SETDEFAULTRESOURCES_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_setdefaultresources_in
{
    ELMO_UINT8 ucConnectionType;
} MMC_SETDEFAULTRESOURCES_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_SETDEFAULTRESOURCES_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_setdefaultresources_out
{
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
} MMC_SETDEFAULTRESOURCES_OUT;


///////////////////////////////////////////////////////////////////////////////
/// \enum MMC_DOWNLOAD_TYPE_ENUM
/// \brief Down load configuration file's type.
///////////////////////////////////////////////////////////////////////////////
typedef enum
{
    MMC_PARAMETER_FILE_DOWNLOAD = 0,        ///< Down load parameter file.
    MMC_RESOURCE_FILE_DOWNLOAD,             ///< Down load resource file.
    MMC_SNAPSHOT_RESOURCE_FILE_DOWNLOAD,    ///< Down load resource snapshot file
    MMC_ETHERCAT_CFG_FILE_DOWNLOAD,         ///< Down load resource ethercat file
    MMC_PERSONALITY_FILE_DOWNLOAD,          ///< Down load G-MAS personality file
    MMC_USER_FILE_DOWNLOAD,                 ///< Down load G-MAS user file
    MMC_SNAPSHOT_PARAMETER_FILE_DOWNLOAD,   ///< Create parameters file snapshot and Down load to PC
    MMC_CPLD_FILE_DOWNLOAD                  ///< Down load CPLD file.
}MMC_DOWNLOAD_TYPE_ENUM;

///////////////////////////////////////////////////////////////////////////////
/// \enum MMC_CONNECTION_TYPE
/// \brief Communication connection type.
///////////////////////////////////////////////////////////////////////////////
typedef enum {
    MMC_UNKNOWN_CONN_TYPE,  ///< Unknown connection type.
    MMC_RPC_CONN_TYPE,      ///< RPC connection to MMC server.
    MMC_IPC_CONN_TYPE,      ///< IPC connection to MMC server.
    MMC_INTERNAL_CONN_TYPE, ///< IPC connection to MMC server.
    MMC_IEC_CONN_TYPE,      ///< IEC connection to MMC server.
    MMC_MAX_CONN_TYPE
} MMC_CONNECTION_TYPE;

typedef enum
{
    eNC_BULKREAD_PRESET_NONE,
    eNC_BULKREAD_PRESET_1,
    eNC_BULKREAD_PRESET_2,
    eNC_BULKREAD_PRESET_3,
    eNC_BULKREAD_PRESET_4,
    eNC_BULKREAD_PRESET_5,
    eNC_BULKREAD_PRESET_MAX,
} NC_BULKREAD_PRESET_ENUM;

typedef enum
{
    eBULKREAD_CONFIG_NONE = -1,
    eBULKREAD_CONFIG_1    =  0,
    eBULKREAD_CONFIG_2    =  1,
    eBULKREAD_CONFIG_3    =  2,
    eBULKREAD_CONFIG_4    =  3,
    eBULKREAD_CONFIG_5    =  4,
    eBULKREAD_CONFIG_6    =  5,
    eBULKREAD_CONFIG_MAX,
} NC_BULKREAD_CONFIG_ENUM;

typedef enum
{
    eBULKREAD_CONFIG_PI_1   = 0,
    eBULKREAD_CONFIG_PI_2   = 1,
    eBULKREAD_CONFIG_PI_3   = 2,
    eBULKREAD_CONFIG_PI_4   = 3,
    eBULKREAD_CONFIG_PI_MAX
} NC_BULKREAD_CONFIG_PI_ENUM;

typedef union
{
    NC_BULKREAD_PRESET_ENUM eBulkReadPreset;
    ELMO_ULINT32            ulBulkReadParameters[NC_MAX_REC_SIGNALS_NUM];
} NC_BULKREAD_PARAMETERS_UNION;

typedef union
{
    ELMO_INT8   cVar;
    ELMO_UINT8  ucVar;
    ELMO_INT16  sVar;
    ELMO_UINT16 usVar;
    ELMO_INT32  iVar;
    ELMO_UINT32 uiVar;
    ELMO_LINT32 lVar;
    ELMO_ULINT32 ulVar;
} NC_BULKREAD_VARIABLE_UNION;

typedef enum
{
    eBULKREAD_OTHER_TYPE,
    eBULKREAD_USHORT_TYPE = 2,
    eBULKREAD_SHORT_TYPE  = 3,
} NC_BULKREAD_VAR_TYPE;

typedef struct
{
    ELMO_INT32  aPos;
    ELMO_INT32  aVel;
    ELMO_INT32  aTorque;
    ELMO_ULINT32 ulAxisStatus;
    ELMO_UINT32 uiInputs;
    OPM402      eOpMode;
} NC_BULKREAD_PRESET_1;

typedef struct
{
    NC_BULKREAD_PRESET_1*   stAxisParams;
    ELMO_INT32              iFreeLargeFbsNumber;
    ELMO_INT32              iFreeMediumFbsNumber;
    ELMO_INT32              iFreeSmallFbsNumber;
} NC_BULKREAD_PRESET_2;


typedef struct
{
    ELMO_INT32                  aPos;
    ELMO_INT32                  aVel;
    ELMO_INT32                  aTorque;
    ELMO_ULINT32                ulAxisStatus;
    ELMO_UINT32                 uiInputs;
    OPM402                      eOpMode;
    NC_BULKREAD_VARIABLE_UNION  ucCommError;
    NC_BULKREAD_VARIABLE_UNION  usLastEmcyErrorCode;
    NC_BULKREAD_VARIABLE_UNION  usControlWord;
    NC_BULKREAD_VARIABLE_UNION  usStatusWord;
} NC_BULKREAD_PRESET_3;


typedef struct
{
    ELMO_INT32  aPos;
    ELMO_INT32  aHWPos;
    ELMO_INT32  iPosFollowingErr;
    ELMO_INT32  aVel;
    ELMO_INT32  aTorque;
    ELMO_ULINT32 ulAxisStatus;
    ELMO_UINT32 uiInputs;
    OPM402      eOpMode;
    ELMO_UINT32 uiStatusRegister;
    ELMO_UINT32 uiMcsLimitRegister;
} NC_BULKREAD_PRESET_4;

typedef struct
{
    ELMO_INT32                  aPos;
    ELMO_INT32                  aHWPos;
    ELMO_INT32                  iPosFollowingErr;
    ELMO_INT32                  aVel;
    ELMO_INT32                  aTorque;
    ELMO_ULINT32                ulAxisStatus;
    ELMO_UINT32                 uiInputs;
    OPM402                      eOpMode;
    ELMO_UINT32                 uiStatusRegister;
    ELMO_UINT32                 uiMcsLimitRegister;
    NC_BULKREAD_VARIABLE_UNION  usLastEmcyErrorCode;
    NC_BULKREAD_VARIABLE_UNION  usControlWord;
    NC_BULKREAD_VARIABLE_UNION  usStatusWord;
    NC_BULKREAD_VARIABLE_UNION  ucCommError;
} NC_BULKREAD_PRESET_5;



///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_CONNECTION_PARAM_STRUCT
/// \brief Connection parameters structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT32 uiTcpPort;
    ELMO_UINT32 uiCbUdpPort;
    ELMO_UINT8  ucIp[16];
} MMC_CONNECTION_PARAM_STRUCT;


///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_CONNECTION_PARAM_STRUCT
/// \brief Connection parameters structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT32 uiParams ;     // Currently: bit0 - Explicit connection. All other connections will be closed.
} MMC_IPC_CONNECTION_PARAM_STRUCT;


///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_GET_VERPATH_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT8 dummy;
}MMC_GET_VERPATH_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_GET_VERPATH_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT16 usStatus;       ///< Returned command status.
    ELMO_INT16  usErrorID;      ///< Returned command error ID.
    ELMO_INT8   cVerPath[200];
}MMC_GET_VERPATH_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_CONFIG_IN
/// \brief Set MMC to Configuration Mode command input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_config_in
{
    ELMO_UINT8 dummy;
}MMC_CONFIG_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_CONFIG_OUT
/// \brief Set MMC to Configuration Mode command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT16 usStatus;    ///< Returned command status.
    ELMO_INT16  usErrorID;   ///< Returned command error ID.
} MMC_CONFIG_OUT;

//////////////////////////////////////////////////////////////////////////////
/// \struct MMC_EXIT_IN
/// \brief Exit from Configuration Mode command input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT8 dummy;
}MMC_EXIT_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_EXIT_OUT
/// \brief Exit from Configuration Mode command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT16 usStatus;   ///< Returned command status.
    ELMO_INT16  usErrorID;  ///< Returned command error ID.
} MMC_EXIT_OUT;


///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_GET_RESLIST_IN
/// \brief Get resource files list input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT8 dummy;
}MMC_GET_RESLIST_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_GET_RESLIST_OUT
/// \brief Get resource files list output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
    ELMO_INT8   pResList[1024];
}MMC_GET_RESLIST_OUT;


///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_GET_VER_IN
/// \brief Get Version Command input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT8 dummy;
}MMC_GET_VER_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_GET_VER_OUT
/// \brief Get Version Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT32 uiUbootVer;     ///< U-Boot version
    ELMO_UINT16 usStatus;       ///< Returned command status.
    ELMO_INT16  usErrorID;      ///< Returned command error ID.
    ELMO_INT8   cFirst;         ///< First byte of MMC version.
    ELMO_INT8   cSecond;        ///< Second byte of MMC version.
    ELMO_INT8   cThird;         ///< Third byte of MMC version.
    ELMO_INT8   cFourth;        ///< Fourth byte of MMC version.
} MMC_GET_VER_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_GET_VEREX_IN
/// \brief Get Version Ex Command input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT8 dummy;
}MMC_GET_VEREX_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_GET_VEREX_OUT
/// \brief Get Version Ex Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT16 usStatus;   ///< Returned command status.
    ELMO_INT16  usErrorID;  ///< Returned command error ID.
    ELMO_INT8 pcData[MAX_GETVERSION_CHARS];

} MMC_GET_VEREX_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_FBSTATUS_IN
/// \brief Get Function Block Status Command input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT32 uiHndl;    ///< Function Block handle.
} MMC_FBSTATUS_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_FBSTATUS_OUT
/// \brief Get Function Block Status Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT32 uiFbStatus;     ///< Returned Function Block status.
    ELMO_UINT16 usStatus;       ///< Returned command status.
    ELMO_INT16  usErrorID;      ///< Returned command error ID.
    ELMO_UINT16 usFbErrorID;    ///< Returned Function Block error ID.
} MMC_FBSTATUS_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_MARKFBFREE_IN
/// \brief  Mark Function Block Free Command input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT32 uiHndl;    ///< Function Block handle.
} MMC_MARKFBFREE_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_MARKFBFREE_OUT
/// \brief Mark Function Block Free Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT16 usStatus;   ///< Returned command status.
    ELMO_INT16  usErrorID;  ///< Returned command error ID.
} MMC_MARKFBFREE_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_FREEFBSTAT_IN
/// \brief Get Free Function Blocks Statistics Command input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT32  uiHndl;   ///< Requested Function Block handle.
}MMC_FREEFBSTAT_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_FREEFBSTAT_OUT
/// \brief Get Free Function Blocks Statistics Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT32 uiFreeLargeFb;  ///< Amount of free large size function blocks.
    ELMO_UINT32 uiFreeMediumFb; ///< Amount of free medium size function blocks.
    ELMO_UINT32 uiFreeSmallFb;  ///< Amount of free small size function blocks.
    ELMO_UINT16 usStatus;       ///< Returned command status.
    ELMO_INT16  usErrorID;      ///< Returned command error ID.
}MMC_FREEFBSTAT_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_SHOWNODESTAT_IN
/// \brief Show Axis/Group Status and Debug Data Command input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT32  uiHndl;   ///< Requested Node handle.
}MMC_SHOWNODESTAT_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_SHOWNODESTAT_OUT
/// \brief Show Axis/Group Status and Debug Data Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT16 usStatus;       ///< Returned command status.
    ELMO_INT16  usErrorID;      ///< Returned command error ID.
}MMC_SHOWNODESTAT_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_AXISBYNAME_IN
/// \brief Get Axis/Group Handle By Name Data Command input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_INT8 cAxisName[NODE_NAME_MAX_LENGTH];   ///< Axis/Group Name
}MMC_AXISBYNAME_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_AXISBYNAME_OUT
/// \brief Get Axis/Group Handle By Name Data Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT16 usStatus;       ///< Returned command status.
    ELMO_INT16  usErrorID;      ///< Returned command error ID.
    ELMO_UINT16 usAxisIdx;      ///< Axis/Group Index Reference.
}MMC_AXISBYNAME_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_BEGIN_RECORDING_IN
/// \brief Begin Data Recording Command input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_ULINT32 uiRg;                         ///< Recording Data Gap.
    ELMO_ULINT32 uiRl;                         ///< Recording Data Length.
    ELMO_ULINT32 uiRc;                         ///< Recording Data Signals Bit mask.
    ELMO_ULINT32 uiRv[NC_MAX_REC_SIGNALS_NUM]; ///< Recording Signals Id's.
    ELMO_ULINT32 uiRp[NC_MAX_REC_PARAMS_NUM];  ///< Recording Parameters.
}MMC_BEGIN_RECORDING_IN;


///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_BEGIN_ENHANCED_RECORDING_IN
/// \brief Begin Data Recording Command input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_ULINT32 uiRg;                                      ///< Recording Data Gap.
    ELMO_ULINT32 uiRl;                                      ///< Recording Data Length.
    ELMO_ULINT32 uiRv[NC_ENHANCED_MAX_REC_SIGNALS_NUM]; ///< Recording Signals Id's.
    ELMO_ULINT32 uiRp[NC_ENHANCED_REC_MAX_PARAMS_NUM];      ///< Recording Parameters.
    ELMO_ULINT32 uiNrv;
    ELMO_ULINT32 uiSpare[16];
}MMC_BEGIN_ENHANCED_RECORDING_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_BEGIN_RECORDING_OUT
/// \brief Begin Data Recording Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT16 usStatus;        ///< Returned command status.
    ELMO_INT16  usErrorID;       ///< Returned command error ID.
}MMC_BEGIN_RECORDING_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_STOP_RECORDING_IN
/// \brief Stop Data Recording Command input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT8 dummy;
}MMC_STOP_RECORDING_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_STOP_RECORDING_OUT
/// \brief Stop Data Recording Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT16 usStatus;       ///< Returned command status.
    ELMO_INT16  usErrorID;      ///< Returned command error ID.
}MMC_STOP_RECORDING_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_UPLOAD_DATA_IN
/// \brief Upload Recorded Data Command input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT32 uiFrom;    ///< Upload From Index.
    ELMO_UINT32 uiTo;      ///< Upload To Index
    ELMO_UINT32 uiBufIdx;  ///< Buffer Index.
}MMC_UPLOAD_DATA_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_UPLOAD_DATA_OUT
/// \brief Upload Recorded Data Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_LINT32 ulUpdatData[NC_MAX_LONG];   ///< Uploaded Data Array.
    ELMO_UINT16 usStatus;                   ///< Returned command status.
    ELMO_INT16  usErrorID;                  ///< Returned command error ID.
}MMC_UPLOAD_DATA_OUT;

typedef struct
{
    ELMO_INT8   cUpdateData[NC_MAX_REC_PACKET_SIZE];///< Uploaded Data Array.
    ELMO_UINT16 usStatus;                           ///< Returned command status.
    ELMO_INT16  usErrorID;                          ///< Returned command error ID.
}MMC_ENHANCED_REC_UPLOAD_DATA_OUT;


///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_REC_STATUS_IN
/// \brief Get Recording Status Command input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT8 dummy;
}MMC_REC_STATUS_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_REC_STATUS_OUT
/// \brief Get Recording Status Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_ULINT32    uiRr;           ///< Rest Recording Index.
    ELMO_ULINT32    uiSr;           ///< Recorder Trigger Status.
    ELMO_UINT16     usStatus;       ///< Returned command status.
    ELMO_INT16      usErrorID;      ///< Returned command error ID.
}MMC_REC_STATUS_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \enum NC_RV_TYPE_ENUM
/// \brief Recorder Supported Data Types.
///////////////////////////////////////////////////////////////////////////////
typedef enum {
    NC_UCHAR_TYPE = 0,
    NC_CHAR_TYPE,
    NC_USHORT_TYPE,
    NC_SHORT_TYPE,
    NC_UINT_TYPE,
    NC_INT_TYPE,
    NC_ULONG_TYPE,
    NC_LONG_TYPE,
    NC_FLOAT_TYPE,
    NC_DOUBLE_L_TYPE,
    NC_DOUBLE_H_TYPE
}NC_RV_TYPE_ENUM;

///////////////////////////////////////////////////////////////////////////////
/// \struct NC_REC_RV_STRUCT
/// \brief Recorder Signal Value Structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_ULINT32    ulValue;  ///< Signal Value Reference Handle.
    ELMO_ULINT32    ulType;   ///< Signal Value Type.
    ELMO_FLOAT       ulFactor;         ///< Signal Value Multiple Factor.
}NC_REC_RV_STRUCT;

///////////////////////////////////////////////////////////////////////////////
/// \struct NC_REC_RV_STRUCT
/// \brief Recorder Signal Value Structure.
///////////////////////////////////////////////////////////////////////////////
#pragma pack(push,4)
typedef struct 
{
    ELMO_ULINT32 ulValue;   ///< Signal Value Reference Handle.
    ELMO_DOUBLE  dFactor;   ///< Signal Value Multiple Factor.
}NC_ENHANCED_REC_RV_STRUCT;
#pragma pack(pop)


///////////////////////////////////////////////////////////////////////////////
/// \struct NC_UPLOAD_REC_HEADER_STRUCT
/// \brief Recorder Upload Data Header Structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct {
    ELMO_ULINT32    ulDummy;      ///< For alignment to upload data field in common message
    ELMO_ULINT32    ulRc;         ///< Recording Data Signals Bit mask.
    ELMO_ULINT32    ulRg;         ///< Recording Data Gap.
    ELMO_ULINT32    ulRl;         ///< Recording Data Length.
    NC_REC_RV_STRUCT usRv[NC_MAX_REC_SIGNALS_NUM];  ///< Recording Signals Id's.
    ELMO_ULINT32    ulRp[NC_MAX_REC_PARAMS_NUM];      ///< Recording Parameters.
    ELMO_ULINT32    ulTi;         ///< Trigger Index
    ELMO_ULINT32    ulTs;         ///< Recorder Update Time
    ELMO_ULINT32    ulSpare[3];   ///< Spare
    ELMO_UINT8      dummy [948] ;
    ELMO_UINT16     usStatus;    ///< Returned command status.
    ELMO_INT16      usErrorID;    ///< Returned command error ID.
}NC_UPLOAD_REC_HEADER_STRUCT;


///////////////////////////////////////////////////////////////////////////////
/// \struct NC_ENHACNED_UPLOAD_REC_HEADER_STRUCT
/// \brief Recorder Upload Data Header Structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct {
    ELMO_ULINT32 ulDummy;       ///< For alignment to upload data field in common message
    ELMO_ULINT32 ulRg;          ///< Recording Data Gap.
    ELMO_ULINT32 ulRl;          ///< Recording Data Length.
    NC_ENHANCED_REC_RV_STRUCT usRv[NC_ENHANCED_MAX_REC_SIGNALS_NUM];            ///< Recording Signals Id's.
    ELMO_UINT8   ucType[NC_ENHANCED_MAX_REC_SIGNALS_NUM];           ///< Signal Value Type.
    ELMO_ULINT32 ulRp[NC_ENHANCED_REC_MAX_PARAMS_NUM];      ///< Recording Parameters.
    ELMO_ULINT32 ulNrv;     ///< Recording Data Signals Bit mask.
    ELMO_ULINT32 ulTi;          ///< Trigger Index
    ELMO_ULINT32 ulTs;          ///< Recorder Update Time
    ELMO_ULINT32 ulSpare[3];    ///< Spare
    ELMO_UINT8   dummy [76] ;
    ELMO_UINT16  usStatus;  ///< Returned command status.
    ELMO_INT16   usErrorID; ///< Returned command error ID.
}NC_ENHANCED_UPLOAD_REC_HEADER_STRUCT;


///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_SET_VERPATH_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_INT8       cVerPath[200];
}MMC_SET_VERPATH_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_SET_VERPATH_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT16     usStatus;    ///< Returned command status.
    ELMO_INT16      usErrorID;   ///< Returned command error ID.
}MMC_SET_VERPATH_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_DOWNLOADVERSION_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_ULINT32    ulCRC;
}MMC_DOWNLOADVERSION_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_DOWNLOADVERSION_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT16     usStatus;
    ELMO_INT16      usErrorID;
}MMC_DOWNLOADVERSION_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_SETDBGSW_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT16     usSW;
}MMC_SETDBGSW_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_SETDBGSW_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT16     usStatus;
    ELMO_INT16      usErrorID;
}MMC_SETDBGSW_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_SAVEPARAM_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT8      dummy;
}MMC_SAVEPARAM_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_SAVEPARAM_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT16     usStatus;
    ELMO_INT16      usErrorID;
}MMC_SAVEPARAM_OUT;


///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_LOADPARAM_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT8      dummy;
}MMC_LOADPARAM_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_LOADPARAM_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT16     usStatus;
    ELMO_INT16      usErrorID;
}MMC_LOADPARAM_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_SETDEFAULTPARAMETERS_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT8      dummy;
}MMC_SETDEFAULTPARAMETERS_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_SETDEFAULTPARAMETERS_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT16     usStatus;
    ELMO_INT16      usErrorID;
}MMC_SETDEFAULTPARAMETERS_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_SETDEFAULTPARAMETERSGLOBAL_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT8      dummy;
}MMC_SETDEFAULTPARAMETERSGLOBAL_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_SETDEFAULTPARAMETERSGLOBAL_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT16     usStatus;
    ELMO_INT16      usErrorID;
}MMC_SETDEFAULTPARAMETERSGLOBAL_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_SETISTOLOADGLOBALPARAMS_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT8      ucValue;
}MMC_SETISTOLOADGLOBALPARAMS_IN;


///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_SETISTOLOADGLOBALPARAMS_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT16     usStatus;
    ELMO_INT16      usErrorID;
}MMC_SETISTOLOADGLOBALPARAMS_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_CREATEPERSONALITYPARAMETERS_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT8      dummy;
}MMC_CREATEPERSONALITYPARAMETERS_IN;


///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_CREATEPERSONALITYPARAMETERS_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT16     usStatus;
    ELMO_INT16      usErrorID;
}MMC_CREATEPERSONALITYPARAMETERS_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_RESSNAPSHOT_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT8      dummy;
}MMC_RESSNAPSHOT_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_RESSNAPSHOT_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT16     usStatus;
    ELMO_INT16      usErrorID;
}MMC_RESSNAPSHOT_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_RESEXPORTFILE_IN
/// \brief Export Resource File Command input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_INT8       pFName[33];        ///< Resource File Name.
    ELMO_INT8       pServer[17];       ///< Server....
    ELMO_INT8       pFilePath[101];    ///< Server File Path.
    ELMO_INT8       ucDownloadType;    ///< Down load type.
}MMC_RESEXPORTFILE_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_RESEXPORTFILE_OUT
/// \brief Export Resource File Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT16     usStatus;        ///< Returned command status.
    ELMO_INT16      usErrorID;       ///< Returned command error ID.
}MMC_RESEXPORTFILE_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_RESIMPORTFILE_IN
/// \brief Import Resource File Command input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_INT8       pFName[33];        ///< Resource File Name.
    ELMO_INT8       pServer[17];       ///< Server....
    ELMO_INT8       ucDownloadType;    ///< Down load type.
}MMC_RESIMPORTFILE_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_RESIMPORTFILE_OUT
/// \brief Import Resource File Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT16     usStatus;        ///< Returned command status.
    ELMO_INT16      usErrorID;       ///< Returned command error ID.
}MMC_RESIMPORTFILE_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_EXIT_APP_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT8      ucEnable;
}MMC_EXIT_APP_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_EXIT_APP_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT16     usStatus;
    ELMO_INT16      usErrorID;
}MMC_EXIT_APP_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READDOWNLOADVERSTATUS_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT8      ucDummy;
}MMC_READDOWNLOADVERSTATUS_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READDOWNLOADVERSTATUS_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT32     uiStatus;
    ELMO_UINT16     usStatus;
    ELMO_INT16      usErrorID;
}MMC_READDOWNLOADVERSTATUS_OUT;

typedef struct 
{
    ELMO_UINT8      ucDummy;
}MMC_SET_GMAS_PREOP_IN;

typedef struct 
{
    ELMO_UINT16     usStatus;
    ELMO_INT16      usErrorID;
}MMC_SET_GMAS_PREOP_OUT;

typedef struct 
{
    ELMO_UINT8      ucDummy;
}MMC_SET_GMAS_OP_IN;

typedef struct 
{
    ELMO_UINT16     usStatus;
    ELMO_INT16      usErrorID;
}MMC_SET_GMAS_OP_OUT;

typedef struct 
{
    ELMO_UINT8      ucDummy;
}MMC_GET_GMASOP_MODE_IN;

typedef struct 
{
    ELMO_UINT16     usStatus;
    ELMO_INT16      usErrorID;
    ELMO_UINT8      ucResult;
}MMC_GET_GMASOP_MODE_OUT;


typedef struct foe_slave_info
{
    ELMO_UINT16     usSlaveID;
    ELMO_INT16      sErrorID;
}FOE_SLAVE_INFO;

typedef struct mmc_downloadfoe_in
{
    ELMO_UINT16     pwSlaveId[NC_NODES_SING_AXIS_NUM];
    ELMO_INT8       pcFileName[256];
    ELMO_UINT8      pucServer[4];
    ELMO_UINT8      ucSlavesNum;
}MMC_DOWNLOADFOE_IN;


typedef struct mmc_downloadfoe_out
{
    ELMO_UINT16     usStatus;
    ELMO_UINT16     usErrorID;
}MMC_DOWNLOADFOE_OUT;


typedef struct mmc_getfoestatus_in
{
    ELMO_UINT8      ucDummy;
}MMC_GETFOESTATUS_IN;

typedef struct mmc_getfoestatus_out
{
    ELMO_UINT16     usStatus;
    ELMO_UINT16     usErrorID;
    ELMO_INT16      sFOEStatus;
    FOE_SLAVE_INFO pstSlavesErrorID[NC_NODES_SING_AXIS_NUM];
    ELMO_UINT8      ucNumOfSlaves;
    ELMO_UINT8      ucProgress ;
    ELMO_UINT8      ucFOEStarted ;
}MMC_GETFOESTATUS_OUT;


/*************************************************/

#define ETHERCAT_STATISTICSEX_MAX_SLAVES    76

typedef struct _MMC_ECAT_SII_CONTENT
{
    ELMO_ULINT32    ulVendorId;
    ELMO_ULINT32    ulProductCode;
    ELMO_ULINT32    ulRevisionNo;
    ELMO_ULINT32    ulSerialNo;
} MMC_ECAT_SII_CONTENT;

typedef struct mmc_getcommstatisticsex_in
{
    ELMO_UINT16 pwSlaveId[ETHERCAT_STATISTICSEX_MAX_SLAVES];
    ELMO_UINT8  ucSlavesNum;
}MMC_GETCOMMSTATISTICSEX_IN;

typedef struct mmc_getcommstatisticsex_out
{
    ELMO_ULINT32    dwSendErrors;
    ELMO_ULINT32    dwReceiveErrors;
    ELMO_ULINT32    dwWrongWC;
    ELMO_ULINT32    dwParseErrors;
    MMC_ECAT_SII_CONTENT pstSII_Content[ETHERCAT_STATISTICSEX_MAX_SLAVES];
    ELMO_UINT16     usNumOfSlaves;
    ELMO_UINT16     usStatus;
    ELMO_UINT16     usErrorID;
    ELMO_UINT8      ucMasterState;
    ELMO_UINT8      pucAxesState[ETHERCAT_STATISTICSEX_MAX_SLAVES];
    ELMO_UINT8      pucAxesDiagnosticState[ETHERCAT_STATISTICSEX_MAX_SLAVES];
    ELMO_UINT8      ucMasterDiagnosticState;
}MMC_GETCOMMSTATISTICSEX_OUT;

/* Diagnostic */
typedef enum tagEcatSlaveDiagnosticState
{
    /*! Slave doesn't respond to commands. */
    EcatSlaveDiagnosticStateOffLine             = 0x00000001,
    /*! Slave's state is different as the set one. */
    EcatSlaveDiagnosticStateErrorEcatState      = 0x00000002,
    /*! Slave is not configured. */
    EcatSlaveDiagnosticStateNotConfigured       = 0x00000004,
    /*! Slave's configuration doesn't match the configuration for this slave found in master. */
    EcatSlaveDiagnosticStateWrongConfiguration  = 0x00000008,
    EcatSlaveDiagnosticStateInitCmdError        = 0x00000010,
    EcatSlaveDiagnosticStateMailboxInitCmdError = 0x00000020,
}EcatSlaveDiagnosticState;

typedef enum tagEcatMasterDiagnosticState
{
    /*! Diagnostics completed successfully. */
    EcatMasterDiagnosticStateUpdated            = 0x00000001,
    /*! Error while sending/receiving a frame. */
    EcatMasterDiagnosticStateSendReceiveError   = 0x00000002,
    /*! Error while processing the received frame. */
    EcatMasterDiagnosticStateParseError         = 0x00000004,
    /*! No connection between the NIC adapter and slaves. */
    EcatMasterDiagnosticStateLinkDown           = 0x00000008,
    /*! Wrong configuration */
    EcatMasterDiagnosticStateWrongConfiguration = 0x00000010,
    /*! Slave-to-slave timeout */
    EcatMasterDiagnosticStateS2STimeout         = 0x00000020,
    /*! Default data was set */
    EcatMasterDiagnosticStateDefaultDataWasSet  = 0x00000040,
     /*! WatchDogTimeOut */
    EcatMasterDiagnosticStateWatchDogTimeOut    = 0x00000080,
}EcatMasterDiagnosticState;


/**************************************************/


typedef struct mmc_configbulkread_in
{
    NC_BULKREAD_PARAMETERS_UNION uBulkReadParams;
    NC_BULKREAD_CONFIG_ENUM eConfiguration;
    ELMO_UINT16     usAxisRefArray[NC_MAX_AXES_PER_BULK_READ];
    ELMO_UINT16     usNumberOfAxes;
    ELMO_UINT8      ucIsPreset;
} MMC_CONFIGBULKREAD_IN;

typedef struct mmc_configbulkread_out
{
    ELMO_FLOAT       fFactorsArray[NC_MAX_BULK_READ_READABLE_PACKET_SIZE];
    ELMO_UINT16     usStatus;
    ELMO_INT16      usErrorID;
} MMC_CONFIGBULKREAD_OUT;

typedef struct
{
    ELMO_UINT16 usIndex;
    ELMO_UINT16 usAxisRef;   
    ELMO_UINT8  ucPiDirection;
    ELMO_UINT8  pPadding[3];
}PI_BULKREAD_ENTRY;

typedef struct
{
    PI_BULKREAD_ENTRY           pVarsArray[NC_MAX_PI_BULK_READ_VARIABLES];
    NC_BULKREAD_CONFIG_PI_ENUM  eConfiguration;
}MMC_CONFIGBULKREADPI_IN;

typedef struct
{
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
}MMC_CONFIGBULKREADPI_OUT;

typedef struct mmc_performbulkread_in
{
    NC_BULKREAD_CONFIG_ENUM eConfiguration;
} MMC_PERFORMBULKREAD_IN;

typedef struct mmc_performbulkread_out
{
    ELMO_ULINT32    ulOutBuf[NC_MAX_BULK_READ_READABLE_PACKET_SIZE];
    NC_BULKREAD_PRESET_ENUM eChosenPreset;
    ELMO_UINT16     usStatus;
    ELMO_INT16      usErrorID;
} MMC_PERFORMBULKREAD_OUT;

typedef struct mmc_performbulkreadpi_in
{
    NC_BULKREAD_CONFIG_PI_ENUM eConfiguration;
} MMC_PERFORMBULKREADPI_IN;

typedef struct mmc_performbulkreadpi_out
{
    ELMO_ULINT32    ulOutBuf[NC_MAX_BULK_READ_READABLE_PACKET_SIZE];
    ELMO_UINT16     usStatus;
    ELMO_INT16      usErrorID;
} MMC_PERFORMBULKREADPI_OUT;

typedef struct mmc_toggleconsoleoutput_in
{
    ELMO_UINT8  ucEnable;
} MMC_TOGGLECONSOLEOUTPUT_IN;

typedef struct mmc_toggleconsoleoutput_out
{
    ELMO_INT16  usErrorId;
} MMC_TOGGLECONSOLEOUTPUT_OUT;



typedef struct {
    ELMO_UINT8  dummy;
}MMC_GETACTIVEAXESNUM_IN;

typedef struct {
    ELMO_INT32  iActiveAxesNum;
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
}MMC_GETACTIVEAXESNUM_OUT;

typedef struct {
    ELMO_UINT8  dummy;
}MMC_GETACTIVEVECTORSNUM_IN;

typedef struct {
    ELMO_INT32  iActiveVectorsNum;
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
}MMC_GETACTIVEVECTORSNUM_OUT;

typedef struct mmc_clearfblist_in
{
    ELMO_UINT16 usAxisRef;
} MMC_CLEARFBLIST_IN;

typedef struct mmc_clearfblist_out
{
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
} MMC_CLEARFBLIST_OUT;

typedef struct mmc_geterrorcodedescriptionbyid_in
{
    ELMO_INT32  iCode;
    ELMO_INT8   cType;
} MMC_GETERRORCODEDESCRIPTIONBYID_IN;

typedef struct mmc_geterrorcodedescriptionbyid_out
{
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
    ELMO_INT8   pResolution[1100];
    ELMO_INT8   pDescription[256];
} MMC_GETERRORCODEDESCRIPTIONBYID_OUT;


typedef struct mmc_getstatusregister_in
{
    ELMO_UINT8  dummy;
} MMC_GETSTATUSREGISTER_IN;

typedef struct mmc_getstatusregister_out
{
    ELMO_UINT32 uiStatusRegister;
    ELMO_UINT32 uiMcsLimitRegister;
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
    ELMO_UINT8  ucEndMotionReason;
    ELMO_UINT8  cBuffer[32];
} MMC_GETSTATUSREGISTER_OUT;


typedef struct
{
    ELMO_UINT8 ucDummy;
} MMC_GETCYCLESCOUNTER_IN;

typedef struct
{
    ELMO_ULINT32    ulCyclesCounter;
    ELMO_UINT16     usStatus;
    ELMO_INT16      usErrorID;
} MMC_GETCYCLESCOUNTER_OUT;

typedef struct
{
    ELMO_ULINT32 ulDwellTimeMs;
} MMC_DWELL_IN;

typedef struct
{
    ELMO_UINT32 uiHandle;
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
} MMC_DWELL_OUT;

typedef struct mmc_waituntilconditionfb_in
{
    ELMO_DOUBLE     dbReferenceValue;
    ELMO_INT32      iParameterID;
    ELMO_INT32      iParameterIndex;
    MC_CONDITIONFB_OPERATION_TYPE eOperationType;
    ELMO_ULINT32    ulSpare;
    ELMO_UINT16     usSourceAxisReference;
    ELMO_UINT8      ucExecute;
    ELMO_UINT8      ucPadding;
    ELMO_UINT8      ucSpare[20];
}MMC_WAITUNTILCONDITIONFB_IN;


typedef struct mmc_waituntilconditionfb_out
{
    ELMO_UINT32     uiHndl;
    ELMO_UINT16     usStatus;
    ELMO_INT16      usErrorID;
}MMC_WAITUNTILCONDITIONFB_OUT;

typedef struct mmc_waituntilconditionfbex_in
{
    ELMO_DOUBLE     dbReferenceValue;
    ELMO_INT32      iParameterID;
    ELMO_INT32      iParameterIndex;
    MC_CONDITIONFB_OPERATION_TYPE eOperationType;
    ELMO_ULINT32    ulSpare;
    ELMO_UINT16     usSourceAxisReference;
    ELMO_UINT8      ucExecute;
    ELMO_UINT8      ucDirection;
    ELMO_UINT8      ucSpare[20];
}MMC_WAITUNTILCONDITIONFBEX_IN;


typedef struct mmc_waituntilconditionfbex_out
{
    ELMO_UINT32 uiHndl;
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
}MMC_WAITUNTILCONDITIONFBEX_OUT;


typedef struct mmc_killrepetitive_in
{
    ELMO_UINT32 uiDummy;
}MMC_KILLREPETITIVE_IN;


typedef struct mmc_killrepetitive_out
{
    ELMO_UINT16 usStatus;
    ELMO_UINT16 usErrorID;
}MMC_KILLREPETITIVE_OUT;


typedef struct mmc_closeopenedipcconn_in
{
    ELMO_UINT8 ucDummy;
}MMC_CLOSEOPENEDIPCCONN_IN;


typedef struct mmc_closeopenedipcconn_out
{
    ELMO_UINT16 usStatus;
    ELMO_INT16  usErrorID;
}MMC_CLOSEOPENEDIPCCONN_OUT;


///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_USRCOMMADN_IN
/// \brief Execute user command (run user program or linux command).
///
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    MC_COMMAND_OPERATION    eUsrCommandOp;
    ELMO_INT8               cUserCommand[256];
    ELMO_UINT8              ucSpare[20];
} MMC_USRCOMMAND_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_USRCOMMADN_OUT
/// \brief USER COMMAND OUT structure
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT16 usStatus;    ///< Returned command status.
    ELMO_INT16  usErrorID;        ///< Returned command error ID.
    ELMO_UINT8  ucIsRunning;
    ELMO_UINT8  ucIsExist;
    ELMO_INT8   cExecutableFileName[64];
    ELMO_INT8   cSpear[448];
} MMC_USRCOMMAND_OUT;

typedef struct
{
    ELMO_UINT16 usAxisRef;
} MMC_SETALLFBEXEMODETOIMM_IN;
typedef struct
{
    ELMO_UINT16 usStatus;       ///< Returned command status.
    ELMO_INT16  usErrorID;      ///< Returned command error ID.
} MMC_SETALLFBEXEMODETOIMM_OUT;

typedef struct mmc_getgmasinfo_in
{
    ELMO_UINT8 ucDummy;
} MMC_GETGMASINFO_IN;

typedef struct mmc_getgmasinfo_out
{
    ELMO_UINT8      ucDeviceType;
    ELMO_UINT8      ucPadding1;
    ELMO_UINT8      ucPadding2;
    ELMO_UINT8      ucPadding3;
    ELMO_ULINT32    ulSpear[4];
    ELMO_UINT16     usStatus;    ///< Returned command status.
    short sErrorID;             ///< Returned command error ID.
} MMC_GETGMASINFO_OUT;

typedef struct
{
    ELMO_UINT8 ucDummy;
} MMC_GETSYSTEMDATA_IN;

typedef struct
{
    ELMO_UINT32 uiGlobalParamsDataOffset;
    ELMO_UINT32 uiNodeDataOffset;
    ELMO_UINT32 uiKernelBaseAdd;
} MMC_GETSYSTEMDATA_OUT;

typedef struct mmc_getMaestroTime_in
{
    ELMO_INT8 cSpares[32];
} MMC_GET_MAESTRO_TIME_IN;

typedef struct mmc_getMaestroTime_out
{
    ELMO_UINT16 usStatus;   ///< Returned command status.
    ELMO_INT16 sErrorID;                ///< Returned command error ID.
    ELMO_UINT16 usMonth;
    ELMO_UINT16 usDay;
    ELMO_UINT16 usYear;
    ELMO_UINT16 usHour;
    ELMO_UINT16 usMinute;
    ELMO_UINT16 usSecond;
    ELMO_UINT16 usMillisecond;
    ELMO_INT8 cSpares[16];
} MMC_GET_MAESTRO_TIME_OUT;

typedef struct mmc_setMaestroTime_in
{
    ELMO_UINT16 usMonth;
    ELMO_UINT16 usDay;
    ELMO_UINT16 usYear;
    ELMO_UINT16 usHour;
    ELMO_UINT16 usMinute;
    ELMO_UINT16 usSecond;
    ELMO_UINT16 usMillisecond;
    ELMO_INT8 cSpares[16];
} MMC_SET_MAESTRO_TIME_IN;

typedef struct mmc_setMaestroTime_out
{
    ELMO_UINT16 usStatus;   ///< Returned command status.
    ELMO_INT16  sErrorID;   ///< Returned command error ID.
} MMC_SET_MAESTRO_TIME_OUT;


typedef struct
{
    ELMO_ULINT32 ulSpares[50];
} MMC_BURN_CPLD_IN;

typedef struct
{
    ELMO_UINT16 usStatus;   ///< Returned command status.
    ELMO_INT8 sErrorID;     ///< Returned command error ID.
} MMC_BURN_CPLD_OUT;

//@ML CPLD
typedef struct
{
    ELMO_UINT32 ulSpare[50];
    ELMO_UINT8 ucConfigFunc;
} MMC_CONFIG_CPLD_IN;

typedef struct
{
    ELMO_UINT16 usStatus;               ///< Returned command status.
    ELMO_INT16  sErrorID;               ///< Returned command error ID.
    ELMO_UINT8  ucFuncError;            ///< Which configuration functionality failed.
    ELMO_UINT8  ucIOError;              ///< In case of RS485 error - Which IO configuration failed.               
} MMC_CONFIG_CPLD_OUT;

typedef struct
{
    NC_REC_PARAM_NAMES_ENUM eNodeParam[300];
    ELMO_UINT16 usRefAxis;
} MMC_GETNODEDIRECTDATA_IN;
typedef struct
{
//  unsigned long uldPosOffset;
    ELMO_ULINT32 ulNodeDataOffset[300];
    ELMO_UINT16  usStatus;      ///< Returned command status.
    ELMO_INT16   sErrorID;      ///< Returned command error ID.
} MMC_GETNODEDIRECTDATA_OUT;

typedef struct
{
    ELMO_UINT8 ucDummy;
} MMC_GETGLBLPARAMOFFSET_IN;

typedef struct
{
    ELMO_ULINT32 ulGblParamOffset[100];
    ELMO_UINT16  usStatus;      ///< Returned command status.
    ELMO_INT16   sErrorID;      ///< Returned command error ID.
} MMC_GETGLBLPARAMOFFSET_OUT;

typedef struct
{
    ELMO_UINT8 ucNumberofAxes;
}MMC_GETAXESNAME_IN;

typedef struct
{
    ELMO_INT8   pAxesNames[NC_NODES_SING_AXIS_NUM][NODE_NAME_MAX_LENGTH];
    ELMO_UINT16 usStatus;       ///< Returned command status.
    ELMO_INT16  sErrorID;       ///< Returned command error ID.
}MMC_GETAXESNAME_OUT;

typedef struct
{
    ELMO_UINT8 ucDummy;
}MMC_GETPINUMBER_IN;

typedef struct
{
    ELMO_UINT32 uiNumberOfPIInput;
    ELMO_UINT32 uiNumberOfPIOutPut;
    ELMO_UINT16 usStatus;       ///< Returned command status.
    ELMO_INT16  sErrorID;       ///< Returned command error ID.
}MMC_GETPINUMBER_OUT;


////////////////////////////////////////////////////////////////////////////////
/// \fn MMC_LIB_API int MMC_InitConnection(
///             IN MMC_CONNECTION_TYPE eType,
///             IN MMC_CONNECTION_PARAM_STRUCT sConnParam,
///             IN MMC_CB_FUNC pCbFunc,
///             OUT MMC_CONNECT_HNDL* pHndl)
/// \brief This function initiates connection to MMC server.
/// \param  eType - [IN] Connection type (RPC or IPC)
/// \param  sConnParam - [IN] Connection parameters. (Like IP, port, UDP poort for callback)
/// \param  pCbFunc - [IN] Pointer to UDP Callback Function.
/// \param  pHndl - [OUT] Connection handle.
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_InitConnection(
        IN MMC_CONNECTION_TYPE          eType,
        IN MMC_CONNECTION_PARAM_STRUCT  sConnParam,
        IN MMC_CB_FUNC                  pCbFunc,
        OUT MMC_CONNECT_HNDL*           pHndl);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_IPCInitConnection(
///             IN MMC_IPC_CONNECTION_PARAM_STRUCT sConnParam,
///             IN MMC_CB_FUNC pCbFunc,
///             OUT MMC_CONNECT_HNDL* pHndl)
/// \brief This function initiates IPC connection to MMC.
/// \param MMC_IPC_CONNECTION_PARAM_STRUCT sConnParam - [IN] Connection parameters. (Currently only Explicit connection supported).
/// \param MMC_CB_FUNC pCbFunc - [IN]
/// \param MMC_CONNECT_HNDL* pHndl - [OUT] Connection handle
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
int MMC_IPCInitConnection(
        IN MMC_IPC_CONNECTION_PARAM_STRUCT sConnParam,
        IN MMC_CB_FUNC pCbFunc ,
        OUT MMC_CONNECT_HNDL* pHndl) ;


////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_RpcInitConnection(
///             IN MMC_CONNECTION_TYPE eType,
///             IN MMC_CONNECTION_PARAM_STRUCT sConnParam,
///             IN MMC_CB_FUNC pCbFunc,
///             OUT MMC_CONNECT_HNDL* pHndl)
/// \brief This function initiates connection to MMC.
/// \param MMC_CONNECTION_TYPE eType - [IN] Connection type (RPC or IPC)
/// \param MMC_CONNECTION_PARAM_STRUCT sConnParam - [IN] Connection parameters. (Like IP, port, UDP poort for callback)
/// \param MMC_CB_FUNC pCbFunc - [IN]
/// \param char* cpHostIPAddr - [IN]    - Host IP Address for multiple NIC support.
/// \param MMC_CONNECT_HNDL* pHndl - [OUT] Connection handle
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_RpcInitConnection(
        IN MMC_CONNECTION_TYPE  eType,
        IN MMC_CONNECTION_PARAM_STRUCT sConnParam,
        IN MMC_CB_FUNC          pCbFunc ,
        IN ELMO_PINT8           cpHostIPAddr,
        OUT MMC_CONNECT_HNDL*   pHndl);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_RpcInitConnectionEx(
///             IN MMC_CONNECTION_TYPE eType,
///             IN MMC_CONNECTION_PARAM_STRUCT sConnParam,
///             IN MMC_CB_FUNC pCbFunc,
///             OUT MMC_CONNECT_HNDL* pHndl)
/// \brief This function initiates connection to MMC.
/// \param MMC_CONNECTION_TYPE eType - [IN] Connection type (RPC or IPC)
/// \param MMC_CONNECTION_PARAM_STRUCT sConnParam - [IN] Connection parameters. (Like IP, port, UDP poort for callback)
/// \param MMC_CB_FUNC pCbFunc - [IN]
/// \param char* cpHostIPAddr - [IN]    - Host IP Address for multiple NIC support.
/// \param MMC_CONNECT_HNDL* pHndl - [OUT] Connection handle
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_RpcInitConnectionEx(
        IN MMC_CONNECTION_TYPE          eType,
        IN MMC_CONNECTION_PARAM_STRUCT  sConnParam,
        IN MMC_MB_CLBK                  pCbFunc ,
        IN ELMO_PINT8                   cpHostIPAddr,
        OUT MMC_CONNECT_HNDL*           pHndl);


///////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_CloseConnection(
///             IN MMC_CONNECT_HNDL hConn)
/// \brief This function close connection to MMC.
/// \param MMC_CONNECT_HNDL hConn - [IN] Connection handle
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_CloseConnection(
        IN MMC_CONNECT_HNDL hConn);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ConfigCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             OUT MMC_CONFIG_OUT* pOutParam)
/// \brief This function set MMC to configuration mode and allow to change any
///         configuration parameters.
/// \param  hConn - [IN] Connection handle
/// \param  pOutParam - [OUT] Set To Configuration State output data structure
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ConfigCmd(
        IN MMC_CONNECT_HNDL hConn,
        OUT MMC_CONFIG_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ExitCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             OUT MMC_EXIT_OUT* pOutParam)
/// \brief This function exit MMC from configuration mode  back to regular mode
/// \param  hConn - [IN] Connection handle
/// \param  pOutParam - [OUT] Exit from  Configuration Mode output data structure
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ExitCmd(
        IN MMC_CONNECT_HNDL hConn,
        OUT MMC_EXIT_OUT*   pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_GetResListCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_GET_RESLIST_IN* pInParam,
///             OUT MMC_GET_RESLIST_OUT* pOutParam)
/// \brief This function return list of all resource files
/// \param  hConn - [IN] Connection handle
/// \param  pInParam - [IN] Resource files list input data structure
/// \param  pOutParam - [OUT] Resource files list output data structure
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_GetResListCmd(
        IN MMC_CONNECT_HNDL         hConn,
        IN MMC_GET_RESLIST_IN*      pInParam,
        OUT MMC_GET_RESLIST_OUT*    pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_GetResSnapshotCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_RESSNAPSHOT_IN* pInParam,
///             OUT MMC_RESSNAPSHOT_OUT* pOutParam)
/// \brief This function save resource configuration to temporary snapshot file.
/// \param  hConn - [IN] Connection handle
/// \param  pInParam - [IN] Resource File Snapshot input data structure
/// \param  pOutParam - [OUT] Resource File Snapshot output data structure
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_GetResSnapshotCmd(
        IN MMC_CONNECT_HNDL         hConn,
        IN MMC_RESSNAPSHOT_IN*      pInParam,
        OUT MMC_RESSNAPSHOT_OUT*    pOutParam);


////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_GetAxisByNameCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXISBYNAME_IN* pInParam,
///             OUT MMC_AXISBYNAME_OUT* pOutParam)
/// \brief This function return axis index reference by his name
/// \param  hConn - [IN] Connection handle
/// \param  pInParam - [IN] Axis By Name input data structure
/// \param  pOutParam - [OUT] Axis By Name output data structure
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_GetAxisByNameCmd(
        IN MMC_CONNECT_HNDL     hConn,
        IN MMC_AXISBYNAME_IN*   pInParam,
        OUT MMC_AXISBYNAME_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_GetGroupByNameCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXISBYNAME_IN* pInParam,
///             OUT MMC_AXISBYNAME_OUT* pOutParam)
/// \brief This function return group index reference by his name
/// \param  hConn - [IN] Connection handle
/// \param  pInParam - [IN] Axis By Name input data structure
/// \param  pOutParam - [OUT] Axis By Name output data structure
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_GetGroupByNameCmd(
        IN MMC_CONNECT_HNDL     hConn,
        IN MMC_AXISBYNAME_IN*   pInParam,
        OUT MMC_AXISBYNAME_OUT* pOutParam);

/// \fn int MMC_GetVersionCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             OUT MMC_GET_VER_OUT* sVersion)
/// \brief This function send "Get Version" command to MMC and
/// get MMC version in output parameter
/// \param  hConn - [IN] Connection handle
/// \param  sVersion - [OUT] Version data structure
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_GetVersionCmd(
        IN MMC_CONNECT_HNDL     hConn,
        OUT MMC_GET_VER_OUT*    sVersion);

/// \fn int MMC_GetVersionExCmd(
///             IN MMC_CONNECT_HNDL     hConn,
///             OUT MMC_GET_VEREX_OUT*  sVersion)
/// \brief This function send "Get Version" command to MMC and
/// get MMC version in output parameter
/// \param  hConn - [IN] Connection handle
/// \param  sVersion - [OUT] Version data structure
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_GetVersionExCmd(
        IN MMC_CONNECT_HNDL     hConn,
        OUT MMC_GET_VEREX_OUT*  sVersion);



////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_CmdStatus(
///             IN MMC_CONNECT_HNDL     hConn,
///             IN MMC_FBSTATUS_IN*     pInParam,
///             OUT MMC_FBSTATUS_OUT*   pOutParam)
/// \brief This function This function  send Read Function Block Status command to MMC server for specific Axis/Group
/// and receive status back.
/// \param  hConn - [IN] Connection handle
/// \param  pInParam - [IN] Pointer to Command Status input parameters structure.
/// \param  pOutParam - [OUT] Pointer to Command Status output parameters structure.
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_CmdStatus(
        IN MMC_CONNECT_HNDL     hConn,
        IN MMC_FBSTATUS_IN*     pInParam,
        OUT MMC_FBSTATUS_OUT*   pOutParam);


////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_MarkFbFree(
///             IN MMC_CONNECT_HNDL     hConn,
///             IN MMC_MARKFBFREE_IN*   pInParam,
///             OUT MMC_MARKFBFREE_OUT* pOutParam)
/// \brief This function
/// \param  hConn - [IN] Connection handle
/// \param  pInParam - [IN] Pointer to Mark FB Free input parameters structure.
/// \param  pOutParam - [OUT] Pointer to Mark FB Free output parameters structure.
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_MarkFbFree(
        IN MMC_CONNECT_HNDL     hConn,
        IN MMC_MARKFBFREE_IN*   pInParam,
        OUT MMC_MARKFBFREE_OUT* pOutParam);

///////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_FreeFbStatCmd(
///             IN MMC_CONNECT_HNDL     hConn,
///             IN MMC_FREEFBSTAT_IN*   pInParam,
///             OUT MMC_FREEFBSTAT_OUT* pOutParam)
/// \brief This function return debug information that contain amount free function blocks in system.
/// \param  hConn - [IN] Connection handle
/// \param  pInParam - [IN] Pointer to Free F.B. Status input parameters structure.
/// \param  pOutParam - [OUT] Pointer to Free F.B. Status output parameters structure.
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_FreeFbStatCmd(
        IN MMC_CONNECT_HNDL     hConn,
        IN MMC_FREEFBSTAT_IN*   pInParam,
        OUT MMC_FREEFBSTAT_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ShowNodeStatCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_SHOWNODESTAT_IN* pInParam,
///             OUT MMC_SHOWNODESTAT_OUT* pOutParam)
/// \brief This function print debug information for Axis/Group
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis/Group Reference handle
/// \param  pInParam - [IN] Pointer to Show Node Statistics input parameters structure.
/// \param  pOutParam - [OUT] Pointer to Show Node Statistics output parameters structure.
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ShowNodeStatCmd(
        IN MMC_CONNECT_HNDL         hConn,
        IN MMC_AXIS_REF_HNDL        hAxisRef,
        IN MMC_SHOWNODESTAT_IN*     pInParam,
        OUT MMC_SHOWNODESTAT_OUT*   pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_BeginRecordingCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_BEGIN_RECORDING_IN* pInParam,
///             OUT MMC_BEGIN_RECORDING_OUT* pOutParam)
/// \brief This function send Begin Recording command to MMC.
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to Begin Recording input parameters
/// \param pOutParam - [OUT] Pointer to Begin Recording output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_BeginRecordingCmd(
        IN MMC_CONNECT_HNDL             hConn,
        IN MMC_BEGIN_RECORDING_IN*      pInParam,
        OUT MMC_BEGIN_RECORDING_OUT*    pOutParam);

MMC_LIB_API ELMO_INT32 MMC_BeginRecordingCmdEX(
        IN MMC_CONNECT_HNDL             hConn,
        IN MMC_BEGIN_RECORDING_IN*      pInParam,
        OUT MMC_BEGIN_RECORDING_OUT*    pOutParam);


////////////////////////////////////////////////////////////////////////////////
/// \fn int BeginEnhancedRecordingCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_BEGIN_ENHANCED_RECORDING_IN* pInParam,
///             OUT MMC_BEGIN_RECORDING_OUT* pOutParam)
/// \brief This function...
/// \param MMC_CONNECT_HNDL hConn - [IN] Connection handle
/// \param MMC_BEGIN_ENHANCED_RECORDING_IN* pInParam - [IN] Pointer to Begin Enhanced Recording input parameters
/// \param MMC_BEGIN_RECORDING_OUT* pOutParam - [OUT] Pointer to Begin Recording output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API int MMC_BeginEnhancedRecordingCmd(
        IN MMC_CONNECT_HNDL hConn,
        IN MMC_BEGIN_ENHANCED_RECORDING_IN* pInParam,
        OUT MMC_BEGIN_RECORDING_OUT* pOutParam);


////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_StopRecordingCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_STOP_RECORDING_IN* pInParam,
///             OUT MMC_STOP_RECORDING_OUT* pOutParam)
/// \brief This function Stop Recording Command to MMC.
/// \param  hConn - [IN] Connection handle
/// \param  pInParam - [IN] Pointer to Stop Recording input parameters
/// \param  pOutParam - [OUT] Pointer to Stop Recording output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_StopRecordingCmd(
        IN MMC_CONNECT_HNDL         hConn,
        IN MMC_STOP_RECORDING_IN*   pInParam,
        OUT MMC_STOP_RECORDING_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_UploadDataHeaderCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             OUT NC_UPLOAD_REC_HEADER_STRUCT* pOutParam)
/// \brief This function...
/// \param MMC_CONNECT_HNDL hConn - [IN] Connection handle
/// \param NC_UPLOAD_REC_HEADER_STRUCT* pOutParam - [OUT] Pointer to Upload Recording header Data output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_UploadDataHeaderCmd(
        IN MMC_CONNECT_HNDL                 hConn,
        OUT NC_UPLOAD_REC_HEADER_STRUCT*    pOutParam) ;

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_EnhancedUploadDataHeaderCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             OUT NC_UPLOAD_REC_HEADER_STRUCT* pOutParam)
/// \brief This function...
/// \param MMC_CONNECT_HNDL hConn - [IN] Connection handle
/// \param NC_UPLOAD_REC_HEADER_STRUCT* pOutParam - [OUT] Pointer to Upload Recording header Data output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API int MMC_EnhancedUploadDataHeaderCmd(
        IN MMC_CONNECT_HNDL hConn,
        OUT NC_ENHANCED_UPLOAD_REC_HEADER_STRUCT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_UploadDataCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_UPLOAD_DATA_IN* pInParam,
///             OUT MMC_UPLOAD_DATA_OUT* pOutParam)
/// \brief This function send Upload Recorder Data Command to MMC.
/// \param  hConn - [IN] Connection handle
/// \param  pInParam - [IN] Pointer to Upload Recording Data input parameters
/// \param  pOutParam - [OUT] Pointer to Upload Recording Data output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_UploadDataCmd(
        IN MMC_CONNECT_HNDL         hConn,
        IN MMC_UPLOAD_DATA_IN*      pInParam,
        OUT MMC_UPLOAD_DATA_OUT*    pOutParam);


////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_EnhancedUploadDataCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_UPLOAD_DATA_IN* pInParam,
///             OUT MMC_UPLOAD_DATA_OUT* pOutParam)
/// \brief This function...
/// \param MMC_CONNECT_HNDL hConn - [IN] Connection handle
/// \param MMC_UPLOAD_DATA_IN* pInParam - [IN] Pointer to Upload Recording Data input parameters
/// \param MMC_UPLOAD_DATA_OUT* pOutParam - [OUT] Pointer to Upload Recording Data output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_EnhancedUploadDataCmd(
        IN MMC_CONNECT_HNDL hConn,
        IN MMC_UPLOAD_DATA_IN* pInParam,
        OUT MMC_ENHANCED_REC_UPLOAD_DATA_OUT* pOutParam);


////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_RecStatusCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_REC_STATUS_IN* pInParam,
///             OUT MMC_REC_STATUS_OUT* pOutParam)
/// \brief This function send Recording Status Request.
/// \param  hConn - [IN] Connection handle
/// \param  pInParam - [IN] Pointer to Recording Status input parameters
/// \param  pOutParam - [OUT] Pointer to Recording Status output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_RecStatusCmd(
        IN MMC_CONNECT_HNDL     hConn,
        IN MMC_REC_STATUS_IN*   pInParam,
        OUT MMC_REC_STATUS_OUT* pOutParam);


////////////////////////////////////////////////////////////////////////////////
/// \fn int  MMC_SetVerPathCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_SET_VERPATH_IN* pInParam,
///             OUT MMC_SET_VERPATH_OUT* pOutParam)
/// \brief This function set version path.
/// \param  hConn - [IN] Connection handle
/// \param  pInParam - [IN] Pointer to Set Version Path input parameters
/// \param  pOutParam - [OUT] Pointer to Set Version Path output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_SetVerPathCmd(
        IN MMC_CONNECT_HNDL         hConn,
        IN MMC_SET_VERPATH_IN*      pInParam,
        OUT MMC_SET_VERPATH_OUT*    pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int  MMC_GetVerPathCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_GET_VERPATH_IN* pInParam,
///             OUT MMC_GET_VERPATH_OUT* pOutParam)
/// \brief This function read Version Path.
/// \param  hConn - [IN] Connection handle
/// \param  pInParam - [IN] Pointer to Version Path input parameters
/// \param  pOutParam - [OUT] Pointer to Get Version Path output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_GetVerPathCmd(
        IN MMC_CONNECT_HNDL         hConn,
        IN MMC_GET_VERPATH_IN*      pInParam,
        OUT MMC_GET_VERPATH_OUT*    pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int   MMC_GlobalReadParameter(
///                 IN MMC_CONNECT_HNDL hConn,
///                 IN MMC_READPARAMETER_IN* pInParam,
///                 OUT MMC_READPARAMETER_OUT* pOutParam)
/// \brief This function read global parameter.
/// \param  hConn - [IN] Connection handle
/// \param  pInParam - [IN] Pointer to Read Parameter input parameters
/// \param  pOutParam - [OUT] Pointer to Read Parameter output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_GlobalReadParameter(
        IN MMC_CONNECT_HNDL         hConn,
        IN MMC_READPARAMETER_IN*    pInParam,
        OUT MMC_READPARAMETER_OUT*  pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int   MMC_GlobalReadBoolParameter(
///                 IN MMC_CONNECT_HNDL hConn,
///                 IN MMC_READBOOLPARAMETER_IN* pInParam,
///                 OUT MMC_READBOOLPARAMETER_OUT* pOutParam)
/// \brief This function read boolean global parameter.
/// \param  hConn - [IN] Connection handle
/// \param  pInParam - [IN] Pointer to Read Boolean Parameter input parameters
/// \param  pOutParam - [OUT] Pointer to Read Boolean Parameter output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_GlobalReadBoolParameter(
        IN MMC_CONNECT_HNDL             hConn,
        IN MMC_READBOOLPARAMETER_IN*    pInParam,
        OUT MMC_READBOOLPARAMETER_OUT*  pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int   MMC_GlobalWriteParameter(
///                 IN MMC_CONNECT_HNDL hConn,
///                 IN MMC_WRITEPARAMETER_IN* pInParam,
///                 OUT MMC_WRITEPARAMETER_OUT* pOutParam)
/// \brief This function write global parameter.
/// \param  hConn - [IN] Connection handle
/// \param  pInParam - [IN] Pointer to Write Parameter input parameters
/// \param  pOutParam - [OUT] Pointer to Write Parameter output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_GlobalWriteParameter(
        IN MMC_CONNECT_HNDL         hConn,
        IN MMC_WRITEPARAMETER_IN*   pInParam,
        OUT MMC_WRITEPARAMETER_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int   MMC_GlobalWriteBoolParameter(
///                 IN MMC_CONNECT_HNDL hConn,
///                 IN MMC_WRITEBOOLPARAMETER_IN* pInParam,
///                 OUT MMC_WRITEBOOLPARAMETER_OUT* pOutParam)
/// \brief This function write boolean global parameter.
/// \param  hConn - [IN] Connection handle
/// \param  pInParam - [IN] Pointer to Write Boolean Parameter input parameters
/// \param  pOutParam - [OUT] Pointer to Write Boolean Parameter output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_GlobalWriteBoolParameter(
        IN MMC_CONNECT_HNDL             hConn,
        IN MMC_WRITEBOOLPARAMETER_IN*   pInParam,
        OUT MMC_WRITEBOOLPARAMETER_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_DownloadFirmware (
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_DOWNLOADVERSION_IN * pInParam
///             OUT MMC_DOWNLOADVERSION_OUT* pOutParam)
/// \brief      This function performs download of new firmware version to the MMC.
/// \param  hConn - [IN] Connection handle
/// \param  pOutParam - [OUT] Immediate status error.
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_DownloadVersion (
        IN MMC_CONNECT_HNDL             hConn,
        IN  MMC_DOWNLOADVERSION_IN*     pInParam,
        OUT MMC_DOWNLOADVERSION_OUT*    pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_SaveParamCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_READAXISERROR_IN* pInParam,
///             OUT MMC_READAXISERROR_OUT* pOutParam)
/// \brief This function Save Global\Axes parameters to file.
/// \param  hConn - [IN] Connection handle
/// \param  pInParam - [IN] Pointer to Load Parameters input parameters
/// \param              ucType - this parameter define whoch parameters to save (0 = global, 1 = Axis)
/// \param  pOutParam - [OUT] Pointer to Save Parameters output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_SaveParamCmd(
        IN MMC_CONNECT_HNDL     hConn,
        IN MMC_SAVEPARAM_IN*    pInParam,
        OUT MMC_SAVEPARAM_OUT*  pOutParam);



////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_LoadParamCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_READAXISERROR_IN* pInParam,
///             OUT MMC_READAXISERROR_OUT* pOutParam)
/// \brief This function load Global\Axes parameters to file.
/// \param  hConn - [IN] Connection handle
/// \param  pInParam - [IN] Pointer to Load Parameters input parameters
/// \param              ucType - this parameter define whoch parameters to load (0 = global, 1 = Axis)
/// \param  pOutParam - [OUT] Pointer to Save Parameters output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_LoadParamCmd(
        IN MMC_CONNECT_HNDL     hConn,
        IN MMC_LOADPARAM_IN*    pInParam,
        OUT MMC_LOADPARAM_OUT*  pOutParam);



////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_CreatePersonalityParametersCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_CREATEPERSONALITYPARAMETERS_IN* pInParam,
///             OUT MMC_CREATEPERSONALITYPARAMETERS_OUT* pOutParam)
/// \brief This function create a personality file taht contain the parameters.
/// \param  hConn - [IN] Connection handle
/// \param  pInParam - [IN] Pointer to  input parameters
/// \param  pOutParam - [OUT] Pointer output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_CreatePersonalityParametersCmd(
        IN MMC_CONNECT_HNDL                         hConn,
        IN MMC_CREATEPERSONALITYPARAMETERS_IN*      pInParam,
        OUT MMC_CREATEPERSONALITYPARAMETERS_OUT*    pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_SETISTOLOADGLOBALPARAMS_OUT(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_SETISTOLOADGLOBALPARAMS_IN* pInParam,
///             OUT MMC_SETISTOLOADGLOBALPARAMS_OUT* pOutParam)
/// \brief This function set the flag that define whether to load global parameters or not.
/// \param  hConn - [IN] Connection handle
/// \param  pInParam - [IN] Pointer to  input parameters
/// \param  pOutParam - [OUT] Pointer output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_SetIsToLoadGlobalParamsCmd(
        IN MMC_CONNECT_HNDL                     hConn,
        IN MMC_SETISTOLOADGLOBALPARAMS_IN*      pInParam,
        OUT MMC_SETISTOLOADGLOBALPARAMS_OUT*    pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_SetDefaultParametersCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_SETDEFAULTPARAMETERS_IN* pInParam,
///             OUT MMC_SETDEFAULTPARAMETERS_OUT* pOutParam)
/// \brief This function set default manufacturer parameters to the Global\Axis related parameters.
/// \param  hConn - [IN] Connection handle
/// \param  pInParam - [IN] Pointer to  input parameters
/// \param  pOutParam - [OUT] Pointer output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_SetDefaultParametersCmd(
        IN MMC_CONNECT_HNDL                 hConn,
        IN MMC_AXIS_REF_HNDL                hAxisRef,
        IN MMC_SETDEFAULTPARAMETERS_IN*     pInParam,
        OUT MMC_SETDEFAULTPARAMETERS_OUT*   pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_SetDefaultParametersGlobalCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_SETDEFAULTPARAMETERS_IN* pInParam,
///             OUT MMC_SETDEFAULTPARAMETERS_OUT* pOutParam)
/// \brief This function set default manufacturer parameters to the Global\Axis related parameters.
/// \param  hConn - [IN] Connection handle
/// \param  pInParam - [IN] Pointer to  input parameters
/// \param  pOutParam - [OUT] Pointer output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_SetDefaultParametersGlobalCmd(
        IN MMC_CONNECT_HNDL                     hConn,
        IN MMC_SETDEFAULTPARAMETERSGLOBAL_IN*   pInParam,
        OUT MMC_SETDEFAULTPARAMETERSGLOBAL_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int Calc_md5sum (
///             N char* fName,
///             IN MMC_DOWNLOADVERSION_IN * pInParam
/// \brief      This function performs an md5sum calculation.on a file.
/// \param  char* fName - [IN] File name
/// \return return - 0 if failed
///                  otherwise returns the md5 first 4 bytes.
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API  ELMO_ULINT32 Calc_md5sum(ELMO_PINT8 fName) ;

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ResImportFileCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_RESIMPORTFILE_IN* pInParam,
///             OUT MMC_RESIMPORTFILE_OUT* pOutParam)
/// \brief This function do import resources filefrom host to target by TFTP.
/// \param  hConn - [IN] Connection handle
/// \param  pInParam - [IN] Import Resource File  input data structure
/// \param  pOutParam - [OUT] Import Resource File output data structure
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ResImportFileCmd(
        IN MMC_CONNECT_HNDL         hConn,
        IN MMC_RESIMPORTFILE_IN*    pInParam,
        OUT MMC_RESIMPORTFILE_OUT*  pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ResExportFileCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_RESEXPORTFILE_IN* pInParam,
///             OUT MMC_RESEXPORTFILE_OUT* pOutParam)
/// \brief This function do export resources file from target to host by TFTP.
/// \param  hConn - [IN] Connection handle
/// \param  pInParam - [IN] Export Resource File  input data structure
/// \param  pOutParam - [OUT] Export Resource File output data structure
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ResExportFileCmd(
        IN MMC_CONNECT_HNDL         hConn,
        IN MMC_RESEXPORTFILE_IN*    pInParam,
        OUT MMC_RESEXPORTFILE_OUT*  pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ResetMultiAxisControl (
///
///
/// \brief      This function closes MultiAxisControl by closing connection,shared memory, and nc_drv
/// \return return - 0 if failed
///                  error_id in case of error
///
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ResetMultiAxisControl (
        IN MMC_CONNECT_HNDL     hConn,
        IN  MMC_EXIT_APP_IN*    pInParam,
        OUT MMC_EXIT_APP_OUT*   pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ReadDownloadVersionStatus (
///
///
/// \brief      This function returns the status of the last downloaded version
/// \return return - 0 if failed
///                  error_id in case of error
///
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ReadDownloadVersionStatus (
        IN MMC_CONNECT_HNDL                 hConn,
        IN  MMC_READDOWNLOADVERSTATUS_IN*   pInParam,
        OUT MMC_READDOWNLOADVERSTATUS_OUT*  pOutParam);


////////////////////////////////////////////////////////////////////////////////
/// \fn int  MMC_ChangeToPreOPMode(
///             IN MMC_CONNECT_HNDL hConn,
///             OUT MMC_SET_GMAS_PREOP_OUT* pOutParam)
/// \brief  Changes the GMAS to Pre Operation mode.
/// \param  hConn - [IN] Connection handle
/// \param  pOutParam - [OUT] Pointer to MMC_SET_GMAS_PREOP_OUT structure.
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ChangeToPreOPMode(
        IN MMC_CONNECT_HNDL         hConn,
        OUT MMC_SET_GMAS_PREOP_OUT* pOutParam) ;

////////////////////////////////////////////////////////////////////////////////
/// \fn int  MMC_ChangeToOperationMode(
///             IN MMC_CONNECT_HNDL hConn,
///             OUT MMC_SET_GMAS_OP_OUT* pOutParam)
/// \brief  Changes the GMAS to Operation mode.
/// \param  hConn - [IN] Connection handle
/// \param  pOutParam - [OUT] Pointer to MMC_SET_GMAS_OP_OUT structure,
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ChangeToOperationMode(
        IN MMC_CONNECT_HNDL         hConn,
        OUT MMC_SET_GMAS_OP_OUT*    pOutParam) ;

////////////////////////////////////////////////////////////////////////////////
/// \fn int  GetGMASOperationMode(
///             IN MMC_CONNECT_HNDL hConn,
///             OUT MMC_GET_GMASOP_MODE_OUT* pOutParam)
/// \brief  Returns the current GMAS operation mode.
/// \param  hConn - [IN] Connection handle
/// \param  pOutParam - [OUT] Pointer to MMC_SET_GMAS_OP_OUT structure.
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_GetGMASOperationMode(
        IN MMC_CONNECT_HNDL             hConn,
        OUT MMC_GET_GMASOP_MODE_OUT*    pOutParam) ;

////////////////////////////////////////////////////////////////////////////////
/// \fn int  MMC_CreateSYNCTimer(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_SYNC_TIMER_CB_FUNC func,
///             IN unsigned short usSYNCTimerTime)
/// \brief
/// \param  hConn - [IN] Connection handle
/// \param  func  - [IN] Callback function pointer
/// \param  usSYNCTimerTime - [IN] SYNC Timer time
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
// HH: Define in OS_Platform*.h
// MMC_LIB_API ELMO_INT32 MMC_CreateSYNCTimer(IN MMC_CONNECT_HNDL hConn,
//        IN MMC_SYNC_TIMER_CB_FUNC   func,
//        IN ELMO_UINT16              usSYNCTimerTime);

////////////////////////////////////////////////////////////////////////////////
/// \fn int  MMC_DestroySYNCTimer(
///             IN MMC_CONNECT_HNDL hConn)
/// \brief
/// \param  hConn - [IN] Connection handle
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
// HH: Define in OS_Platform*.h.
// MMC_LIB_API ELMO_INT32 MMC_DestroySYNCTimer(IN MMC_CONNECT_HNDL hConn);

////////////////////////////////////////////////////////////////////////////////
/// \fn int  MMC_SetRTUserCallback(
///             IN MMC_CONNECT_HNDL hConn,
///             IN unsigned short usSetUserRtCallBack)
/// \brief
/// \param  hConn - [IN] Connection handle
/// \param  usSetRTUserCallback - [IN] set the user callback (setSyncTimer) to priority 99 and  set affinity
/// \ Supported only in PLATINUM
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32  MMC_SetRTUserCallback(
                        IN MMC_CONNECT_HNDL hConn,
                        IN ELMO_UINT8 usSetRTUserCallback);


////////////////////////////////////////////////////////////////////////////////
/// \fn int  MMC_GetErrorCodeDescriptionByID(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_GETERRORCODEDESCRIPTIONBYID_IN* pInParam,
///             OUT MMC_GETERRORCODEDESCRIPTIONBYID_OUT* pOutParam)
/// \brief  This function receives error\warning code and return the description
///         and resolution from the Personality file.
/// \param  hConn - [IN] Connection handle
/// \param  pInParam - [IN] the input receives the error\warning number
/// \param  pOutParam - [OUT] the output returns two strings, status and error id.
///                             1. Description
///                             2. Resolution
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_GetErrorCodeDescriptionByID(
        IN MMC_CONNECT_HNDL                         hConn,
        IN MMC_GETERRORCODEDESCRIPTIONBYID_IN*      pInParam,
        OUT MMC_GETERRORCODEDESCRIPTIONBYID_OUT*    pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_WriteGroupOfParameters(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_WRITEGROUPOFPARAMETERS_IN* pInParam,
///             OUT MMC_WRITEGROUPOFPARAMETERS_OUT* pOutParam)
/// \brief This function write a group of parameters to G-MAS memory
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to WriteGroupOfParameters input parameters
/// \param pOutParam - [OUT] Pointer to WriteGroupOfParameters output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_WriteGroupOfParameters(
        IN MMC_CONNECT_HNDL                     hConn,
        IN MMC_AXIS_REF_HNDL                    hAxisRef,
        IN MMC_WRITEGROUPOFPARAMETERS_IN*       pInParam,
        OUT MMC_WRITEGROUPOFPARAMETERS_OUT*     pOutParam);

MMC_LIB_API ELMO_INT32 MMC_WriteGroupOfParametersEX(
        IN MMC_CONNECT_HNDL                     hConn,
        IN MMC_AXIS_REF_HNDL                    hAxisRef,
        IN MMC_WRITEGROUPOFPARAMETERSEX_IN*     pInParam,
        OUT MMC_WRITEGROUPOFPARAMETERSEX_OUT*   pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ReadGroupOfParameters(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_READGROUPOFPARAMETERS_IN*    pInParam,
///             OUT MMC_READGROUPOFPARAMETERS_OUT*  pOutParam)
/// \brief This function retreive a group of parameters to the user
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to ReadGroupOfParameters input parameters
/// \param pOutParam - [OUT] Pointer to ReadGroupOfParameters output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ReadGroupOfParameters(
        IN MMC_CONNECT_HNDL                 hConn,
        IN MMC_READGROUPOFPARAMETERS_IN*    pInParam,
        OUT MMC_READGROUPOFPARAMETERS_OUT*  pOutParam);


////////////////////////////////////////////////////////////////////////////////
/// \fn int  MMC_GetStatusRegisterCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_GETSTATUSREGISTER_IN* pInParam,
///             OUT MMC_GETSTATUSREGISTER_OUT* pOutParam)
/// \brief  This function returns the motion status word of axis\group.
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] axis\group reference
/// \param  pInParam - [IN] dummy
/// \param  pOutParam - [OUT] the output structure contain the following parameters
///                             1. usStatus - status of the axis\group (PLCOpen)
///                             2. usError - error id.
///                             3. uiStatusRegister - motion status word
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_GetStatusRegisterCmd(
        IN MMC_CONNECT_HNDL             hConn,
        IN MMC_AXIS_REF_HNDL            hAxisRef,
        IN MMC_GETSTATUSREGISTER_IN*    pInParam,
        OUT MMC_GETSTATUSREGISTER_OUT*  pOutParam);

MMC_LIB_API ELMO_INT32 MMC_DownloadFoE(
        IN MMC_CONNECT_HNDL         hConn,
        IN MMC_DOWNLOADFOE_IN*      pInParam,
        OUT MMC_DOWNLOADFOE_OUT*    pOutParam);

MMC_LIB_API ELMO_INT32 MMC_GetFoEStatus(
        IN MMC_CONNECT_HNDL       hConn,
        OUT MMC_GETFOESTATUS_OUT* pOutParam);

MMC_LIB_API ELMO_INT32 MMC_GetEthercatCommStatistics(
        IN MMC_CONNECT_HNDL                 hConn,
        IN MMC_GETCOMMSTATISTICSEX_IN*      pInParam,
        OUT MMC_GETCOMMSTATISTICSEX_OUT*    pOutParam) ;

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ConfigBulkReadCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_CONFIGBULKREAD_IN* pInParam,
///             OUT MMC_CONFIGBULKREAD_OUT* pOutParam)
/// \brief This function configures bulk read for regular variables.
/// \param  hConn - [IN] Connection handle
/// \param  pInParam - [IN] Pointer to MMC_ConfigBulkReadCmd input parameters
/// \param  pOutParam - [OUT] Pointer to MMC_ConfigBulkReadCmd output parameters
/// \param             usStatus         - [OUT] This parameter is the status of the axis
/// \param             usErrorID        - [OUT] This parameter is errorid, equal zero when no-error,
///                                             positive value when warning, negative value when error
/// \return return - 0 if success, negative value in case of error, positive value in case of warning
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ConfigBulkReadCmd(IN MMC_CONNECT_HNDL    hConn,
                                      IN MMC_CONFIGBULKREAD_IN*     pInParam,
                                      OUT MMC_CONFIGBULKREAD_OUT*   pOutParam);
                                      
////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ConfigBulkReadCmdPI(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_CONFIGBULKREADPI_IN* pInParam,
///             OUT MMC_CONFIGBULKREADPI_OUT* pOutParam)
/// \brief This function configures bulk read for PI variables.
/// \param  hConn - [IN] Connection handle
/// \param  pInParam - [IN] Pointer to MMC_ConfigBulkReadCmdPI input parameters
/// \param  pOutParam - [OUT] Pointer to MMC_ConfigBulkReadCmdPI output parameters
/// \param             usStatus         - [OUT] This parameter is the status of the axis
/// \param             usErrorID        - [OUT] This parameter is errorid, equal zero when no-error,
///                                             positive value when warning, negative value when error
/// \return return - 0 if success, negative value in case of error, positive value in case of warning
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ConfigBulkReadCmdPI(IN MMC_CONNECT_HNDL  hConn,
                                      IN MMC_CONFIGBULKREADPI_IN*   pInParam,
                                      OUT MMC_CONFIGBULKREADPI_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_PerformBulkReadCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_PERFORMBULKREAD_IN* pInParam,
///             OUT MMC_PERFORMBULKREAD_OUT* pOutParam)
/// \brief This function performs bulk read for regular variables.
/// \param  hConn - [IN] Connection handle
/// \param  pInParam - [IN] Pointer to MMC_PerformBulkReadCmd input parameters
/// \param  pOutParam - [OUT] Pointer to MMC_PerformBulkReadCmd output parameters
/// \param             usStatus         - [OUT] This parameter is the status of the axis
/// \param             usErrorID        - [OUT] This parameter is errorid, equal zero when no-error,
///                                             positive value when warning, negative value when error
/// \return return - 0 if success, negative value in case of error, positive value in case of warning
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_PerformBulkReadCmd(IN MMC_CONNECT_HNDL hConn,
                                       IN MMC_PERFORMBULKREAD_IN* pInParam,
                                       OUT MMC_PERFORMBULKREAD_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_PerformBulkReadCmdPI(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_PERFORMBULKREADPI_IN* pInParam,
///             OUT MMC_PERFORMBULKREADPI_OUT* pOutParam)
/// \brief This function performs bulk read for regular variables.
/// \param  hConn - [IN] Connection handle
/// \param  pInParam - [IN] Pointer to MMC_PerformBulkReadCmdPI input parameters
/// \param  pOutParam - [OUT] Pointer to MMC_PerformBulkReadCmdPI output parameters
/// \param             usStatus         - [OUT] This parameter is the status of the axis
/// \param             usErrorID        - [OUT] This parameter is errorid, equal zero when no-error,
///                                             positive value when warning, negative value when error
/// \return return - 0 if success, negative value in case of error, positive value in case of warning
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_PerformBulkReadCmdPI(IN MMC_CONNECT_HNDL hConn,
                                       IN MMC_PERFORMBULKREADPI_IN* pInParam,
                                       OUT MMC_PERFORMBULKREADPI_OUT* pOutParam);

MMC_LIB_API ELMO_INT32 MMC_ToggleConsoleOutputCmd(
        IN MMC_CONNECT_HNDL                 hConn,
        IN MMC_TOGGLECONSOLEOUTPUT_IN*      pInParam,
        OUT MMC_TOGGLECONSOLEOUTPUT_OUT*    pOutParam);

MMC_LIB_API ELMO_INT32 MMC_GetActiveAxesNum(
        IN MMC_CONNECT_HNDL             hConn,
        IN MMC_GETACTIVEAXESNUM_IN*     pInParam,
        OUT MMC_GETACTIVEAXESNUM_OUT*   pOutParam) ;

MMC_LIB_API ELMO_INT32 MMC_GetActiveVectorsNum(
        IN MMC_CONNECT_HNDL             hConn,
        IN MMC_GETACTIVEVECTORSNUM_IN*  pInParam,
        OUT MMC_GETACTIVEVECTORSNUM_OUT* pOutParam) ;


MMC_LIB_API ELMO_INT32 MMC_ClearNodeFbListCmd(
        IN MMC_CONNECT_HNDL         hConn,
        IN MMC_CLEARFBLIST_IN*      pInParam,
        OUT MMC_CLEARFBLIST_OUT*    pOutParam);

MMC_LIB_API ELMO_INT32 MMC_GetCyclesCounterCmd(
        IN MMC_CONNECT_HNDL             hConn,
        IN MMC_GETCYCLESCOUNTER_IN*     pInParam,
        OUT MMC_GETCYCLESCOUNTER_OUT*   pOutParam);

MMC_LIB_API ELMO_INT32 MMC_DwellCmd(IN MMC_CONNECT_HNDL hConn,
        IN MMC_AXIS_REF_HNDL    hAxisRef,
        IN MMC_DWELL_IN*        pInParam,
        OUT MMC_DWELL_OUT*      pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int  MMC_WaitUntilConditionFB(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_WAITUNTILCONDITIONFB_IN* pInParam,
///             OUT MMC_WAITUNTILCONDITIONFB_OUT* pOutParam)
/// \brief  This function insert a condition FB to the nodes queue axis\group.
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] axis\group reference
/// \param  pInParam - [IN] source and reference value, condition type
/// \param  pOutParam - [OUT] return the status and FB handler
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_WaitUntilConditionFB(IN MMC_CONNECT_HNDL hConn,
        IN MMC_AXIS_REF_HNDL            hAxisRef,
        IN MMC_WAITUNTILCONDITIONFB_IN* pInParam,
        OUT MMC_WAITUNTILCONDITIONFB_OUT* pOutParam);

MMC_LIB_API ELMO_INT32 MMC_WaitUntilConditionFBEX(IN MMC_CONNECT_HNDL hConn,
                            IN MMC_AXIS_REF_HNDL                hAxisRef,
                            IN MMC_WAITUNTILCONDITIONFBEX_IN*   pInParam,
                            OUT MMC_WAITUNTILCONDITIONFBEX_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_WriteMemoryRange(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_WRITEMEMORYRANGE_IN* pInParam,
///             OUT MMC_WRITEMEMORYRANGE_OUT* pOutParam)
/// \brief This function write a memory range of an ethercat slave
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to WriteMemoryRange input parameters
/// \param pOutParam - [OUT] Pointer to WriteMemoryRange output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_WriteMemoryRange(
        IN MMC_CONNECT_HNDL         hConn,
        IN MMC_AXIS_REF_HNDL        hAxisRef,
        IN MMC_WRITEMEMORYRANGE_IN* pInParam,
        OUT MMC_WRITEMEMORYRANGE_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ReadMemoryRange(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_READMEMORYRANGE_IN* pInParam,
///             OUT MMC_READMEMORYRANGE_OUT* pOutParam)
/// \brief This function reads a memory range from an ethercat slave
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to ReadMemoryRange input parameters
/// \param pOutParam - [OUT] Pointer to ReadMemoryRange output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ReadMemoryRange(
        IN MMC_CONNECT_HNDL         hConn,
        IN MMC_AXIS_REF_HNDL        hAxisRef,
        IN MMC_READMEMORYRANGE_IN*  pInParam,
        OUT MMC_READMEMORYRANGE_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ReadPIVarBOOL(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_READPIVARBOOL_IN* pInParam,
///             OUT MMC_READPIVARBOOL_OUT* pOutParam)
/// \brief This function reads a PI input\output according to its index
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to ReadPIVar input parameters
/// \param pOutParam - [OUT] Pointer to ReadPIVarBOOL output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ReadPIVarBOOL(
        IN MMC_CONNECT_HNDL         hConn,
        IN MMC_AXIS_REF_HNDL        hAxisRef,
        IN MMC_READPIVARBOOL_IN*    pInParam,
        OUT MMC_READPIVARBOOL_OUT*  pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ReadPIVarChar(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_READPIVARCHAR_IN* pInParam,
///             OUT MMC_READPIVARCHAR_OUT* pOutParam)
/// \brief This function reads a PI input\output according to its index
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to ReadPIVar input parameters
/// \param pOutParam - [OUT] Pointer to ReadPIVarBOOL output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ReadPIVarChar(
        IN MMC_CONNECT_HNDL         hConn,
        IN MMC_AXIS_REF_HNDL        hAxisRef,
        IN MMC_READPIVARCHAR_IN*    pInParam,
        OUT MMC_READPIVARCHAR_OUT*  pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ReadPIVarUChar(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_READPIVARUCHAR_IN* pInParam,
///             OUT MMC_READPIVARUCHAR_OUT* pOutParam)
/// \brief This function reads a PI input\output according to its index
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to ReadPIVar input parameters
/// \param pOutParam - [OUT] Pointer to ReadPIVarBOOL output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ReadPIVarUChar(
        IN MMC_CONNECT_HNDL         hConn,
        IN MMC_AXIS_REF_HNDL        hAxisRef,
        IN MMC_READPIVARUCHAR_IN*   pInParam,
        OUT MMC_READPIVARUCHAR_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ReadPIVarShort(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_READPIVARSHORT_IN* pInParam,
///             OUT MMC_READPIVARSHORT_OUT* pOutParam)
/// \brief This function reads a PI input\output according to its index
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to ReadPIVar input parameters
/// \param pOutParam - [OUT] Pointer to ReadPIVarBOOL output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ReadPIVarShort(
        IN MMC_CONNECT_HNDL         hConn,
        IN MMC_AXIS_REF_HNDL        hAxisRef,
        IN MMC_READPIVARSHORT_IN*   pInParam,
        OUT MMC_READPIVARSHORT_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ReadPIVarUShort(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_READPIVARUSHORT_IN* pInParam,
///             OUT MMC_READPIVARUSHORT_OUT* pOutParam)
/// \brief This function reads a PI input\output according to its index
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to ReadPIVar input parameters
/// \param pOutParam - [OUT] Pointer to ReadPIVarUnsignedShort output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ReadPIVarUShort(
        IN MMC_CONNECT_HNDL         hConn,
        IN MMC_AXIS_REF_HNDL        hAxisRef,
        IN MMC_READPIVARUSHORT_IN*  pInParam,
        OUT MMC_READPIVARUSHORT_OUT*    pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ReadPIVarInt(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_READPIVARINT_IN* pInParam,
///             OUT MMC_READPIVARINT_OUT* pOutParam)
/// \brief This function reads a PI input\output according to its index
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to ReadPIVar input parameters
/// \param pOutParam - [OUT] Pointer to ReadPIVarSignedInt output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ReadPIVarInt(
        IN MMC_CONNECT_HNDL         hConn,
        IN MMC_AXIS_REF_HNDL        hAxisRef,
        IN MMC_READPIVARINT_IN*     pInParam,
        OUT MMC_READPIVARINT_OUT*   pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ReadPIVarUInt(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_READPIVARUINT_IN* pInParam,
///             OUT MMC_READPIVARUINT_OUT* pOutParam)
/// \brief This function reads a PI input\output according to its index
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to MMC_ReadPIVarUInt input parameters
/// \param pOutParam - [OUT] Pointer to MMC_ReadPIVarUInt output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ReadPIVarUInt(
        IN MMC_CONNECT_HNDL         hConn,
        IN MMC_AXIS_REF_HNDL        hAxisRef,
        IN MMC_READPIVARUINT_IN*    pInParam,
        OUT MMC_READPIVARUINT_OUT*  pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ReadPIVarFloat(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_READPIVARFLOAT_IN* pInParam,
///             OUT MMC_READPIVARFLOAT_OUT* pOutParam)
/// \brief This function reads a PI input\output according to its index
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to ReadPIVar input parameters
/// \param pOutParam - [OUT] Pointer to ReadPIVarFloat output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ReadPIVarFloat(
        IN MMC_CONNECT_HNDL         hConn,
        IN MMC_AXIS_REF_HNDL        hAxisRef,
        IN MMC_READPIVARFLOAT_IN*   pInParam,
        OUT MMC_READPIVARFLOAT_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ReadPIVarRaw(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_READPIVARRAW_IN* pInParam,
///             OUT MMC_READPIVARRAW_OUT* pOutParam)
/// \brief This function reads a PI input\output according to its index - 8Multiple and bitwise types
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to ReadPIVar input parameters
/// \param pOutParam - [OUT] Pointer to ReadPIVarRaw output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ReadPIVarRaw(
        IN MMC_CONNECT_HNDL         hConn,
        IN MMC_AXIS_REF_HNDL        hAxisRef,
        IN MMC_READPIVARRAW_IN*     pInParam,
        OUT MMC_READPIVARRAW_OUT*   pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ReadPIVarLongLong(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_READPIVARLONGLONG_IN* pInParam,
///             OUT MMC_READPIVARLONGLONG_OUT* pOutParam)
/// \brief This function reads a PI input\output according to its index
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to ReadPIVar input parameters
/// \param pOutParam - [OUT] Pointer to ReadPIVarSignedLongLong output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ReadPIVarLongLong(
        IN MMC_CONNECT_HNDL             hConn,
        IN MMC_AXIS_REF_HNDL            hAxisRef,
        IN MMC_READPIVARLONGLONG_IN*    pInParam,
        OUT MMC_READPIVARLONGLONG_OUT*  pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ReadPIVarULongLong(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_READPIVARULONGLONG_IN* pInParam,
///             OUT MMC_READPIVARULONGLONG_OUT* pOutParam)
/// \brief This function reads a PI input\output according to its index
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to ReadPIVar input parameters
/// \param pOutParam - [OUT] Pointer to ReadPIVarUnsignedLongLong output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ReadPIVarULongLong(
        IN MMC_CONNECT_HNDL             hConn,
        IN MMC_AXIS_REF_HNDL            hAxisRef,
        IN MMC_READPIVARULONGLONG_IN*   pInParam,
        OUT MMC_READPIVARULONGLONG_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ReadPIVarDouble(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_READPIVARDOUBLE_IN* pInParam,
///             OUT MMC_READPIVARDOUBLE_OUT* pOutParam)
/// \brief This function reads a PI input\output according to its index
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to ReadPIVarDouble input parameters
/// \param pOutParam - [OUT] Pointer to ReadPIVarDouble output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ReadPIVarDouble(
        IN MMC_CONNECT_HNDL         hConn,
        IN MMC_AXIS_REF_HNDL        hAxisRef,
        IN MMC_READPIVARDOUBLE_IN*  pInParam,
        OUT MMC_READPIVARDOUBLE_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ReadLargePIVarRaw(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_READLARGEPIVARRAW_IN* pInParam,
///             OUT MMC_READLARGEPIVARRAW_OUT* pOutParam)
/// \brief This function reads a PI input\output according to its index
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to ReadPIVar input parameters
/// \param pOutParam - [OUT] Pointer to ReadLargePIVarRaw output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ReadLargePIVarRaw(
        IN MMC_CONNECT_HNDL             hConn,
        IN MMC_AXIS_REF_HNDL            hAxisRef,
        IN MMC_READLARGEPIVARRAW_IN*    pInParam,
        OUT MMC_READLARGEPIVARRAW_OUT*  pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_WritePIVarBool(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_WRITEPIVARBOOL_IN* pInParam,
///             OUT MMC_WRITEPIVARBOOL_OUT* pOutParam)
/// \brief This function writes a PI output according to its index
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to MMC_WritePIVarBool input parameters
/// \param pOutParam - [OUT] Pointer to MMC_WritePIVarBool output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_WritePIVarBool(
        IN MMC_CONNECT_HNDL         hConn,
        IN MMC_AXIS_REF_HNDL        hAxisRef,
        IN MMC_WRITEPIVARBOOL_IN*   pInParam,
        OUT MMC_WRITEPIVARBOOL_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_WritePIVarChar(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_WRITEPIVARCHAR_IN* pInParam,
///             OUT MMC_WRITEPIVARCHAR_OUT* pOutParam)
/// \brief This function writes a PI output according to its index
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to MMC_WritePIVarChar input parameters
/// \param pOutParam - [OUT] Pointer to MMC_WritePIVarChar output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_WritePIVarChar(
        IN MMC_CONNECT_HNDL         hConn,
        IN MMC_AXIS_REF_HNDL        hAxisRef,
        IN MMC_WRITEPIVARCHAR_IN*   pInParam,
        OUT MMC_WRITEPIVARCHAR_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_WritePIVarUChar(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_WRITEPIVARUSCHAR_IN* pInParam,
///             OUT MMC_WRITEPIVARUSCHAR_OUT* pOutParam)
/// \brief This function writes a PI output according to its index
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to MMC_WritePIVarUChar input parameters
/// \param pOutParam - [OUT] Pointer to MMC_WritePIVarUChar output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_WritePIVarUChar(
        IN MMC_CONNECT_HNDL         hConn,
        IN MMC_AXIS_REF_HNDL        hAxisRef,
        IN MMC_WRITEPIVARUCHAR_IN*  pInParam,
        OUT MMC_WRITEPIVARUCHAR_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_WritePIVarUShort(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_WRITEPIVARUSSHORT_IN* pInParam,
///             OUT MMC_WRITEPIVARUSSHORT_OUT* pOutParam)
/// \brief This function writes a PI output according to its index
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to MMC_WritePIVarUShort input parameters
/// \param pOutParam - [OUT] Pointer to MMC_WritePIVarUShort output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_WritePIVarUShort(
        IN MMC_CONNECT_HNDL             hConn,
        IN MMC_AXIS_REF_HNDL            hAxisRef,
        IN MMC_WRITEPIVARUSHORT_IN*     pInParam,
        OUT MMC_WRITEPIVARUSHORT_OUT*   pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_WritePIVarShort(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_WRITEPIVARSHORT_IN* pInParam,
///             OUT MMC_WRITEPIVARSHORT_OUT* pOutParam)
/// \brief This function writes a PI output according to its index
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to MMC_WritePIVarShort input parameters
/// \param pOutParam - [OUT] Pointer to MMC_WritePIVarShort output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_WritePIVarShort(
        IN MMC_CONNECT_HNDL         hConn,
        IN MMC_AXIS_REF_HNDL        hAxisRef,
        IN MMC_WRITEPIVARSHORT_IN*  pInParam,
        OUT MMC_WRITEPIVARSHORT_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_WritePIVarUInt(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_WRITEPIVARUINT_IN* pInParam,
///             OUT MMC_WRITEPIVARUINT_OUT* pOutParam)
/// \brief This function writes a PI output according to its index
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to WritePIVarUnsignedInt input parameters
/// \param pOutParam - [OUT] Pointer to WritePIVar output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_WritePIVarUInt(
        IN MMC_CONNECT_HNDL         hConn,
        IN MMC_AXIS_REF_HNDL        hAxisRef,
        IN MMC_WRITEPIVARUINT_IN*   pInParam,
        OUT MMC_WRITEPIVARUINT_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_WritePIVarInt(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_WRITEPIVARINT_IN* pInParam,
///             OUT MMC_WRITEPIVARINT_OUT* pOutParam)
/// \brief This function writes a PI output according to its index
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to MMC_WritePIVarInt input parameters
/// \param pOutParam - [OUT] Pointer to MMC_WritePIVarInt output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_WritePIVarInt(
        IN MMC_CONNECT_HNDL         hConn,
        IN MMC_AXIS_REF_HNDL        hAxisRef,
        IN MMC_WRITEPIVARINT_IN*    pInParam,
        OUT MMC_WRITEPIVARINT_OUT*  pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_WritePIVarFloat(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_WRITEPIVARFLOAT_IN* pInParam,
///             OUT MMC_WRITEPIVARFLOAT_OUT* pOutParam)
/// \brief This function writes a PI output according to its index
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to MMC_WritePIVarFloat input parameters
/// \param pOutParam - [OUT] Pointer to MMC_WritePIVarFloat output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_WritePIVarFloat(
        IN MMC_CONNECT_HNDL         hConn,
        IN MMC_AXIS_REF_HNDL        hAxisRef,
        IN MMC_WRITEPIVARFLOAT_IN*  pInParam,
        OUT MMC_WRITEPIVARFLOAT_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_WritePIVarRaw(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_WRITEPIVARRAW_IN* pInParam,
///             OUT MMC_WRITEPIVARRAW_OUT* pOutParam)
/// \brief This function writes a PI output according to its index
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to MMC_WritePIVarRaw input parameters
/// \param pOutParam - [OUT] Pointer to MMC_WritePIVarRaw output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_WritePIVarRaw(
        IN MMC_CONNECT_HNDL         hConn,
        IN MMC_AXIS_REF_HNDL        hAxisRef,
        IN MMC_WRITEPIVARRAW_IN*    pInParam,
        OUT MMC_WRITEPIVARRAW_OUT*  pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_WritePIVarULongLong(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_WRITEPIVARUSLONGLONG_IN* pInParam,
///             OUT MMC_WRITEPIVARUSLONGLONG_OUT* pOutParam)
/// \brief This function writes a PI output according to its index
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to MMC_WritePIVarULongLong input parameters
/// \param pOutParam - [OUT] Pointer to MMC_WritePIVarULongLong output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_WritePIVarULongLong(
        IN MMC_CONNECT_HNDL             hConn,
        IN MMC_AXIS_REF_HNDL            hAxisRef,
        IN MMC_WRITEPIVARULONGLONG_IN*  pInParam,
        OUT MMC_WRITEPIVARULONGLONG_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_WritePIVarLongLong(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_WRITEPIVARLONGLONG_IN* pInParam,
///             OUT MMC_WRITEPIVARLONGLONG_OUT* pOutParam)
/// \brief This function writes a PI output according to its index
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to MMC_WritePIVarLongLong input parameters
/// \param pOutParam - [OUT] Pointer to MMC_WritePIVarLongLong output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_WritePIVarLongLong(
        IN MMC_CONNECT_HNDL             hConn,
        IN MMC_AXIS_REF_HNDL            hAxisRef,
        IN MMC_WRITEPIVARLONGLONG_IN*   pInParam,
        OUT MMC_WRITEPIVARLONGLONG_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_WritePIVarDouble(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_WRITEPIVARDOUBLE_IN* pInParam,
///             OUT MMC_WRITEPIVARDOUBLE_OUT* pOutParam)
/// \brief This function writes a PI output according to its index
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to MMC_WritePIVarDouble input parameters
/// \param pOutParam - [OUT] Pointer to MMC_WritePIVarDouble output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_WritePIVarDouble(
        IN MMC_CONNECT_HNDL             hConn,
        IN MMC_AXIS_REF_HNDL            hAxisRef,
        IN MMC_WRITEPIVARDOUBLE_IN*     pInParam,
        OUT MMC_WRITEPIVARDOUBLE_OUT*   pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_WriteLargePIVarRaw(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_WRITELARGEPIVARRAW_IN* pInParam,
///             OUT MMC_WRITELARGEPIVARRAW_OUT* pOutParam)
/// \brief This function writes a PI output according to its index
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to MMC_WriteLargePIVarRaw input parameters
/// \param pOutParam - [OUT] Pointer to MMC_WriteLargePIVarRaw output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_WriteLargePIVarRaw(
        IN MMC_CONNECT_HNDL             hConn,
        IN MMC_AXIS_REF_HNDL            hAxisRef,
        IN MMC_WRITELARGEPIVARRAW_IN*   pInParam,
        OUT MMC_WRITELARGEPIVARRAW_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_GetPIVarInfo(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_GETPIVARINFO_IN* pInParam,
///             OUT MMC_GETPIVARINFO_OUT* pOutParam)
/// \brief This function returns info about the desired PI variable
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to GetPIVarInfo input parameters
/// \param pOutParam - [OUT] Pointer to GetPIVarInfo output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_GetPIVarInfo(
        IN MMC_CONNECT_HNDL         hConn,
        IN MMC_AXIS_REF_HNDL        hAxisRef,
        IN MMC_GETPIVARINFO_IN*     pInParam,
        OUT MMC_GETPIVARINFO_OUT*   pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_GetPIVarInfoByAlias(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_GETPIINFOBYALIAS_IN* pInParam,
///             OUT MMC_GETPIINFOBYALIAS_OUT* pOutParam)
/// \brief This function returns a PI variable entry in the PI table by its alias
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to GetPIVarInfoByAlias input parameters
/// \param pOutParam - [OUT] Pointer to GetPIVarInfoByAlias output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_GetPIVarInfoByAlias(
        IN MMC_CONNECT_HNDL                 hConn,
        IN MMC_AXIS_REF_HNDL                hAxisRef,
        IN MMC_GETPIVARINFOBYALIAS_IN*      pInParam,
        OUT MMC_GETPIVARINFOBYALIAS_OUT*    pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_GetPIVarsRangeInfo(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_GETPIVARSRANGEINFO_IN* pInParam,
///             OUT MMC_GETPIVARSRANGEINFO_OUT* pOutParam)
/// \brief This function returns a range of PI variables entries in the PI table
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to GetPIVarsRangeInfo input parameters
/// \param pOutParam - [OUT] Pointer to GetPIVarsRangeInfo output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_GetPIVarsRangeInfo(
        IN MMC_CONNECT_HNDL             hConn,
        IN MMC_AXIS_REF_HNDL            hAxisRef,
        IN MMC_GETPIVARSRANGEINFO_IN*   pInParam,
        OUT MMC_GETPIVARSRANGEINFO_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_SetDefaultResources(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_SETDEFAULTRESOURCES_IN* pInParam,
///             OUT MMC_SETDEFAULTRESOURCES_OUT* pOutParam)
/// \brief This function restores GMAS resource file to factory default according to 
///        the desired communication type -  eCOMM_TYPE_ETHERCAT or eCOMM_TYPE_CAN
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to SetDefaultResources input parameters
/// \param pOutParam - [OUT] Pointer to SetDefaultResources output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_SetDefaultResources(
        IN MMC_CONNECT_HNDL                 hConn,
        IN MMC_SETDEFAULTRESOURCES_IN*      pInParam,
        OUT MMC_SETDEFAULTRESOURCES_OUT*    pOutParam);


////////////////////////////////////////////////////////////////////////////////
/// \fn int  MMC_KillRepetitive(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_KILLREPETITIVE_IN* pInParam,
///             OUT MMC_KILLREPETITIVE_OUT* pOutParam)
/// \brief  This function stop repetitive motion after the current FB.
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] axis\group reference
/// \param  pInParam - [IN] dummy
/// \param  pOutParam - [OUT] return the status and error ID
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_KillRepetitive(
                            IN MMC_CONNECT_HNDL         hConn,
                            IN MMC_AXIS_REF_HNDL        hAxisRef,
                            IN MMC_KILLREPETITIVE_IN*   pInParam,
                            OUT MMC_KILLREPETITIVE_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int  MMC_UserCommandControl(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_USRCOMMADN_IN* pInParam,
///             OUT MMC_USRCOMMADN_OUT* pOutParam)
/// \brief  This function execute user command (execute user program or execute LINUX command)
/// \param  hConn - [IN] Connection handle
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_UserCommandControl(
        IN MMC_CONNECT_HNDL     hConn,
        IN MMC_USRCOMMAND_IN*   pInParam,
        OUT MMC_USRCOMMAND_OUT* pOutParam);

MMC_LIB_API ELMO_INT32 MMC_SetAllFbExeModeImm(
        IN  MMC_CONNECT_HNDL                hConn,
        IN  MMC_SETALLFBEXEMODETOIMM_IN*    pInParam,
        OUT MMC_SETALLFBEXEMODETOIMM_OUT*   pOutParam);


////////////////////////////////////////////////////////////////////////////////
/// \fn int  MMC_GetGMASInfo(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_GETGMASINFO_IN* pInParam,
///             OUT MMC_GETGMASINFO_OUT* pOutParam)
/// \brief  This function retrive the GMAS rev and which type of endiann the GMAS support
/// \param  hConn - [IN] Connection handle
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32  MMC_GetGMASInfo(
        IN MMC_CONNECT_HNDL         hConn,
        IN MMC_GETGMASINFO_IN*      pInParam,
        OUT MMC_GETGMASINFO_OUT*    pOutParam);

MMC_LIB_API ELMO_INT32 MMC_AddSecondaryCBK( MMC_CONNECT_HNDL hConn,MMC_CB_FUNC pCbFunc);

                            /* Return boolean:                                  */
                            /*      true=>user program running on LITTLE Endian */
                            /*      false=>user program running on BIG    Endian*/
MMC_LIB_API ELMO_BOOL   MMC_IsLittleEndianIamRunning(void);

                            /* Update variable which point whether              */
                            /* GMAS Endian is same or different Endian relating */
                            /* to host (CPU running user program).              */
                            /* Uses (in RPC) for direct if GMAS's variables     */
                            /* when send / read from host program should swap.  */
                            /* Probably calls from '*Connect*()' functions      */
                            /* Attention: Host-Cpu running user program able    */
                            /* (E.g. RPC) comunicate on one Gmas connection     */
                            /* as Little Endian and to (same sesstion)          */
                            /* other GMAS (other connection) as Big Endian.     */
MMC_LIB_API ELMO_INT32 MMC_UPD_GmasConnectionEndian(IN MMC_CONNECT_HNDL pHndl);


MMC_LIB_API ELMO_INT32 MMC_GetMaestroTime(
    IN MMC_CONNECT_HNDL hConn,
    IN MMC_GET_MAESTRO_TIME_IN* pInParam,
    OUT MMC_GET_MAESTRO_TIME_OUT* pOutParam);

MMC_LIB_API ELMO_INT32 MMC_SetMaestroTime(
    IN MMC_CONNECT_HNDL hConn,
    IN MMC_SET_MAESTRO_TIME_IN* pInParam,
    OUT MMC_SET_MAESTRO_TIME_OUT* pOutParam);

MMC_LIB_API ELMO_INT32 MMC_GetSystemData(MMC_CONNECT_HNDL hConn, MMC_GETSYSTEMDATA_OUT *pOutParam);

MMC_LIB_API ELMO_INT32 GetNodeDirectData(MMC_CONNECT_HNDL hConn,IN MMC_GETNODEDIRECTDATA_IN* pInParam,OUT MMC_GETNODEDIRECTDATA_OUT* pOutParam);

MMC_LIB_API ELMO_INT32 GetGblParamOffset(MMC_CONNECT_HNDL hConn,IN MMC_GETGLBLPARAMOFFSET_IN* pInParam,MMC_GETGLBLPARAMOFFSET_OUT* pOutParam);

MMC_LIB_API ELMO_INT32 GetAxesName(MMC_CONNECT_HNDL hConn,IN MMC_GETAXESNAME_IN* pInParam,MMC_GETAXESNAME_OUT* pOutParam);

MMC_LIB_API ELMO_INT32 MMC_GetPINumber(
        IN MMC_CONNECT_HNDL hConn,
        IN MMC_AXIS_REF_HNDL hAxisRef,
        IN MMC_GETPINUMBER_IN* pInParam,
        OUT MMC_GETPINUMBER_OUT* pOutParam);

MMC_LIB_API ELMO_INT32 MMC_BurnCPLD(
        MMC_CONNECT_HNDL hConn,
        IN MMC_BURN_CPLD_IN* pInParam,
        OUT MMC_BURN_CPLD_OUT *pOutParam);

MMC_LIB_API ELMO_INT32 MMC_ConfigCPLD(
        MMC_CONNECT_HNDL hConn,
        IN MMC_CONFIG_CPLD_IN* pInParam,
        OUT MMC_CONFIG_CPLD_OUT *pOutParam);

//moving to g++ @YL 4-10-2010
#ifdef __cplusplus
}
#endif

#endif
