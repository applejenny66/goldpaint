////////////////////////////////////////////////////////////////////////////////
/// \file MMC_PLCopen_single_API.h
/// Name        : MMC_PLCopen_single_API.h
/// Author      : Barak R
/// Created on  : April 19, 20010
/// Version     : 0.0.1
///               0.2.0 Updated 20Jan2015 Haim H. native Data Types (names), for supporting 64B OS.
/// Copyright   : Your copyright notice
/// Description : This file contain definitions for ...
////////////////////////////////////////////////////////////////////////////////

#ifndef MMC_PLCOPEN_SINGLE_API_H
#define MMC_PLCOPEN_SINGLE_API_H

//moving to g++ @YL 4-10-2010
#ifdef __cplusplus
 extern "C" {
#endif

///////////////////////////////////////////////////////////////////////////////
/// Axis Status Bit Mask
///////////////////////////////////////////////////////////////////////////////
#define NC_AXIS_IN_ERROR_REACTION           0x00080000  ///< Axis\Group is currently in error reaction bit Mask
#define NC_AXIS_BUSY_MASK                   0x00000800  ///< Axis Busy State Bit Mask.
#define NC_AXIS_ERROR_STOP_MASK             0x00000400  ///< Axis Error Stop State Bit Mask.
#define NC_AXIS_DISABLED_MASK               0x00000200  ///< Axis Disabled State Bit Mask.
#define NC_AXIS_STOPPING_MASK               0x00000100  ///< Axis Stopping State Bit Mask.
#define NC_AXIS_STAND_STILL_MASK            0x00000080  ///< Axis Stand Still State Bit Mask.
#define NC_AXIS_DISCRETE_MOTION_MASK        0x00000040  ///< Axis Discrete Motion State Bit Mask.
#define NC_AXIS_CONTINUOUS_MOTION_MASK      0x00000020  ///< Axis Continuous Motion State Bit Mask.
#define NC_AXIS_SYNCHRONIZED_MOTION_MASK    0x00000010  ///< Axis Synchronized Motion State Bit Mask.
#define NC_AXIS_HOMING_MASK                 0x00000008  ///< Axis Homing State Bit Mask.
#define NC_AXIS_CONSTANT_VELOCITY_MASK      0x00000004  ///< Axis Constant Velocity State Bit Mask.
#define NC_AXIS_ACCELERATING_MASK           0x00000002  ///< Axis Accelerating State Bit Mask.
#define NC_AXIS_DECELERATING_MASK           0x00000001  ///< Axis Decelerating State Bit Mask.

///////////////////////////////////////////////////////////////////////////////
// Axis Status Bit Mask
///////////////////////////////////////////////////////////////////////////////



//////////// structures

///////////////////////////////////////////////////////////////////////////////
/// \enum MC_SWITCH_MODE_ENUM
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef enum
{
    MC_ON,
    MC_OFF,
    MC_EDGE_ON,
    MC_EDGE_OFF,
    MC_EDGE_SWITCH_POSITIVE,
    MC_EDGE_SWITCH_NEGATIVE
}MC_SWITCH_MODE_ENUM;

///////////////////////////////////////////////////////////////////////////////
/// \enum MC_HOME_DIRECTION_ENUM
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef enum
{
    MC_POSITIVE,
    MC_NEGATIVE,
    MC_SWITCH_POSITIVE,
    MC_SWITCH_NEGATIVE,
}MC_HOME_DIRECTION_ENUM;

///////////////////////////////////////////////////////////////////////////////
/// \enum MMC_HOME_MODE_ENUM
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef enum {
    MC_ABS_SWITCH,
    MC_LIMIT_SWITCH,
    MC_REF_PULSE,
    MC_DIRECT,
    MC_BLOCK,
}MMC_HOME_MODE_ENUM;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_MOVEABSOLUTE_IN
/// \brief Move Absolute Command input data structure.
///
/// (See : "Technical Specification
///     PLCopen - Technical Committee 2 � Task Force
///     Function blocks for motion control")
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_DOUBLE dbPosition;                  ///< Target position for the motion (in technical unit [u]) (negative or positive)
    ELMO_FLOAT fVelocity;                    ///< Value of the maximum velocity (always positive) (not necessarily reached) [u/s].
    ELMO_FLOAT fAcceleration;                ///< Value of the acceleration (always positive) (increasing energy of the motor) [u/s2]
    ELMO_FLOAT fDeceleration;                ///< Value of the deceleration (always positive) (decreasing energy of the motor) [u/s2]
    ELMO_FLOAT fJerk;                        ///< Value of the Jerk [u/s3]. (always positive)
    MC_DIRECTION_ENUM eDirection;           ///< MC_Direction Enumerator type (1-of-4 values: positive_direction, shortest_way, negative_direction, current_direction)
    MC_BUFFERED_MODE_ENUM eBufferMode;      ///< MC_BufferMode Defines the behavior of the axis: modes are Aborting, Buffered, Blending
    ELMO_UINT8 ucExecute;                   ///< Start the motion at rising edge
} MMC_MOVEABSOLUTE_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_MOVEABSOLUTE_OUT
/// \brief Move Absolute Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT32   uiHndl;       ///< Returned function block handle.
    ELMO_UINT16 usStatus;       ///< Returned command status.
    ELMO_INT16 usErrorID;       ///< Returned command error ID.
} MMC_MOVEABSOLUTE_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_MOVEABSOLUTEEX_IN
/// \brief Move Absolute Extended Command input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_DOUBLE dbPosition;                  ///< Target position for the motion (in technical unit [u]) (negative or positive)
    ELMO_DOUBLE dVelocity;                   ///< Value of the maximum velocity (always positive) (not necessarily reached) [u/s].
    ELMO_DOUBLE dAcceleration;               ///< Value of the acceleration (always positive) (increasing energy of the motor) [u/s2]
    ELMO_DOUBLE dDeceleration;               ///< Value of the deceleration (always positive) (decreasing energy of the motor) [u/s2]
    ELMO_DOUBLE dJerk;                       ///< Value of the Jerk [u/s3]. (always positive)
    MC_DIRECTION_ENUM eDirection;           ///< MC_Direction Enumerator type (1-of-4 values: positive_direction, shortest_way, negative_direction, current_direction)
    MC_BUFFERED_MODE_ENUM eBufferMode;      ///< MC_BufferMode Defines the behavior of the axis: modes are Aborting, Buffered, Blending
    ELMO_UINT8 ucExecute;                   ///< Start the motion at rising edge
} MMC_MOVEABSOLUTEEX_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_MOVEABSOLUTEEX_OUT
/// \brief Move Absolute Extended Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT32   uiHndl;       ///< Returned function block handle.
    ELMO_UINT16 usStatus;       ///< Returned command status.
    ELMO_INT16 usErrorID;       ///< Returned command error ID.
} MMC_MOVEABSOLUTEEX_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_MOVEADDITIVE_IN
/// \brief Move Additive Command input data structure.
///
/// (See : "Technical Specification
///     PLCopen - Technical Committee 2 � Task Force
///     Function blocks for motion control")
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_DOUBLE dbDistance;                  ///< Relative distance for the motion (in technical unit [u])
    ELMO_FLOAT fVelocity;                    ///< Value of the maximum velocity (always positive) (not necessarily reached) [u/s].
    ELMO_FLOAT fAcceleration;                ///< Value of the acceleration (always positive) (increasing energy of the motor) [u/s2]
    ELMO_FLOAT fDeceleration;                ///< Value of the deceleration (always positive) (decreasing energy of the motor) [u/s2]
    ELMO_FLOAT fJerk;                        ///< Value of the Jerk [u/s3]. (always positive)
    MC_DIRECTION_ENUM eDirection;           ///< MC_Direction Enum type (1-of-4 values: positive_direction, shortest_way, negative_direction, current_direction)
    MC_BUFFERED_MODE_ENUM eBufferMode;      ///< MC_BufferMode Defines the behavior of the axis: modes are Aborting, Buffered, Blending
    ELMO_UINT8 ucExecute;                   ///< Start the motion at rising edge
}MMC_MOVEADDITIVE_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_MOVEADDITIVE_OUT
/// \brief Move Additive Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT32   uiHndl;       ///< Returned function block handle.
    ELMO_UINT16 usStatus;       ///< Returned command status.
    ELMO_INT16 usErrorID;       ///< Returned command error ID.
}MMC_MOVEADDITIVE_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_MOVEADDITIVEEX_IN
/// \brief Move Additive Extended Command input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_DOUBLE dbDistance;                  ///< Relative distance for the motion (in technical unit [u])
    ELMO_DOUBLE dVelocity;                   ///< Value of the maximum velocity (always positive) (not necessarily reached) [u/s].
    ELMO_DOUBLE dAcceleration;               ///< Value of the acceleration (always positive) (increasing energy of the motor) [u/s2]
    ELMO_DOUBLE dDeceleration;               ///< Value of the deceleration (always positive) (decreasing energy of the motor) [u/s2]
    ELMO_DOUBLE dJerk;                       ///< Value of the Jerk [u/s3]. (always positive)
    MC_DIRECTION_ENUM eDirection;           ///< MC_Direction Enum type (1-of-4 values: positive_direction, shortest_way, negative_direction, current_direction)
    MC_BUFFERED_MODE_ENUM eBufferMode;      ///< MC_BufferMode Defines the behavior of the axis: modes are Aborting, Buffered, Blending
    ELMO_UINT8 ucExecute;                   ///< Start the motion at rising edge
}MMC_MOVEADDITIVEEX_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_MOVEADDITIVEEX_OUT
/// \brief Move Additive Extended Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT32   uiHndl;       ///< Returned function block handle.
    ELMO_UINT16 usStatus;       ///< Returned command status.
    ELMO_INT16 usErrorID;       ///< Returned command error ID.
}MMC_MOVEADDITIVEEX_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_MOVERELATIVE_IN
/// \brief Move Relative Command input data structure.
///
/// (See : "Technical Specification
///     PLCopen - Technical Committee 2 � Task Force
///     Function blocks for motion control")
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_DOUBLE dbDistance;                  ///< Relative distance for the motion (in technical unit [u])
    ELMO_FLOAT fVelocity;                    ///< Value of the maximum velocity (always positive) (not necessarily reached) [u/s].
    ELMO_FLOAT fAcceleration;                ///< Value of the acceleration (always positive) (increasing energy of the motor) [u/s2]
    ELMO_FLOAT fDeceleration;                ///< Value of the deceleration (always positive) (decreasing energy of the motor) [u/s2]
    ELMO_FLOAT fJerk;                        ///< Value of the Jerk [u/s3]. (always positive)
    MC_DIRECTION_ENUM eDirection;           ///< MC_Direction Enum type (1-of-4 values: positive_direction, shortest_way, negative_direction, current_direction)
    MC_BUFFERED_MODE_ENUM eBufferMode;      ///< MC_BufferMode Defines the behavior of the axis: modes are Aborting, Buffered, Blending
    ELMO_UINT8 ucExecute;                   ///< Start the motion at rising edge
}MMC_MOVERELATIVE_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_MOVERELATIVE_OUT
/// \brief Move Relative Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT32   uiHndl;       ///< Returned function block handle.
    ELMO_UINT16 usStatus;       ///< Returned command status.
    ELMO_INT16 usErrorID;       ///< Returned command error ID.
}MMC_MOVERELATIVE_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_MOVERELATIVEEX_IN
/// \brief Move Relative Extended Command input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_DOUBLE dbDistance;                  ///< Relative distance for the motion (in technical unit [u])
    ELMO_DOUBLE dVelocity;                   ///< Value of the maximum velocity (always positive) (not necessarily reached) [u/s].
    ELMO_DOUBLE dAcceleration;               ///< Value of the acceleration (always positive) (increasing energy of the motor) [u/s2]
    ELMO_DOUBLE dDeceleration;               ///< Value of the deceleration (always positive) (decreasing energy of the motor) [u/s2]
    ELMO_DOUBLE dJerk;                       ///< Value of the Jerk [u/s3]. (always positive)
    MC_DIRECTION_ENUM eDirection;           ///< MC_Direction Enum type (1-of-4 values: positive_direction, shortest_way, negative_direction, current_direction)
    MC_BUFFERED_MODE_ENUM eBufferMode;      ///< MC_BufferMode Defines the behavior of the axis: modes are Aborting, Buffered, Blending
    ELMO_UINT8 ucExecute;                   ///< Start the motion at rising edge
}MMC_MOVERELATIVEEX_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_MOVERELATIVEEX_OUT
/// \brief Move Relative Extended Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT32   uiHndl;       ///< Returned function block handle.
    ELMO_UINT16 usStatus;       ///< Returned command status.
    ELMO_INT16 usErrorID;       ///< Returned command error ID.
}MMC_MOVERELATIVEEX_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_MOVEVELOCITY_IN
/// \brief Move Velocity Command input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_FLOAT fVelocity;
    ELMO_FLOAT fAcceleration;
    ELMO_FLOAT fDeceleration;
    ELMO_FLOAT fJerk;
    MC_DIRECTION_ENUM eDirection;
    MC_BUFFERED_MODE_ENUM eBufferMode;
    ELMO_UINT8 ucExecute;
}MMC_MOVEVELOCITY_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_MOVEVELOCITY_OUT
/// \brief Move Velocity Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT32   uiHndl;       ///< Returned function block handle.
    ELMO_UINT16 usStatus;       ///< Returned command status.
    ELMO_INT16 usErrorID;       ///< Returned command error ID.
}MMC_MOVEVELOCITY_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_MOVEVELOCITYEX_IN
/// \brief Move Velocity Extended Command input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_DOUBLE dVelocity;
    ELMO_DOUBLE dAcceleration;
    ELMO_DOUBLE dDeceleration;
    ELMO_DOUBLE dJerk;
    MC_DIRECTION_ENUM eDirection;
    MC_BUFFERED_MODE_ENUM eBufferMode;
    ELMO_UINT8 ucExecute;
}MMC_MOVEVELOCITYEX_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_MOVEVELOCITYEX_OUT
/// \brief Move Velocity Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT32   uiHndl;       ///< Returned function block handle.
    ELMO_UINT16 usStatus;       ///< Returned command status.
    ELMO_INT16 usErrorID;       ///< Returned command error ID.
}MMC_MOVEVELOCITYEX_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_MOVECONTINUOUS_IN
/// \brief Move Continuous Command input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_DOUBLE dbDistance;
    ELMO_FLOAT fVelocity;
    ELMO_FLOAT fEndVelocity;
    ELMO_FLOAT fAcceleration;
    ELMO_FLOAT fDeceleration;
    ELMO_FLOAT fJerk;
    MC_BUFFERED_MODE_ENUM eBufferMode;
    ELMO_UINT8 ucExecute;
}MMC_MOVECONTINUOUS_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_MOVECONTINUOUS_OUT
/// \brief Move Continuous Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT32   uiHndl;       ///< Returned function block handle.
    ELMO_UINT16 usStatus;       ///< Returned command status.
    ELMO_INT16 usErrorID;       ///< Returned command error ID.
}MMC_MOVECONTINUOUS_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_HOME_IN
/// \brief Home Command input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_DOUBLE dbPosition;
    ELMO_FLOAT fAcceleration;
    ELMO_FLOAT fVelocity;
    ELMO_FLOAT fDistanceLimit;
    ELMO_FLOAT fTorqueLimit;
    MMC_HOME_MODE_ENUM eHomingMode;
    MC_BUFFERED_MODE_ENUM eBufferMode;
    MC_HOME_DIRECTION_ENUM eDirection;
    MC_SWITCH_MODE_ENUM eSwitchMode;
    ELMO_UINT32 uiTimeLimit;
    ELMO_UINT8 ucExecute;
}MMC_HOME_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_HOME_OUT
/// \brief Home Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT32   uiHndl;       ///< Returned function block handle.
    ELMO_UINT16 usStatus;       ///< Returned command status.
    ELMO_INT16 usErrorID;       ///< Returned command error ID.
}MMC_HOME_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_HOMEDS402_IN
/// \brief Home DS402 Command input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_DOUBLE dbPosition;
    ELMO_FLOAT fAcceleration;
    ELMO_FLOAT fVelocity;
    ELMO_FLOAT fDistanceLimit;
    ELMO_FLOAT fTorqueLimit;
    MC_BUFFERED_MODE_ENUM eBufferMode;
    ELMO_INT32 uiHomingMethod;
    ELMO_UINT32 uiTimeLimit;
    ELMO_UINT8 ucExecute;
}MMC_HOMEDS402_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_HOMEDS402EX_IN
/// \brief Home DS402EX Command input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_homeds402ex_in
{
    ELMO_DOUBLE dbPosition;
    ELMO_DOUBLE dbDetectionVelocityLimit;
    ELMO_FLOAT fAcceleration;
    ELMO_FLOAT fVelocityHi;   //speed search for switch
    ELMO_FLOAT fVelocityLo;   //speed search for zero
    ELMO_FLOAT fDistanceLimit;
    ELMO_FLOAT fTorqueLimit;
    MC_BUFFERED_MODE_ENUM eBufferMode;
    ELMO_INT32 uiHomingMethod;
    ELMO_UINT32 uiTimeLimit;
    ELMO_UINT32 uiDetectionTimeLimit;
    ELMO_UINT8 ucExecute;
    ELMO_UINT8 ucSpares[32]; //future usage
} MMC_HOMEDS402EX_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READACTUALPOSITION_IN
/// \brief Read Actual Position Command input data structure.
///
/// (See : " Technical Specification
///     PLCopen - Technical Committee 2 � Task Force
///     Function blocks for motion control")
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT8 ucEnable;        ///< Get the value of the parameter continuously while enabled
}MMC_READACTUALPOSITION_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READACTUALPOSITION_OUT
/// \brief Read Actual Position Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_DOUBLE dbPosition;      ///<New absolute position (in axis� unit [u])
    ELMO_UINT16 usStatus;       ///< Returned command status.
    ELMO_INT16 usErrorID;       ///< Returned command error ID.
}MMC_READACTUALPOSITION_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READACTUALVELOCITY_IN
/// \brief Read Actual Velocity Command input data structure.
///
/// (See : " Technical Paper
///     PLCopen Technical Committee
///     Function Blocks for motion control:
///     Part 2 - Extensions")
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT8 ucEnable;        ///< Get the value of the parameter continuously while enabled
}MMC_READACTUALVELOCITY_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READACTUALVELOCITY_OUT
/// \brief Read Actual Velocity Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_DOUBLE dVelocity;       ///< The value of the actual velocity (in axis� unit [u/s])
    ELMO_UINT16 usStatus;       ///< Returned command status.
    ELMO_INT16 usErrorID;       ///< Returned command error ID.
}MMC_READACTUALVELOCITY_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READAXISERROR_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT8 ucEnable;
}MMC_READAXISERROR_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READAXISERROR_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT16 usStatus;
    ELMO_INT16 usErrorID;
    ELMO_UINT16 usAxisErrorID;
    ELMO_UINT16 usLastEmergencyErrCode;
}MMC_READAXISERROR_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_HALT_IN
/// \brief Halt Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_FLOAT fDeceleration;
    ELMO_FLOAT fJerk;
    MC_BUFFERED_MODE_ENUM eBufferMode;
    ELMO_UINT8 ucExecute;
}MMC_HALT_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_HALT_OUT
/// \brief Halt Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT32   uiHndl;       ///< Returned function block handle.
    ELMO_UINT16 usStatus;       ///< Returned command status.
    ELMO_INT16 usErrorID;       ///< Returned command error ID.
}MMC_HALT_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READDIGITALINPUT_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_INT32 iInputNumber;
    ELMO_UINT8 ucEnable;
}MMC_READDIGITALINPUT_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READDIGITALINPUT_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT16 usStatus;
    ELMO_INT16 usErrorID;
    ELMO_UINT8 ucValue;
}MMC_READDIGITALINPUT_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READDIGITALINPUTS_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT8 ucEnable;
}MMC_READDIGITALINPUTS_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READDIGITALINPUTS_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_ULINT32 ulValue;
    ELMO_UINT16 usStatus;
    ELMO_INT16 usErrorID;
}MMC_READDIGITALINPUTS_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READDIGITALOUTPUT_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readdigitaloutput_in
{
    ELMO_INT32 iOutputNumber;
    ELMO_UINT8 ucEnable;
}MMC_READDIGITALOUTPUT_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READDIGITALOUTPUT_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readdigitaloutput_out
{
    ELMO_UINT16 usStatus;
    ELMO_INT16 usErrorID;
    ELMO_UINT8 ucValue;
}MMC_READDIGITALOUTPUT_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEDIGITALOUTPUT_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writedigitaloutput_in
{
    ELMO_INT32 iOutputNumber;
    ELMO_UINT8 ucEnable;
    ELMO_UINT8 ucValue;
}MMC_WRITEDIGITALOUTPUT_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEDIGITALOUTPUT_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writedigitaloutput_out
{
    ELMO_UINT16 usStatus;
    ELMO_INT16 usErrorID;
}MMC_WRITEDIGITALOUTPUT_OUT;


///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READDIGITALOUTPUT32Bit_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readdigitaloutput32Bit_in
{
    ELMO_INT32 iOutputNumber;
    ELMO_UINT8 ucEnable;
}MMC_READDIGITALOUTPUT32Bit_IN;


///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READDIGITALOUTPUT32Bit_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_readdigitaloutput32Bit_out
{
    ELMO_ULINT32 ulValue;
    ELMO_UINT16 usStatus;
    ELMO_INT16 usErrorID;
}MMC_READDIGITALOUTPUT32Bit_OUT;


///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEDIGITALOUTPUT32Bit_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writedigitaloutput32Bit_in
{
    ELMO_INT32 iOutputNumber;
    ELMO_ULINT32 ulValue;
    ELMO_UINT8 ucEnable;
}MMC_WRITEDIGITALOUTPUT32Bit_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_WRITEDIGITALOUTPUT32Bit_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_writedigitaloutput32Bit_out
{
    ELMO_UINT16 usStatus;
    ELMO_INT16 usErrorID;
}MMC_WRITEDIGITALOUTPUT32Bit_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_SETPOSITION_IN
/// \brief Set Position Command input data structure.
///
/// (See : "Technical Paper
///     PLCopen Technical Committee
///     Function Blocks for motion control:
///     Part 2 - Extensions")
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_DOUBLE dbPosition;
    MC_EXECUTION_MODE eExecutionMode;
    ELMO_UINT8 ucPosMode; // abs mode = 0. relative mode = 1.
    ELMO_UINT8 ucExecute;
} MMC_SETPOSITION_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_SETPOSITION_OUT
/// \brief Set Position Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT16 usStatus;       ///< Returned command status.
    ELMO_INT16 usErrorID;       ///< Returned command error ID.
} MMC_SETPOSITION_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_STOP_IN
/// \brief Stop Command input data structure.
///
/// (See : "Technical Specification
///     PLCopen - Technical Committee 2 � Task Force
///     Function blocks for motion control")
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_FLOAT fDeceleration;    ///< Value of the deceleration [u/s2]
    ELMO_FLOAT fJerk;            ///< Value of the Jerk [u/s3]
    MC_BUFFERED_MODE_ENUM eBufferMode;  ///< Defines the behavior of the axis: modes are Aborting, Buffered, Blending
    ELMO_UINT8 ucExecute;       ///< Start the action at rising edge
}MMC_STOP_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_STOP_OUT
/// \brief Stop Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT32   uiHndl;       ///< Returned function block handle.
    ELMO_UINT16 usStatus;       ///< Returned command status.
    ELMO_INT16 usErrorID;       ///< Returned command error ID.
}MMC_STOP_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_RESET_IN
/// \brief Reset Command input data structure.
///
/// (See : "Technical Specification
///     PLCopen - Technical Committee 2 � Task Force
///     Function blocks for motion control")
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT8 ucExecute;
}MMC_RESET_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_RESET_OUT
/// \brief Reset Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT16 usStatus;
    ELMO_INT16 usErrorID;
}MMC_RESET_OUT;


///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READSTATUS_IN
/// \brief Read Axis Status Command input data structure.
///
/// (See : "Technical Specification
///     PLCopen - Technical Committee 2 � Task Force
///     Function blocks for motion control")
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT32  uiHndlr;       ///< Requested Axis handle.
    ELMO_UINT8 ucEnable;        ///< Get the value of the parameter continuously while enabled
} MMC_READSTATUS_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READSTATUS_OUT
/// \brief Read Axis Status Command input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    /* HH unsigned long ulState;    */      ///< Axis state.
    ELMO_UINT32 ulState;         ///< Axis state.

    ELMO_UINT16 usStatus;        ///< Returned command status.
    ELMO_INT16 usErrorID;        ///< Returned command error ID.
    ELMO_UINT16 usAxisErrorID;   ///< Axis error ID.
    ELMO_UINT16 usStatusWord;    ///< Drive Status Word.
} MMC_READSTATUS_OUT;



///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_POWER_IN
/// \brief Power On Command input data structure.
///
/// (See : "Technical Specification
///     PLCopen - Technical Committee 2 � Task Force
///     Function blocks for motion control")
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    MC_BUFFERED_MODE_ENUM eBufferMode;
    ELMO_UINT8 ucEnable;
    ELMO_UINT8 ucEnablePositive;
    ELMO_UINT8 ucEnableNegative;
    ELMO_UINT8 ucExecute;
} MMC_POWER_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_POWER_OUT
/// \brief Power On Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT32   uiHndl;       ///< Returned function block handle.
    ELMO_UINT16 usStatus;       ///< Returned command status.
    ELMO_INT16 usErrorID;       ///< Returned command error ID.
} MMC_POWER_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_GETFBDEPTH_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT32  uiHndl;
}MMC_GETFBDEPTH_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_GETFBDEPTH_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT32  uiFbInQ;
    ELMO_UINT16 usStatus;
    ELMO_INT16 usErrorID;
}MMC_GETFBDEPTH_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READACTUALTORQUE_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT8 ucEnable;
}MMC_READACTUALTORQUE_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_READACTUALTORQUE_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_DOUBLE dActualTorque;
    ELMO_UINT16 usStatus;
    ELMO_INT16 usErrorID;
    ELMO_UINT8 ucValid;
}MMC_READACTUALTORQUE_OUT;

/* Added by Alex - Integration of MoveRepetitive() */
///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_MOVEABSOLUTEREPETITIVE_IN
/// \brief Move Absolute Repetitive Command input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_DOUBLE dbPosition;                  ///< Target position for the motion (in technical unit [u]) (negative or positive)
    ELMO_FLOAT fVelocity;                    ///< Value of the maximum velocity (always positive) (not necessarily reached) [u/s].
    ELMO_FLOAT fAcceleration;                ///< Value of the acceleration (always positive) (increasing energy of the motor) [u/s2]
    ELMO_FLOAT fDeceleration;                ///< Value of the deceleration (always positive) (decreasing energy of the motor) [u/s2]
    ELMO_FLOAT fJerk;                        ///< Value of the Jerk [u/s3]. (always positive)
    MC_DIRECTION_ENUM eDirection;           ///< MC_Direction Enumerator type (1-of-4 values: positive_direction, shortest_way, negative_direction, current_direction)
    MC_BUFFERED_MODE_ENUM eBufferMode;      ///< MC_BufferMode Defines the behavior of the axis: modes are Aborting, Buffered, Blending
    ELMO_UINT32  uiExecDelayMs;             ///< Defines the delay between 2 blocks (?)
    ELMO_UINT8 ucExecute;                   ///< Start the motion at rising edge
} MMC_MOVEABSOLUTEREPETITIVE_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_MOVEABSOLUTEREPETITIVE_OUT
/// \brief Move Absolute Repetitive Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT32   uiHndl;                   ///< Returned function block handle.
    ELMO_UINT16 usStatus;                   ///< Returned command status.
    ELMO_INT16 usErrorID;                   ///< Returned command error ID.
} MMC_MOVEABSOLUTEREPETITIVE_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_MOVEABSOLUTEREPETITIVEEX_IN
/// \brief Move Absolute Repetitive Extended Command input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_DOUBLE dbPosition;                  ///< Target position for the motion (in technical unit [u]) (negative or positive)
    ELMO_DOUBLE dVelocity;                   ///< Value of the maximum velocity (always positive) (not necessarily reached) [u/s].
    ELMO_DOUBLE dAcceleration;               ///< Value of the acceleration (always positive) (increasing energy of the motor) [u/s2]
    ELMO_DOUBLE dDeceleration;               ///< Value of the deceleration (always positive) (decreasing energy of the motor) [u/s2]
    ELMO_DOUBLE dJerk;                       ///< Value of the Jerk [u/s3]. (always positive)
    MC_DIRECTION_ENUM eDirection;           ///< MC_Direction Enumerator type (1-of-4 values: positive_direction, shortest_way, negative_direction, current_direction)
    MC_BUFFERED_MODE_ENUM eBufferMode;      ///< MC_BufferMode Defines the behavior of the axis: modes are Aborting, Buffered, Blending
    ELMO_UINT32  uiExecDelayMs;             ///< Defines the delay between 2 blocks (?)
    ELMO_UINT8 ucExecute;                   ///< Start the motion at rising edge
} MMC_MOVEABSOLUTEREPETITIVEEX_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_MOVEABSOLUTEREPETITIVEEX_OUT
/// \brief Move Absolute Repetitive Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT32   uiHndl;                   ///< Returned function block handle.
    ELMO_UINT16 usStatus;                   ///< Returned command status.
    ELMO_INT16 usErrorID;                   ///< Returned command error ID.
} MMC_MOVEABSOLUTEREPETITIVEEX_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_MOVERELATIVEREPETITIVE_IN
/// \brief Move Relative Repetitive Command input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_DOUBLE dbDistance;                  ///< Target position for the motion (in technical unit [u]) (negative or positive)
    ELMO_FLOAT fVelocity;                    ///< Value of the maximum velocity (always positive) (not necessarily reached) [u/s].
    ELMO_FLOAT fAcceleration;                ///< Value of the acceleration (always positive) (increasing energy of the motor) [u/s2]
    ELMO_FLOAT fDeceleration;                ///< Value of the deceleration (always positive) (decreasing energy of the motor) [u/s2]
    ELMO_FLOAT fJerk;                        ///< Value of the Jerk [u/s3]. (always positive)
    MC_DIRECTION_ENUM eDirection;           ///< MC_Direction Enumerator type (1-of-4 values: positive_direction, shortest_way, negative_direction, current_direction)
    MC_BUFFERED_MODE_ENUM eBufferMode;      ///< MC_BufferMode Defines the behavior of the axis: modes are Aborting, Buffered, Blending
    ELMO_UINT32  uiExecDelayMs;             ///< Defines the delay between 2 blocks (?)
    ELMO_UINT8 ucExecute;                   ///< Start the motion at rising edge
} MMC_MOVERELATIVEREPETITIVE_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_MOVERELATIVEREPETITIVE_OUT
/// \brief Move Absolute Repetitive Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT32   uiHndl;                   ///< Returned function block handle.
    ELMO_UINT16 usStatus;                   ///< Returned command status.
    ELMO_INT16          usErrorID;          ///< Returned command error ID.
} MMC_MOVERELATIVEREPETITIVE_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_MOVERELATIVEREPETITIVEEX_IN
/// \brief Move Relative Repetitive Command input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_DOUBLE dbDistance;                  ///< Target position for the motion (in technical unit [u]) (negative or positive)
    ELMO_DOUBLE dVelocity;                   ///< Value of the maximum velocity (always positive) (not necessarily reached) [u/s].
    ELMO_DOUBLE dAcceleration;               ///< Value of the acceleration (always positive) (increasing energy of the motor) [u/s2]
    ELMO_DOUBLE dDeceleration;               ///< Value of the deceleration (always positive) (decreasing energy of the motor) [u/s2]
    ELMO_DOUBLE dJerk;                       ///< Value of the Jerk [u/s3]. (always positive)
    MC_DIRECTION_ENUM eDirection;           ///< MC_Direction Enumerator type (1-of-4 values: positive_direction, shortest_way, negative_direction, current_direction)
    MC_BUFFERED_MODE_ENUM eBufferMode;      ///< MC_BufferMode Defines the behavior of the axis: modes are Aborting, Buffered, Blending
    ELMO_UINT32  uiExecDelayMs;             ///< Defines the delay between 2 blocks (?)
    ELMO_UINT8 ucExecute;                   ///< Start the motion at rising edge
} MMC_MOVERELATIVEREPETITIVEEX_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_MOVERELATIVEREPETITIVEEX_OUT
/// \brief Move Absolute Repetitive Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT32   uiHndl;                   ///< Returned function block handle.
    ELMO_UINT16 usStatus;                   ///< Returned command status.
    ELMO_INT16          usErrorID;          ///< Returned command error ID.
} MMC_MOVERELATIVEREPETITIVEEX_OUT;


///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_MOVEADDITIVEREPETITIVE_IN
/// \brief Move Additive Repetitive Command input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_DOUBLE dbDistance;                  ///< Target position for the motion (in technical unit [u]) (negative or positive)
    ELMO_FLOAT fVelocity;                    ///< Value of the maximum velocity (always positive) (not necessarily reached) [u/s].
    ELMO_FLOAT fAcceleration;                ///< Value of the acceleration (always positive) (increasing energy of the motor) [u/s2]
    ELMO_FLOAT fDeceleration;                ///< Value of the deceleration (always positive) (decreasing energy of the motor) [u/s2]
    ELMO_FLOAT fJerk;                        ///< Value of the Jerk [u/s3]. (always positive)
    MC_DIRECTION_ENUM eDirection;           ///< MC_Direction Enumerator type (1-of-4 values: positive_direction, shortest_way, negative_direction, current_direction)
    MC_BUFFERED_MODE_ENUM eBufferMode;      ///< MC_BufferMode Defines the behavior of the axis: modes are Aborting, Buffered, Blending
    ELMO_UINT32  uiExecDelayMs;             ///< Defines the delay between 2 blocks (?)
    ELMO_UINT8 ucExecute;                   ///< Start the motion at rising edge
} MMC_MOVEADDITIVEREPETITIVE_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_MOVEADDITIVEREPETITIVE_OUT
/// \brief Move Additive Repetitive Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT32   uiHndl;                   ///< Returned function block handle.
    ELMO_UINT16 usStatus;                   ///< Returned command status.
    ELMO_INT16          usErrorID;          ///< Returned command error ID.
} MMC_MOVEADDITIVEREPETITIVE_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_MOVEADDITIVEREPETITIVE_IN
/// \brief Move Additive Repetitive Command input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_DOUBLE dbDistance;                  ///< Target position for the motion (in technical unit [u]) (negative or positive)
    ELMO_DOUBLE dVelocity;                   ///< Value of the maximum velocity (always positive) (not necessarily reached) [u/s].
    ELMO_DOUBLE dAcceleration;               ///< Value of the acceleration (always positive) (increasing energy of the motor) [u/s2]
    ELMO_DOUBLE dDeceleration;               ///< Value of the deceleration (always positive) (decreasing energy of the motor) [u/s2]
    ELMO_DOUBLE dJerk;                       ///< Value of the Jerk [u/s3]. (always positive)
    MC_DIRECTION_ENUM eDirection;           ///< MC_Direction Enumerator type (1-of-4 values: positive_direction, shortest_way, negative_direction, current_direction)
    MC_BUFFERED_MODE_ENUM eBufferMode;      ///< MC_BufferMode Defines the behavior of the axis: modes are Aborting, Buffered, Blending
    ELMO_UINT32  uiExecDelayMs;             ///< Defines the delay between 2 blocks (?)
    ELMO_UINT8 ucExecute;                   ///< Start the motion at rising edge
} MMC_MOVEADDITIVEREPETITIVEEX_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_MOVEADDITIVEREPETITIVEEX_OUT
/// \brief Move Additive Repetitive Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT32   uiHndl;                   ///< Returned function block handle.
    ELMO_UINT16 usStatus;                   ///< Returned command status.
    ELMO_INT16          usErrorID;          ///< Returned command error ID.
} MMC_MOVEADDITIVEREPETITIVEEX_OUT;


typedef struct mmc_positionprofile_in
{
    MC_BUFFERED_MODE_ENUM eBufferMode;
    MC_PATH_REF hMemHandle;
    ELMO_UINT8 ucExecute;
} MMC_POSITIONPROFILE_IN;

typedef struct mmc_positionprofile_out
{
    ELMO_UINT16 usStatus;
    ELMO_INT16 usErrorID;
} MMC_POSITIONPROFILE_OUT;

/// Touch Probe Modes:
///--------------------
enum
{
    eMMC_TOUCHPROBE_POS_EDGE = 0,
    eMMC_TOUCHPROBE_NEG_EDGE
};

typedef struct mmc_touchprobeenable_in
{
    ELMO_UINT8 ucExecute;
    ELMO_UINT8 ucTriggerType;
}MMC_TOUCHPROBEENABLE_IN;

typedef struct mmc_touchprobeenable_out
{
    //ELMO_UINT32 uiHndl;
    ELMO_UINT16 usStatus;
    ELMO_INT16 usErrorID;
}MMC_TOUCHPROBEENABLE_OUT;

typedef struct mmc_touchprobedisable_in
{
    ELMO_UINT8 ucExecute;
}MMC_TOUCHPROBEDISABLE_IN;

typedef struct mmc_touchprobedisable_out
{
    //ELMO_UINT32 uiHndl;
    ELMO_UINT16 usStatus;
    ELMO_INT16 usErrorID;
}MMC_TOUCHPROBEDISABLE_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_AXISLINK_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_axislink_in
{
    ELMO_ULINT32 ulInputParameter1;
    ELMO_ULINT32 ulInputParameter2;
    ELMO_ULINT32 ulInputParameter3;
    ELMO_ULINT32 ulInputParameter4;
    ELMO_UINT16 usSlaveAxisReference;
    ELMO_UINT8 ucMode;

}MMC_AXISLINK_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_AXISLINK_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_axislink_out
{
    ELMO_UINT16 usStatus;
    ELMO_INT16 usErrorID;
}MMC_AXISLINK_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_AXISUNLINK_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_axisunlink_in
{
    ELMO_UINT8 ucdummy;

}MMC_AXISUNLINK_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_AXISUNLINK_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_axisunlink_out
{
    ELMO_UINT16 usStatus;
    ELMO_INT16 usErrorID;
}MMC_AXISUNLINK_OUT;



///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_CHANGEOPMODE_EX_IN
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_DOUBLE dbInitModeValue;
    MC_EXECUTION_MODE eExecutionMode;   
    ELMO_UINT8 ucMotionMode;
ELMO_UINT8 ucSpare[19];
}MMC_CHANGEOPMODE_EX_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_CHANGEOPMODE_EX_OUT
/// \brief
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT32   uiHndl;
    ELMO_UINT16 usStatus;
    ELMO_UINT16 usErrorID;
 }MMC_CHANGEOPMODE_EX_OUT;



///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_MOVETORQUE_IN
/// \brief Move Torque Command input data structure.
///
/// (See : "Technical Specification
///     PLCopen - Technical Committee 2 � Task Force
///     Function blocks for motion control")
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_DOUBLE dbTargetTorque;              ///< Desired Target Torque value
    ELMO_DOUBLE dbTorquetVelocity;           ///< Maximum Target torque velocity value
    ELMO_DOUBLE dbTorqueAcceleration;        ///< Maximum Target torque acceleration value
    MC_BUFFERED_MODE_ENUM eBufferMode;      ///< MC_BufferMode
    ELMO_UINT8 ucExecute;                   ///< Execution bit
} MMC_MOVETORQUE_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_MOVETORQUE_OUT
/// \brief Move Torque Command output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT32   uiHndl;                   ///< Returned function block handle.
    ELMO_UINT16 usStatus;                   ///< Returned command status.
    ELMO_INT16 usErrorID;                   ///< Returned command error ID.
} MMC_MOVETORQUE_OUT;

typedef enum
{
    eNODE_ERROR_ECAT_PHY_ERROR  = 0,
    eSYS_ERROR_CYCLIC_ERROR     = 1,
    eSYS_ERROR_MISSED_FRAMES    = 2,
    eNODE_ERROR_ECAT_AL_ERROR   = 3,
    eNODE_ERROR_UNEXPEC_MO_0    = 4,
    eNODE_ERROR_DRIVE_FAULT     = 5,
    eNODE_ERROR_QSTOP           = 6,
    eNODE_ERROR_HBT             = 7,
    eNODE_ERROR_EMCY            = 8,
    eNODE_ERROR_FB              = 9,
    eNODE_ERROR_MAX
}ERRORS;

typedef enum
{
    ePOLICY_NO_REACTION            = 0,
    ePOLICY_EVENT                  = 0x1,
    ePOLICY_STOP                   = 0x2,
    ePOLICY_POWER_OFF              = 0x4,
    ePOLICY_SAFEOP                 = 0x8,
    ePOLICY_APP_TO_ALL             = 0x80
}POLICY;

#define MAX_REG_POLICY              20

///////////////////////////////////////////////////////////////////////////////
/// \struct NC_POLICY_ENTRY
/// \brief register error policy input data member structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct nc_policy_entry
{
    ERRORS eErrType;    ///< Error type
    ELMO_UINT8 ucPolicy;     ///< 7 bits of possible policies, MSb defines if the policies are applied to all axes
    ELMO_UINT8 ucThreshold;  ///< the errors threshold for starting the reaction
}
NC_POLICY_ENTRY;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_REGERRPOLICY_IN
/// \brief register error policy input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    NC_POLICY_ENTRY pPolicies[MAX_REG_POLICY];
    ELMO_UINT16 usAxisRef;                   ///< relevant only for node errors
    ELMO_UINT8 ucNum;                        ///< num of policies to register
    ELMO_UINT8 pSpare[64];
} MMC_REGERRPOLICY_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_REGERRPOLICY_OUT
/// \brief register error policy output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT16 usStatus;    ///< Returned command status.
    ELMO_INT16 sErrorID;     ///< Returned command error ID.
} MMC_REGERRPOLICY_OUT;


///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_RESETSYSTEM_IN
/// \brief reset system errors input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT8 ucDummy;
}MMC_RESETSYSTEM_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_RESETSYSERR_OUT
/// \brief set stop options output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ELMO_UINT16 usStatus;    ///< Returned command status.
    ELMO_INT16  sErrorID;    ///< Returned command error ID.
} MMC_RESETSYSTEM_OUT;

///////////////////////////////////////////////////////////////////////////////
/// \struct NC_GET_POLICY_ENTRY
/// \brief get error policy input data member structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct nc_get_policy_entry
{
    ELMO_UINT8 ucPolicy;     ///< 7 bits of possible policies, MSb defines if the policies are applied to all axes
    ELMO_UINT8 ucThreshold;  ///< the errors threshold for starting the reaction
    ELMO_UINT8 ucCurrentVal; ///< current value of registered err
}
NC_GET_POLICY_ENTRY;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_GETERRPOLICY_IN
/// \brief reset system errors input data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    ERRORS pErrType[MAX_REG_POLICY];    ///< the types of errors request
    ELMO_UINT16 usAxisRef;              ///< relevant only for node errors
    ELMO_UINT8  ucNum;                  ///< the requested number of error policies
}MMC_GETERRPOLICY_IN;

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_GETERRPOLICY_OUT
/// \brief set stop options output data structure.
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
    NC_GET_POLICY_ENTRY pPolicies[MAX_REG_POLICY];
    ELMO_UINT16 usStatus;               ///< Returned command status.
    ELMO_INT16  sErrorID;               ///< Returned command error ID.
    ELMO_UINT8  pSpare[64];
} MMC_GETERRPOLICY_OUT;

///////// functions

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_MoveAbsoluteCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_MOVEABSOLUTE_IN* pInParam,
///             OUT MMC_MOVEABSOLUTE_OUT* pOutParam)
/// \brief This function send Move Absolute command to MMC server for specific Axis.
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis Reference handle
/// \param  pInParam - [IN] Pointer to Move Absolute input parameters
/// \param  pOutParam - [OUT] Pointer to Move Absolute output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_MoveAbsoluteCmd(
        IN MMC_CONNECT_HNDL hConn,
        IN MMC_AXIS_REF_HNDL hAxisRef,
        IN MMC_MOVEABSOLUTE_IN* pInParam,
        OUT MMC_MOVEABSOLUTE_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_MoveAdditiveCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_MOVEADDITIVE_IN* pInParam,
///             OUT MMC_MOVEADDITIVE_OUT* pOutParam)
/// \brief This function  send Move Additive command to MMC server for specific Axis.
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis Reference handle
/// \param  pInParam - [IN] Pointer to Move Additive input parameters
/// \param  pOutParam - [OUT] Pointer to Move Additive output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_MoveAdditiveCmd(
        IN MMC_CONNECT_HNDL hConn,
        IN MMC_AXIS_REF_HNDL hAxisRef,
        IN MMC_MOVEADDITIVE_IN* pInParam,
        OUT MMC_MOVEADDITIVE_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_MoveRelativeCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_MOVERELATIVE_IN* pInParam,
///             OUT MMC_MOVERELATIVE_OUT* pOutParam)
/// \brief This function  send Move Relative command to MMC server for specific Axis.
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis Reference handle
/// \param  pInParam - [IN] Pointer to Move Relative input parameters
/// \param  pOutParam - [OUT] Pointer to Move Relative output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_MoveRelativeCmd(
        IN MMC_CONNECT_HNDL hConn,
        IN MMC_AXIS_REF_HNDL hAxisRef,
        IN MMC_MOVERELATIVE_IN* pInParam,
        OUT MMC_MOVERELATIVE_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ReadAxisError(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_READAXISERROR_IN* pInParam,
///             OUT MMC_READAXISERROR_OUT* pOutParam)
/// \brief This function read axis error.
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis Reference handle
/// \param  pInParam - [IN] Pointer to Group Read Error input parameters
/// \param  pOutParam - [OUT] Pointer to Group Read Error output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ReadAxisError(
        IN MMC_CONNECT_HNDL hConn,
        IN MMC_AXIS_REF_HNDL hAxisRef,
        IN MMC_READAXISERROR_IN* pInParam,
        OUT MMC_READAXISERROR_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_MoveVelocityCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_MOVEVELOCITY_IN* pInParam,
///             OUT MMC_MOVEVELOCITY_OUT* pOutParam)
/// \brief This function  send Move Velocity command to MMC server for specific Axis.
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis Reference handle
/// \param  pInParam - [IN] Pointer to Move Velocity input parameters
/// \param  pOutParam - [OUT] Pointer to Move Velocity output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_MoveVelocityCmd(
        IN MMC_CONNECT_HNDL hConn,
        IN MMC_AXIS_REF_HNDL hAxisRef,
        IN MMC_MOVEVELOCITY_IN* pInParam,
        OUT MMC_MOVEVELOCITY_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_MoveContinuousCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_MOVECONTINUOUS_IN* pInParam,
///             OUT MMC_MOVECONTINUOUS_OUT* pOutParam)
/// \brief This function  send Move Continuous command to MMC server for specific Axis.
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis Reference handle
/// \param  pInParam - [IN] Pointer to Move Continuous input parameters
/// \param  pOutParam - [OUT] Pointer to Move Continuous output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_MoveContinuousCmd(
        IN MMC_CONNECT_HNDL hConn,
        IN MMC_AXIS_REF_HNDL hAxisRef,
        IN MMC_MOVECONTINUOUS_IN* pInParam,
        OUT MMC_MOVECONTINUOUS_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_HomeCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_HOME_IN* pInParam,
///             OUT MMC_HOME_OUT* pOutParam)
/// \brief This function  send Home command to MMC server for specific Axis.
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis Reference handle
/// \param  pInParam - [IN] Pointer to Home input parameters
/// \param  pOutParam - [OUT] Pointer to Home output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_HomeCmd(
        IN MMC_CONNECT_HNDL hConn,
        IN MMC_AXIS_REF_HNDL hAxisRef,
        IN MMC_HOME_IN* pInParam,
        OUT MMC_HOME_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_HomeDS402Cmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_HOME_IN* pInParam,
///             OUT MMC_HOME_OUT* pOutParam)
/// \brief This function  send Home DS402 command to MMC server for specific Axis.
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis Reference handle
/// \param  pInParam - [IN] Pointer to Home input parameters
/// \param  pOutParam - [OUT] Pointer to Home output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_HomeDS402Cmd(
        IN MMC_CONNECT_HNDL hConn,
        IN MMC_AXIS_REF_HNDL hAxisRef,
        IN MMC_HOMEDS402_IN* pInParam,
        OUT MMC_HOME_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_HomeDS402ExCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_HOMEDS402EX_IN* pInParam,
///             OUT MMC_HOME_OUT* pOutParam)
/// \brief This function  send Home DS402 command to MMC server for specific Axis.
///         supports Velocity Hi\Lo, DetectionTimeLimit and DetectionVelocityLimit
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis Reference handle
/// \param  pInParam - [IN] Pointer to Home input parameters
/// \param  pOutParam - [OUT] Pointer to Home output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_HomeDS402ExCmd(
        IN MMC_CONNECT_HNDL hConn,
        IN MMC_AXIS_REF_HNDL hAxisRef,
        IN MMC_HOMEDS402EX_IN* pInParam,
        OUT MMC_HOME_OUT* pOutParam);


////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ReadActualPositionCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_READACTUALPOSITION_IN* pInParam,
///             OUT MMC_READACTUALPOSITION_OUT* pOutParam)
/// \brief This function read actual position of specific axis
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis Reference handle
/// \param  pInParam - [IN] input parameters structure for Read Actual Position Command
/// \param  pOutParam - [OUT] output parameters structure for Read Actual Position Command
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ReadActualPositionCmd(
        IN MMC_CONNECT_HNDL hConn,
        IN MMC_AXIS_REF_HNDL hAxisRef,
        IN MMC_READACTUALPOSITION_IN* pInParam,
        OUT MMC_READACTUALPOSITION_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ReadActualVelocityCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_READACTUALVELOCITY_IN* pInParam,
///             OUT MMC_READACTUALVELOCITY_OUT* pOutParam)
/// \brief This function read actual velocity of specific axis
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis Reference handle
/// \param  pInParam - [IN] input parameters structure for Read Actual Velocity Command
/// \param  pOutParam - [OUT] output parameters structure for Read Actual Velocity Command
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ReadActualVelocityCmd(
        IN MMC_CONNECT_HNDL hConn,
        IN MMC_AXIS_REF_HNDL hAxisRef,
        IN MMC_READACTUALVELOCITY_IN* pInParam,
        OUT MMC_READACTUALVELOCITY_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_HaltCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_HALT_IN* pInParam,
///             OUT MMC_HALT_OUT* pOutParam)
/// \brief This function  send Halt command to MMC server for specific Axis.
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis Reference handle
/// \param  pInParam - [IN] Pointer to Halt input parameters
/// \param  pOutParam - [OUT] Pointer to Halt output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_HaltCmd(
        IN MMC_CONNECT_HNDL hConn,
        IN MMC_AXIS_REF_HNDL hAxisRef,
        IN MMC_HALT_IN* pInParam,
        OUT MMC_HALT_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_StopCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_STOP_IN* pInParam,
///             OUT MMC_STOP_OUT* pOutParam)
/// \brief This function  send Stop command to MMC server for specific Axis.
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis Reference handle
/// \param  pInParam - [IN] Pointer to Stop input parameters
/// \param  pOutParam - [OUT] Pointer to Stop output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_StopCmd(
        IN MMC_CONNECT_HNDL hConn,
        IN MMC_AXIS_REF_HNDL hAxisRef,
        IN MMC_STOP_IN* pInParam,
        OUT MMC_STOP_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ReadDigitalInputCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_READDIGITALINPUT_IN* pInParam,
///             OUT MMC_READDIGITALINPUT_OUT* pOutParam)
/// \brief This function  return value of specific bit digital input.
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis Reference handle
/// \param  pInParam - [IN] Pointer to Read Digital Input input parameters
/// \param  pOutParam - [OUT] Pointer to Read Digital Input output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ReadDigitalInputCmd(
        IN MMC_CONNECT_HNDL hConn,
        IN MMC_AXIS_REF_HNDL hAxisRef,
        IN MMC_READDIGITALINPUT_IN* pInParam,
        OUT MMC_READDIGITALINPUT_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ReadDigitalInputsCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_READDIGITALINPUTS_IN* pInParam,
///             OUT MMC_READDIGITALINPUTS_OUT* pOutParam)
/// \brief This function  return value of specific bit digital input.
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis Reference handle
/// \param  pInParam - [IN] Pointer to Read Digital Input input parameters
/// \param  pOutParam - [OUT] Pointer to Read Digital Input output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ReadDigitalInputsCmd(
        IN MMC_CONNECT_HNDL hConn,
        IN MMC_AXIS_REF_HNDL hAxisRef,
        IN MMC_READDIGITALINPUTS_IN* pInParam,
        OUT MMC_READDIGITALINPUTS_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_SetPositionCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_SETPOSITION_IN* pInParam,
///             OUT MMC_SETPOSITION_OUT* pOutParam)
/// \brief This function  send Set Position command to MMC server for specific Axis.
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis Reference handle
/// \param  pInParam - [IN] Pointer to Set Position input parameters
/// \param  pOutParam - [OUT] Pointer to Set POsition output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_SetPositionCmd(
        IN MMC_CONNECT_HNDL hConn,
        IN MMC_AXIS_REF_HNDL hAxisRef,
        IN MMC_SETPOSITION_IN* pInParam,
        OUT MMC_SETPOSITION_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_SetOverrideCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_SETOVERRIDE_IN* pInParam,
///             OUT MMC_SETOVERRIDE_OUT* pOutParam)
/// \brief This function  send Set Override command to MMC server for specific Axis.
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis Reference handle
/// \param  pInParam - [IN] Pointer to Set Override input parameters
/// \param  pOutParam - [OUT] Pointer to Set Override output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_SetOverrideCmd(
        IN MMC_CONNECT_HNDL hConn,
        IN MMC_AXIS_REF_HNDL hAxisRef,
        IN MMC_SETOVERRIDE_IN* pInParam,
        OUT MMC_SETOVERRIDE_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ReadStatusCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_READSTATUS_IN* pInParam,
///             OUT MMC_READSTATUS_OUT* pOutParam)
/// \brief This function  send Read Axis Status command to MMC server for specific Axis
/// and receive Axis status back if it's available.
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis Reference handle
/// \param  pInParam - [IN] Pointer to Axis Status input parameters structure
/// \param  pOutParam - [OUT] Axis Status output parameters structure
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ReadStatusCmd(
        IN MMC_CONNECT_HNDL hConn,
        IN MMC_AXIS_REF_HNDL hAxisRef,
        IN MMC_READSTATUS_IN* pInParam,
        OUT MMC_READSTATUS_OUT* pOutParam);


////////////////////////////////////////////////////////////////////////////////
/// \fn int   MMC_ReadParameter(
///                 IN MMC_CONNECT_HNDL hConn,
///                 IN MMC_AXIS_REF_HNDL hAxisRef,
///                 IN MMC_READPARAMETER_IN* pInParam,
///                 OUT MMC_READPARAMETER_OUT* pOutParam)
/// \brief This function read specific axis parameter.
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis Reference handle
/// \param  pInParam - [IN] Pointer to Read Parameter input parameters
/// \param  pOutParam - [OUT] Pointer to Read Parameter output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ReadParameter(
        IN MMC_CONNECT_HNDL hConn,
        IN MMC_AXIS_REF_HNDL hAxisRef,
        IN MMC_READPARAMETER_IN* pInParam,
        OUT MMC_READPARAMETER_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int   MMC_ReadBoolParameter(
///                 IN MMC_CONNECT_HNDL hConn,
///                 IN MMC_AXIS_REF_HNDL hAxisRef,
///                 IN MMC_READBOOLPARAMETER_IN* pInParam,
///                 OUT MMC_READBOOLPARAMETER_OUT* pOutParam)
/// \brief This function read boolean specific axis parameter.
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis Reference handle
/// \param  pInParam - [IN] Pointer to Read Boolean Parameter input parameters
/// \param  pOutParam - [OUT] Pointer to Read Boolean Parameter output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ReadBoolParameter(
        IN MMC_CONNECT_HNDL hConn,
        IN MMC_AXIS_REF_HNDL hAxisRef,
        IN MMC_READBOOLPARAMETER_IN* pInParam,
        OUT MMC_READBOOLPARAMETER_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int   MMC_WriteParameter(
///                 IN MMC_CONNECT_HNDL hConn,
///                 IN MMC_AXIS_REF_HNDL hAxisRef,
///                 IN MMC_WRITEPARAMETER_IN* pInParam,
///                 OUT MMC_WRITEPARAMETER_OUT* pOutParam)
/// \brief This function write specific axis parameter.
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis Reference handle
/// \param  pInParam - [IN] Pointer to Write Parameter input parameters
/// \param  pOutParam - [OUT] Pointer to Write Parameter output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_WriteParameter(
        IN MMC_CONNECT_HNDL hConn,
        IN MMC_AXIS_REF_HNDL hAxisRef,
        IN MMC_WRITEPARAMETER_IN* pInParam,
        OUT MMC_WRITEPARAMETER_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int   MMC_WriteBoolParameter(
///                 IN MMC_CONNECT_HNDL hConn,
///                 IN MMC_AXIS_REF_HNDL hAxisRef,
///                 IN MMC_WRITEBOOLPARAMETER_IN* pInParam,
///                 OUT MMC_WRITEBOOLPARAMETER_OUT* pOutParam)
/// \brief This function write boolean specific axis parameter.
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis Reference handle
/// \param  pInParam - [IN] Pointer to Write Boolean Parameter input parameters
/// \param  pOutParam - [OUT] Pointer to Write Boolean Parameter output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_WriteBoolParameter(
        IN MMC_CONNECT_HNDL hConn,
        IN MMC_AXIS_REF_HNDL hAxisRef,
        IN MMC_WRITEBOOLPARAMETER_IN* pInParam,
        OUT MMC_WRITEBOOLPARAMETER_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_Reset(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_RESET_IN* pInParam,
///             OUT MMC_RESET_OUT* pOutParam)
/// \brief This function reset axis.
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis Reference handle
/// \param  pInParam - [IN] Pointer to Reset input parameters
/// \param  pOutParam - [OUT] Pointer to Reset output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_Reset(
        IN MMC_CONNECT_HNDL hConn,
        IN MMC_AXIS_REF_HNDL hAxisRef,
        IN MMC_RESET_IN* pInParam,
        OUT MMC_RESET_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ResetAsync(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_RESET_IN* pInParam,
///             OUT MMC_RESET_OUT* pOutParam)
/// \brief This function reset axis Asynchronously.
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis Reference handle
/// \param  pInParam - [IN] Pointer to Reset input parameters
/// \param  pOutParam - [OUT] Pointer to Reset output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ResetAsync(
        IN MMC_CONNECT_HNDL hConn,
        IN MMC_AXIS_REF_HNDL hAxisRef,
        IN MMC_RESET_IN* pInParam,
        OUT MMC_RESET_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_PowerCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_POWER_IN* pInParam,
///             OUT MMC_POWER_OUT* pOutParam)
/// \brief This function send Power On command to MMC server.
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis Reference handle
/// \param  pInParam - [IN] Pointer to Power input parameters
/// \param  pOutParam - [OUT] Pointer to Power output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_PowerCmd(
        IN  MMC_CONNECT_HNDL    hConn,
        IN  MMC_AXIS_REF_HNDL   hAxisRef,
        IN  MMC_POWER_IN*       pInParam,
        OUT MMC_POWER_OUT*      pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_GetFbDepthCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_GETFBDEPTH_IN* pInParam,
///             OUT MMC_GETFBDEPTH_OUT* pOutParam)
/// \brief This function sends a command for receive number of function blocks in Node Queue - ONLY THOSE WHICH
///        did not receive DONE/ABORTED status!!!
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis Reference handle
/// \param  pInParam - [IN] Pointer to Get F.B. Depth input parameters
/// \param  pOutParam - [OUT] Pointer to Get F.B. Depth output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_GetFbDepthCmd(
        IN  MMC_CONNECT_HNDL    hConn,
        IN  MMC_AXIS_REF_HNDL   hAxisRef,
        IN  MMC_GETFBDEPTH_IN*  pInParam,
        OUT MMC_GETFBDEPTH_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_GetTotalFbDepthCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_GETFBDEPTH_IN* pInParam,
///             OUT MMC_GETFBDEPTH_OUT* pOutParam)
/// \brief This function sends a command for receive number of function blocks in Node Queue .
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis Reference handle
/// \param  pInParam - [IN] Pointer to Get F.B. Depth input parameters
/// \param  pOutParam - [OUT] Pointer to Get F.B. Depth output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_GetTotalFbDepthCmd(
        IN  MMC_CONNECT_HNDL    hConn,
        IN  MMC_AXIS_REF_HNDL   hAxisRef,
        IN  MMC_GETFBDEPTH_IN*  pInParam,
        OUT MMC_GETFBDEPTH_OUT* pOutParam);
////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ReadActualTorqueCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_READACTUALTORQUE_IN* pInParam,
///             OUT MMC_READACTUALTORQUE_OUT* pOutParam)
/// \brief The function returns the actual torque for specific Node.
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis Reference handle
/// \param  pInParam - [IN] Pointer to Read Actual Torque input parameters
/// \param  pOutParam - [OUT] Pointer to Read Actual Torque output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ReadActualTorqueCmd(
        IN  MMC_CONNECT_HNDL            hConn,
        IN  MMC_AXIS_REF_HNDL           hAxisRef,
        IN  MMC_READACTUALTORQUE_IN*    pInParam,
        OUT MMC_READACTUALTORQUE_OUT*   pOutParam) ;

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ReadDigitalOutputs(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_READDIGITALOUTPUT_IN* pInParam,
///             OUT MMC_READDIGITALOUTPUT_OUT* pOutParam)
/// \brief The function returns the actual Digital Output for specific Node.
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis Reference handle
/// \param  pInParam - [IN]     Pointer to Read Digital Output input parameters
/// \param  pOutParam - [OUT]   Pointer to Read Digital Output output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ReadDigitalOutputs(
        IN  MMC_CONNECT_HNDL            hConn,
        IN  MMC_AXIS_REF_HNDL           hAxisRef,
        IN  MMC_READDIGITALOUTPUT_IN*   pInParam,
        OUT MMC_READDIGITALOUTPUT_OUT*  pOutParam) ;

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_WriteDigitalOutputs(
/// IN MMC_CONNECT_HNDL hConn,
/// IN MMC_AXIS_REF_HNDL hAxisRef,
/// IN MMC_WRITEDIGITALOUTPUT_IN* pInParam,
/// OUT MMC_WRITEDIGITALOUTPUT_OUT* pOutParam)
/// \brief The function sets the actual Digital Output for specific Node.
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis Reference handle
/// \param  pInParam - [IN]     Pointer to Set Digital Output input parameters
/// \param  pOutParam - [OUT]   Pointer to Set Digital Output output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_WriteDigitalOutputs(
        IN  MMC_CONNECT_HNDL            hConn,
        IN  MMC_AXIS_REF_HNDL           hAxisRef,
        IN  MMC_WRITEDIGITALOUTPUT_IN*  pInParam,
        OUT MMC_WRITEDIGITALOUTPUT_OUT* pOutParam) ;

///
////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ReadDigitalOutputs32Bit(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_READDIGITALOUTPUT_IN* pInParam,
///             OUT MMC_READDIGITALOUTPUT_OUT* pOutParam)
/// \brief The function returns the actual Digital Output for specific Node.
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis Reference handle
/// \param  pInParam - [IN]     Pointer to Read Digital Output input parameters
/// \param  pOutParam - [OUT]   Pointer to Read Digital Output output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ReadDigitalOutputs32Bit(
        IN  MMC_CONNECT_HNDL                hConn,
        IN  MMC_AXIS_REF_HNDL               hAxisRef,
        IN  MMC_READDIGITALOUTPUT32Bit_IN*  pInParam,
        OUT MMC_READDIGITALOUTPUT32Bit_OUT* pOutParam) ;

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_WriteDigitalOutputs32Bit(
/// IN MMC_CONNECT_HNDL hConn,
/// IN MMC_AXIS_REF_HNDL hAxisRef,
/// IN MMC_WRITEDIGITALOUTPUT_IN* pInParam,
/// OUT MMC_WRITEDIGITALOUTPUT_OUT* pOutParam)
/// \brief The function sets the actual Digital Output for specific Node.
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis Reference handle
/// \param  pInParam - [IN]     Pointer to Set Digital Output input parameters
/// \param  pOutParam - [OUT]   Pointer to Set Digital Output output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_WriteDigitalOutputs32Bit(
        IN  MMC_CONNECT_HNDL                    hConn,
        IN  MMC_AXIS_REF_HNDL                   hAxisRef,
        IN  MMC_WRITEDIGITALOUTPUT32Bit_IN*     pInParam,
        OUT MMC_WRITEDIGITALOUTPUT32Bit_OUT*    pOutParam) ;

/* Alex - Integration of MoveRepetitive()*/
///////////////////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_MoveAbsoluteRepetitiveCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_MOVEABSOLUTEREPETITIVE_IN* pInParam,
///             OUT MMC_MOVEABSOLUTEREPETITIVE_OUT* pOutParam)
/// \brief This function send Move Absolute Repetitive command to MMC server for specific Axis.
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis Reference handle
/// \param  pInParam - [IN] Pointer to Move Absolute Repetitive input parameters
/// \param  pOutParam - [OUT] Pointer to Move Absolute Repetitive output parameters
/// \return return - 0 if success
///                  error_id in case of error
/////////////////////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_MoveAbsoluteRepetitiveCmd(IN MMC_CONNECT_HNDL hConn,
        IN  MMC_AXIS_REF_HNDL               hAxisRef,
        IN  MMC_MOVEABSOLUTEREPETITIVE_IN*  pInParam,
        OUT MMC_MOVEABSOLUTEREPETITIVE_OUT* pOutParam);

MMC_LIB_API ELMO_INT32 MMC_MoveRelativeRepetitiveCmd(IN MMC_CONNECT_HNDL hConn,
        IN MMC_AXIS_REF_HNDL                hAxisRef,
        IN MMC_MOVERELATIVEREPETITIVE_IN*   pInParam,
        OUT MMC_MOVERELATIVEREPETITIVE_OUT* pOutParam);

MMC_LIB_API ELMO_INT32 MMC_MoveAdditiveRepetitiveCmd(IN MMC_CONNECT_HNDL hConn,
        IN  MMC_AXIS_REF_HNDL               hAxisRef,
        IN  MMC_MOVEADDITIVEREPETITIVE_IN*  pInParam,
        OUT MMC_MOVEADDITIVEREPETITIVE_OUT* pOutParam);

MMC_LIB_API ELMO_INT32 MMC_PositionProfileCmd(IN MMC_CONNECT_HNDL hConn,
        IN  MMC_AXIS_REF_HNDL           hAxisRef,
        IN  MMC_POSITIONPROFILE_IN*     pInParam,
        OUT MMC_POSITIONPROFILE_OUT*    pOutParam);

MMC_LIB_API ELMO_INT32 MMC_MoveAbsoluteExCmd(IN MMC_CONNECT_HNDL hConn,
        IN  MMC_AXIS_REF_HNDL       hAxisRef,
        IN  MMC_MOVEABSOLUTEEX_IN*  pInParam,
        OUT MMC_MOVEABSOLUTEEX_OUT* pOutParam);

MMC_LIB_API ELMO_INT32 MMC_MoveRelativeExCmd(IN MMC_CONNECT_HNDL hConn,
        IN  MMC_AXIS_REF_HNDL       hAxisRef,
        IN  MMC_MOVERELATIVEEX_IN*  pInParam,
        OUT MMC_MOVERELATIVEEX_OUT* pOutParam);

MMC_LIB_API ELMO_INT32 MMC_MoveAdditiveExCmd(IN MMC_CONNECT_HNDL hConn,
        IN  MMC_AXIS_REF_HNDL       hAxisRef,
        IN  MMC_MOVEADDITIVEEX_IN*  pInParam,
        OUT MMC_MOVEADDITIVEEX_OUT* pOutParam);

MMC_LIB_API ELMO_INT32 MMC_MoveVelocityExCmd(IN MMC_CONNECT_HNDL hConn,
        IN  MMC_AXIS_REF_HNDL       hAxisRef,
        IN  MMC_MOVEVELOCITYEX_IN*  pInParam,
        OUT MMC_MOVEVELOCITYEX_OUT* pOutParam);

MMC_LIB_API ELMO_INT32 MMC_MoveAbsoluteRepetitiveExCmd(IN MMC_CONNECT_HNDL hConn,
        IN  MMC_AXIS_REF_HNDL                   hAxisRef,
        IN  MMC_MOVEABSOLUTEREPETITIVEEX_IN*    pInParam,
        OUT MMC_MOVEABSOLUTEREPETITIVEEX_OUT*   pOutParam);
MMC_LIB_API ELMO_INT32 MMC_MoveRelativeRepetitiveExCmd(IN MMC_CONNECT_HNDL hConn,
        IN  MMC_AXIS_REF_HNDL                   hAxisRef,
        IN  MMC_MOVERELATIVEREPETITIVEEX_IN*    pInParam,
        OUT MMC_MOVERELATIVEREPETITIVEEX_OUT*   pOutParam);

MMC_LIB_API ELMO_INT32 MMC_MoveAdditiveRepetitiveExCmd(IN MMC_CONNECT_HNDL hConn,
        IN  MMC_AXIS_REF_HNDL                   hAxisRef,
        IN  MMC_MOVEADDITIVEREPETITIVEEX_IN*    pInParam,
        OUT MMC_MOVEADDITIVEREPETITIVEEX_OUT*   pOutParam);
////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_TouchProbeEnable(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_TOUCHPROBEENABLE_IN* pInParam,
///             OUT MMC_TOUCHPROBEENABLE_OUT* pOutParam)
/// \brief This function ....
/// \param hConn - [IN] Connection handle
/// \param hAxisRef - [IN] Axis Reference handle
/// \param pInParam - [IN] Pointer to Touch Proble Enable input parameters
/// \param pOutParam - [OUT] Pointer to Touch Proble Enable output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_TouchProbeEnable(
        IN  MMC_CONNECT_HNDL            hConn,
        IN  MMC_AXIS_REF_HNDL           hAxisRef,
        IN  MMC_TOUCHPROBEENABLE_IN*    pInParam,
        OUT MMC_TOUCHPROBEENABLE_OUT*   pOutParam) ;

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_TouchProbeDisable(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_TOUCHPROBEENABLE_IN* pInParam,
///             OUT MMC_TOUCHPROBEENABLE_OUT* pOutParam)
/// \brief This function ....
/// \param hConn - [IN] Connection handle
/// \param hAxisRef - [IN] Axis Reference handle
/// \param pInParam - [IN] Pointer to Touch Proble Disable input parameters
/// \param pOutParam - [OUT] Pointer to Touch Proble Disable output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_TouchProbeDisable(
        IN  MMC_CONNECT_HNDL            hConn,
        IN  MMC_AXIS_REF_HNDL           hAxisRef,
        IN  MMC_TOUCHPROBEDISABLE_IN*   pInParam,
        OUT MMC_TOUCHPROBEDISABLE_OUT*  pOutParam) ;

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_AxisLink(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_AXISLINK_IN* pInParam,
///             OUT MMC_AXISLINK_OUT* pOutParam)
/// \brief This function link between two axes
/// \param hConn - [IN] Connection handle
/// \param hAxisRef - [IN] Master Axis Reference handle
/// \param pInParam - [IN] Pointer to AxisLink input parameters
/// \param pOutParam - [OUT] Pointer to AxisLink output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_AxisLink(
        IN  MMC_CONNECT_HNDL    hConn,
        IN  MMC_AXIS_REF_HNDL   hAxisRef,
        IN  MMC_AXISLINK_IN*    pInParam,
        OUT MMC_AXISLINK_OUT*   pOutParam) ;

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_AxisUnLink(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_AXISUNLINK_IN* pInParam,
///             OUT MMC_AXISUNLINK_OUT* pOutParam)
/// \brief This function unlink the master axis from the slave
/// \param hConn - [IN] Connection handle
/// \param hAxisRef - [IN] Master Axis Reference handle
/// \param pInParam - [IN] Pointer to AxisUnLink input parameters
/// \param pOutParam - [OUT] Pointer to AxisUnLink output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_AxisUnLink(
        IN  MMC_CONNECT_HNDL    hConn,
        IN  MMC_AXIS_REF_HNDL   hAxisRef,
        IN  MMC_AXISUNLINK_IN*  pInParam,
        OUT MMC_AXISUNLINK_OUT* pOutParam) ;


////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ChangeOpModeEx(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_CHANGEOPMODE_EX_IN* pInParam,
///             OUT MMC_CHANGEOPMODE_EX_OUT* pOutParam)
/// \brief The function change the motion mode.
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis Reference handle
/// \param  pInParam - [IN] Pointer to input parameters
/// \param  pOutParam - [OUT] Pointer to output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ChangeOpModeEx(
        IN  MMC_CONNECT_HNDL            hConn,
        IN  MMC_AXIS_REF_HNDL           hAxisRef,
        IN  MMC_CHANGEOPMODE_EX_IN*     pInParam,
        OUT MMC_CHANGEOPMODE_EX_OUT*    pOutParam);



////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_MoveTorqueCmd(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_MOVETORQUE_IN* pInParam,
///             OUT MMC_MOVETORQUE_OUT* pOutParam)
/// \brief This function send Move torque command to MMC server for specific Axis.
/// \param  hConn - [IN] Connection handle
/// \param  hAxisRef - [IN] Axis Reference handle
/// \param  pInParam - [IN] Pointer to Move Torque input parameters
/// \param  pOutParam - [OUT] Pointer to Move Torque output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_MoveTorqueCmd(
        IN  MMC_CONNECT_HNDL    hConn,
        IN  MMC_AXIS_REF_HNDL   hAxisRef,
        IN  MMC_MOVETORQUE_IN*  pInParam,
        OUT MMC_MOVETORQUE_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_RegErrPolicy(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_REGERRPOLICY_IN* pInParam,
///             OUT MMC_REGERRPOLICY_OUT* pOutParam)
/// \brief This function registers an error policy for a specific Axis.
/// \param  hConn - [IN] Connection handle
/// \param  pInParam - [IN] Pointer to register error policy input parameters
/// \param  pOutParam - [OUT] Pointer to register error policy output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_RegErrPolicy(
        IN MMC_CONNECT_HNDL hConn,
        IN MMC_REGERRPOLICY_IN* pInParam,
        OUT MMC_REGERRPOLICY_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_ResetSystem(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_RESETSYSTEM_IN* pInParam,
///             OUT MMC_RESETSYSTEM_OUT* pOutParam)
/// \brief This function resets system errors and restarts the system errors monitoring
/// \param hConn - [IN] Connection handle
/// \param pInParam - [IN] Pointer to reset system errors input parameters
/// \param pOutParam - [OUT] Pointer to system errors output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_ResetSystem(
        IN MMC_CONNECT_HNDL hConn,
        IN MMC_RESETSYSTEM_IN* pInParam,
        OUT MMC_RESETSYSTEM_OUT* pOutParam);

////////////////////////////////////////////////////////////////////////////////
/// \fn int MMC_GetErrPolicy(
///             IN MMC_CONNECT_HNDL hConn,
///             IN MMC_AXIS_REF_HNDL hAxisRef,
///             IN MMC_GETERRPOLICY_IN* pInParam,
///             OUT MMC_GETERRPOLICY_OUT* pOutParam)
/// \brief This function get an error policy for a specific Axis.
/// \param  hConn - [IN] Connection handle
/// \param  pInParam - [IN] Pointer to get error policy input parameters
/// \param  pOutParam - [OUT] Pointer to get error policy output parameters
/// \return return - 0 if success
///                  error_id in case of error
////////////////////////////////////////////////////////////////////////////////
MMC_LIB_API ELMO_INT32 MMC_GetErrPolicy(
        IN MMC_CONNECT_HNDL hConn,
        IN MMC_GETERRPOLICY_IN* pInParam,
        OUT MMC_GETERRPOLICY_OUT* pOutParam);


//moving to g++ @YL 4-10-2010
#ifdef __cplusplus
}
#endif

#endif /* MMC_PLCOPEN_SINGLE_API_H */
