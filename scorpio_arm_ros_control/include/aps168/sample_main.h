#ifndef _SAMPLE_MAIN_H
#define _SAMPLE_MAIN_H

#include "aps168/APS168.h"

// version array index
#define IDX_DLL      (0)
#define IDX_DRIVER   (1)
#define IDX_KERNEL   (2)
#define IDX_FIRMWARE (3)
#define IDX_PCB      (4)
#define IDX_MAX      (5)


typedef struct
{
	F64 command_position;
	F64 feedback_position;
	F64 target_position;
	F64 error_position;
	F64 command_velocity;
	F64 feedback_velocity;
	I32 motion_io;
	I32 motion_status;
}AXIS_STATUS;

void home_move_example( I32 Axis_ID );
int	 check_motion_done_example( I32 Axis_ID, I32 *Stop_Code );
const char *stop_code_to_string( I32 Stop_Code );
void emg_stop_example( I32 Axis_ID );
void jog_move_continuous_mode_example( I32 Axis_ID );
void jog_move_step_mode_example( I32 Axis_ID );
void p2p_example( I32 Axis_ID );
void point_table_2D_example( I32 Board_ID, I32 *Axis_ID_Array );
void interpolation_2D_line_example( I32 *Axis_ID_Array );


int PT_Samp_ThreadFunc(void);
int PVT_Test_ThreadFunc(void);
int PVT_Samp_ThreadFunc(void);
int PT_static_nosamp_test(void);
int PVT_static_nosamp_test(void);

int PT_addpoint_dynamic(void);
int PVT_addpoint_dynamic(void);
int getpos_PT(void);
int getpos_PVT(void);

#endif
