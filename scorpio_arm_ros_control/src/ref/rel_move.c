#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>


#include <sys/ioctl.h>
#include <sys/user.h>
#include <sys/time.h>

#include "../include/type_def.h"
#include "../include/APS168.h"
#include "../include/ErrorCodeDef.h"
#include "../include/APS_Define.h"

#include "../include/sample_main.h"
#include "../include/APS_Linux_sample.h"

#define GetBit(x,m)				((x)>>(m) & 1)
#define DIM 2
#define OPT_ABS 0

I32 chk_initial = 0;

typedef struct {
    I32*    AxisArr;
    I32     Dimension;
}PosLogInfo;

#if 0
typedef struct {
    I32*    AxisArr;
    I32     Dimension;
}AxisSet;
#endif

void LogTorque(void *loginfo)
{
    I32 i, ret, mts_mdn = 0, dimension;
    I32 tq;

    dimension = ((PosLogInfo*)loginfo)->Dimension;
    I32 *axisarr = ((PosLogInfo*)loginfo)->AxisArr;
    

    //  create a thread to log & printf position
    //  thread would terminate by major function
    if(dimension < 1)
    {
        printf("Dimension set < 1, error");
        return;
    }

    while( !( ( APS_motion_io_status( axisarr[0] ) >> MIO_SVON ) & 1 ) )
    {
    }
        
    usleep(1000000);


    while(1)
    {
        mts_mdn = 0;
        for(i=0;i < dimension; i++)
        {
            ret = APS_get_actual_torque(axisarr[i],&tq);
            if(ret != 0)
            {
                printf("log position error, return!!\n");
                pthread_exit(0);
                return;
            }
            printf("Axis %d torque: %d   ",i, tq);         
            mts_mdn += APS_motion_status(i)&0x20;
        }
        printf("\n");
        usleep(1000000);

		if(mts_mdn == (((PosLogInfo*)loginfo)->Dimension)*0x20)
        {           
            break;
        }       
    }

    pthread_exit(0);
}

void LogPos(void *loginfo)
{
    I32 i, ret, mts_mdn = 0, dimension;
    F64 pos;

    dimension = ((PosLogInfo*)loginfo)->Dimension;
    I32 *axisarr = ((PosLogInfo*)loginfo)->AxisArr;
    

    //  create a thread to log & printf position
    //  thread would terminate by major function
    if(dimension < 1)
    {
        printf("Dimension set < 1, error");
        return;
    }

    while( !( ( APS_motion_io_status( axisarr[0] ) >> MIO_SVON ) & 1 ) )
    {
    }
        
    usleep(1000000);


    while(1)
    {
        mts_mdn = 0;
        for(i=0;i < dimension; i++)
        {
            ret = APS_get_position_f(axisarr[i],&pos);
            if(ret != 0)
            {
                printf("log position error, return!!\n");
                pthread_exit(0);
                return;
            }
            printf("Axis %d  : %f   ",i, pos);         
            mts_mdn += APS_motion_status(i)&0x20;
        }
        printf("\n");
        usleep(1000000);

		if(mts_mdn == (((PosLogInfo*)loginfo)->Dimension)*0x20)
        {           
            break;
        }       
    }
    //  
    pthread_exit(0);
}
void Initial_StartFieldBus(void)
{
    I32 ret=0, board_id=0;

    printf("Start Initial & start field bus...............\n");

    ret = APS_initial(&board_id,0);
    printf("Initial, ret=%d\n", ret);

    ret = APS_start_field_bus(0,0,0);
    printf("start field bus ret=%d\n", ret);

    if(ret!=0)
    {
        chk_initial = 0;
        printf("Initial_StartFieldBus failed......\n\n");
    }
    else{
        chk_initial = 1;
        printf("Initial_StartFieldBus success......\n\n");
    }
}


void P2PTest(void)
{
    //I32 AxisID = 0, ret;
    I32 ret;
    I32 AxisID[2] = {0, 1};
    pthread_t   torque_thread, pos_thread;
    PosLogInfo  *p2p_info   = (PosLogInfo*)malloc(sizeof(PosLogInfo));

    if(!chk_initial)
    {
        printf("No initial");
        return;
    }

    p2p_info->Dimension = 2;
    p2p_info->AxisArr = &AxisID;

    //  Linux create linux
    ret = pthread_create(&pos_thread, NULL, (void*)LogPos, (void*)p2p_info);
    ret = pthread_create(&torque_thread, NULL, (void*)LogTorque, (void*)p2p_info);
    if(ret!=0)
    {
        printf("Pthread log create occur error : %d", ret);
    }


    set_axis_param(0);
    set_axis_param(1);

	  ret = APS_ptp(0, OPT_RELATIVE, 10000000.0 , 0 );
	  ret = APS_ptp(1, OPT_RELATIVE, 10000000.0 , 0 );

#if 0
	  ret = APS_ptp(0, OPT_RELATIVE, 500000000.0 , 0 );
	  ret = APS_ptp(1, OPT_RELATIVE, 500000000.0 , 0 );
    sleep(5);

	  ret = APS_ptp(0, OPT_RELATIVE, -500000000.0 , 0 );
	  ret = APS_ptp(1, OPT_RELATIVE, -500000000.0 , 0 );
    sleep(5);
#endif

    ret = pthread_join(pos_thread,NULL);
    ret = pthread_join(torque_thread,NULL);

    if(ret!=0)
    {
        printf("Pthread log join occur error : %d", ret);
        return;
    }

    printf("P2P test ok !!!!!!!!!");
}

void set_axis_param( I32 Axis_ID )
{
	I32 ret = 0;
	
	// Config speed profile parameters.
	ret = APS_set_axis_param_f( Axis_ID, PRA_SF, 0.5      );
	ret = APS_set_axis_param_f( Axis_ID, PRA_ACC, 100000000.0 );
	ret = APS_set_axis_param_f( Axis_ID, PRA_DEC, 100000000.0 );
	ret = APS_set_axis_param_f( Axis_ID, PRA_VM,  1000000.0   );

#if 0
	ret = APS_set_axis_param_f( Axis_ID, PRA_SF, 0.5      );
	ret = APS_set_axis_param_f( Axis_ID, PRA_ACC, 100000000.0 );
	ret = APS_set_axis_param_f( Axis_ID, PRA_DEC, 100000000.0 );
	ret = APS_set_axis_param_f( Axis_ID, PRA_VM,  200000000.0   );
#endif

	//Check servo on or not
	if( !( ( APS_motion_io_status( Axis_ID ) >> MIO_SVON ) & 1 ) )  
	{
		ret = APS_set_servo_on( Axis_ID, 1 ); 
		ret = APS_set_command( Axis_ID, 0 );
		ret = APS_set_position( Axis_ID, 0 );

//		Sleep( 500 ); // Wait stable.
	}

	// Start a relative p to p move
	//ret = APS_ptp( Axis_ID, OPT_RELATIVE, 500000000.0 , 0 );
  //sleep(5);

	//ret = APS_ptp( Axis_ID, OPT_RELATIVE, -500000000.0 , 0 );
  //sleep(5);

	/*
	// Method 2, ptp move with maximum velocity.
	APS_ptp_v( 
		  Axis_ID  // I32 Axis_ID
		, OPT_RELATIVE // I32 Option
		, 500.0   // F64 Position
		, 1000.0  // F64 Vm
		, 0       // ASYNCALL *Wait
	);
	*/

	/*
	// Method 3, ptp move with all speed parameters.
	APS_ptp_all( 
		  Axis_ID  // I32 Axis_ID
		, OPT_RELATIVE // I32 Option
		, 500.0   // F64 Position
		, 0.0     // F64 Vs
		, 1000.0  // F64 Vm
		, 0.0     // F64 Ve
		, 10000.0 // F64 Acc
		, 10000.0 // F64 Dec
		, 0.5 // F64 SFac
		, 0   // ASYNCALL *Wait
	);
	*/
}

void axis_set_initial(I32* axis_set)
{
  I32 i, ret = 0, id;

  for(id = 0; id < DIM; id++)
  {
    // Config speed profile parameters.
    ret = APS_set_axis_param_f(id, PRA_SF, 0.5      );
    ret = APS_set_axis_param_f(id, PRA_ACC, 100000000.0 );
    ret = APS_set_axis_param_f(id, PRA_DEC, 100000000.0 );
    ret = APS_set_axis_param_f(id, PRA_VM,  200000000.0   );
    //ret = APS_set_axis_param_f(id, PRA_VM,  1000000.0   );

    // Servo on
    if( !( ( APS_motion_io_status(id) >> MIO_SVON ) & 1 ) )  
    {
      ret = APS_set_servo_on(id, 1); 
      //ret = APS_set_command(id, 0);
      //ret = APS_set_position( Axis_ID, 0 );
    }

  }
	
#if 0
	// Config speed profile parameters.
	ret = APS_set_axis_param_f( Axis_ID, PRA_SF, 0.5      );
	ret = APS_set_axis_param_f( Axis_ID, PRA_ACC, 100000000.0 );
	ret = APS_set_axis_param_f( Axis_ID, PRA_DEC, 100000000.0 );
	ret = APS_set_axis_param_f( Axis_ID, PRA_VM,  1000000.0   );

#if 0
	ret = APS_set_axis_param_f( Axis_ID, PRA_SF, 0.5      );
	ret = APS_set_axis_param_f( Axis_ID, PRA_ACC, 100000000.0 );
	ret = APS_set_axis_param_f( Axis_ID, PRA_DEC, 100000000.0 );
	ret = APS_set_axis_param_f( Axis_ID, PRA_VM,  200000000.0   );
#endif

	//Check servo on or not
	if( !( ( APS_motion_io_status( Axis_ID ) >> MIO_SVON ) & 1 ) )  
	{
		ret = APS_set_servo_on( Axis_ID, 1 ); 
		ret = APS_set_command( Axis_ID, 0 );
		ret = APS_set_position( Axis_ID, 0 );

//		Sleep( 500 ); // Wait stable.
	}

  for(i = 0; i < DIM; i++)
  {
    printf("axis %d\n", i);
  }
#endif
}

int main(int argc, char *argv[])
{
    // Initial axis according to DIM
    // If DIM = 3, the axis_set will be {0, 1, 2}
    I32 i = 0, ret = 0;
    I32 *axis_set = malloc(DIM * sizeof(I32));

    for(i = 0; i < DIM; i++)
      axis_set[i] = i;

    Initial_StartFieldBus();
    axis_set_initial(axis_set);

	  ret = APS_ptp(0, OPT_RELATIVE, 100000000.0 , 0 );
	  ret = APS_ptp(1, OPT_RELATIVE, 100000000.0 , 0 );

    sleep(2);

    ret = APS_stop_field_bus(0,0);
    printf("ret=%d\n", ret);

    ret = APS_close();
    printf("ret=%d\n", ret);

#if 0
    int ret = 0;

    P2PTest();

    printf("Closing........\n");
    
#endif

    return 0;
}

