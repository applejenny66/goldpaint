////////////////////////////////////////////////////////////////////////////////
/// file os_platformVxWorks
/// Name        : OS_platformVxWorks.h
/// Author      : Haim Hillel
/// Created on  : 07Feb2013
/// Version     : 0.0.1
///               0.2.0 Updated 20Jan2015 Haim H. native Data Types (names), for supporting 64B OS.
///				  0.4.0 Updated 11Sep2017 Haim H.
/// Copyright   : Your copyright notice
/// Description : This file contain GMAS RPC VxWorks Operating System  Platform dependencies.
///               The file is selected to be including in OS_Platform.h in case target select to be VxWorks
////////////////////////////////////////////////////////////////////////////////

#ifndef OS_PLATFORMVXWORKS_H_
#define OS_PLATFORMVXWORKS_H_ 

    #include <VxWorks.h>
    #include <taskLib.h>
    #include <pthread.h>
    
    #include <sockLib.h>
    #include <socket.h>
    #include <selectLib.h>
    
    #include <netLib.h>
    #include <netinet\tcp.h>
    #include <netinet\in.h>
    #include <inetLib.h>
    
    #include <semLib.h> 
    #include <semaphore.h>
    
    #include <taskLib.h>
    #include <tickLib.h>
    
    #include <time.h>
    #include <errno.h>
    #include <stdio.h>
    
    #include <hostLib.h>
    #include <sysLib.h>


    #define MMC_LIB_API
    #define OS_SIGNL_EVENT_IMPL     SEM_ID
    #define OS_CRITIC_SEC_IMPL      SEM_ID
    #define OS_PLATFORM_SINGOBJ_TO  ERROR
    #define OS_PlatformTermiEvent   SEM_ID
    
            /* Define Sokcet fun return code symbols behaver as in VxWorks */
    #define OS_PLATFORM_INVALID_SOCKET  ERROR
    #define OS_PLATFORM_SOCKET_ERROR    ERROR



        typedef struct 
            {
            ELM_INT4U       ulliVal;
            ELMO_UINT16      usStatus;
            ELMO_INT16       usErrorID;
            }MMC_GENERALPARAMPDOREAD_OUT;

        typedef union _unGeneralPDOWriteData
            {
            ELMO_UINT8       pData[8];
            ELMO_UINT64      ulliVal;
            }unGeneralPDOWriteData;

        typedef struct
            {
            ELMO_UINT64      ulliDI;
            ELMO_UINT16      usStatus;
            ELMO_INT16       usErrorID;
            }MMC_READDI_OUT;

        typedef struct
            {
            ELMO_UINT64      ulliDO;
            }MMC_WRITEDO_IN;


#include "datastructs.h"

    

///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_IPC_SHM_IN_STRUCT
/// \brief IPC host to target transaction data structure
///
/// This structure defines host to target shared memory transaction for IPC.
///////////////////////////////////////////////////////////////////////////////
typedef struct shared_memory_region1
{
    MMC_IPC_BASIC_IN_STRUCT basic_in ;
    ELMO_UINT8              in_param[IPC_MSG_IN_OUT_ARGS_MAX_SIZE];
    MOTION_FUNCS_INNER_ARGS inner_args;
}LINUX_ALIGNMENT MMC_IPC_SHM_IN_STRUCT;


///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_IPC_SHM_OUT_STRUCT
/// \brief IPC target to host transaction data structure
///////////////////////////////////////////////////////////////////////////////
typedef struct shared_memory_out_region1
{
    ELMO_UINT16              numerator_id;
    ELMO_UINT16              data_size;
    ELMO_UINT16              dummy[2];
    ELMO_UINT8               out_param[IPC_MSG_IN_OUT_ARGS_MAX_SIZE];
    MOTION_FUNCS_INNER_ARGS inner_args;
}LINUX_ALIGNMENT MMC_IPC_SHM_OUT_STRUCT;


///////////////////////////////////////////////////////////////////////////////
/// \struct MMC_IF_STRUCT
/// \brief Connection interface data structure
///////////////////////////////////////////////////////////////////////////////
typedef struct mmc_if_struct
    {
        ELMO_UINT32          magic;
        MMC_CONNECTION_TYPE conn_type;
        MMC_CONNECTION_PARAM_STRUCT conn_param;

        ELMO_DOUBLE          dData[190] ;
        ELMO_INT32           blocksock ;
        ELMO_INT32           sockListen ;
        ELMO_ULINT32         tID ;
        ELMO_INT32           udpport ;

        ELMO_INT32           hConnectThread ;
        OS_SIGNL_EVENT_IMPL hTerminateEvent ;

        ELMO_INT32           iActiveAxesNum;
        MMC_UDP_DATA_FIFO_STRUCT* stUDPDataFIFO[MMC_MAX_NUM_AXIS] ;

        OS_CRITIC_SEC_IMPL  lpCriticalSectionUDP ;
        OS_CRITIC_SEC_IMPL  lpCriticalSectionSendRcv;

        MMC_CB_FUNC         pCbFunc ;
        MMC_CB_FUNC         pCbFuncSecondary ;
        ELMO_INT32           ipc_conn_num;
        ELMO_UINT32          msg_numerator;
        ELMO_PVOID           snd_msg;
                                                                /* GMAS Endian type relating to the Cpu running */
                                                                /* user program => SAME OR DIFFERENT ENDIAN     */
                                                                /* Use in RPC for direct whether GMAS bytes in  */
                                                                /* sensitive to endian should swap.             */
                                                                /* Put also in IPC - for be similar to RPC...   */
        eGMAS_ENDIAN_TYPE   eGmasEndianType;
        ELMO_INT8            cLastError[512] ;
        ELMO_UINT8           ucSendAsyncEventFlag ;
		ELMO_UINT8 			ucDeviceType;
    } LINUX_ALIGNMENT       MMC_IF_STRUCT;

    ELMO_INT32 MMC_DestroySYNCTimer(IN MMC_CONNECT_HNDL hConn);
    ELMO_INT32 MMC_CreateSYNCTimer (IN MMC_CONNECT_HNDL hConn, IN MMC_SYNC_TIMER_CB_FUNC func, IN ELMO_UINT16 usSYNCTimerTime);

    #include "blocksock.h"


        /* OS_PLATFORMVXWORKS_H_  */
#endif
