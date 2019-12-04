/*
============================================================================
 Name : AdsAmsTcpProtocol.h
 Author : Haim H.
 Version :      14Jul2015
 Description :  Implement ADS / AMS Beckhoof protocol above Tcp.
                GMAS implementation for be Server responce to ADS client.
============================================================================
*/

#ifndef __AdsAmsTcpProtocol_HPP__
#define __AdsAmsTcpProtocol_HPP__

#if   (OS_PLATFORM == LINUXIPC32_PLATFORM)
                    /* From MMCUDP/MMCTCP ???? */
                    /* From MMCUDP/MMCTCP ???? */
	#include <arpa/inet.h>
	#include <netdb.h>
	#include <sys/types.h>
	#include <pthread.h>
	#include <sys/ioctl.h>
	#include <errno.h>
	#include <netinet/tcp.h>
	
	#include "MMCPPGlobal.hpp"
	#include "MMCSocket.hpp"
                    /* From MMCUDP/MMCTCP ???? */
                    
                    
    #define DEF_TCP_MSG_MAX_SIZE            512
    #define TCP_SND_BUF_SIZE                DEF_TCP_MSG_MAX_SIZE

                    /* ADS/AMS Structure OFFSETS (interesting item offset within AMS stream) */
	#define AMS_HDR_TCP_RSRV				0
    #define AMS_HDR_TCP_LEN                 2
    #define AMS_HDR_TCP                     6
    #define AMS_HDR_NETID_TRG               (AMS_HDR_TCP+ 0)
    #define AMS_HDR_NETID_SRC               (AMS_HDR_TCP+ 8)
    #define AMS_HDR_PORT_TRG                (AMS_HDR_TCP+ 6)
    #define AMS_HDR_PORT_SRC                (AMS_HDR_TCP+14)
    #define AMS_HDR_CMD                     (AMS_HDR_TCP+16)
    #define AMS_HDR_STATE                   (AMS_HDR_TCP+18)
    #define AMS_HDR_LEN                     (AMS_HDR_TCP+20)
    #define AMS_HDR_ERR_CODE                (AMS_HDR_TCP+24)
    #define AMS_HDR_INVOKE_ID               (AMS_HDR_TCP+28)
    
    #define AMSDATA_STR                     (AMS_HDR_TCP+32)
    
    #define AMSDATA_IDX_GRP                 (AMSDATA_STR+0)
    #define AMSDATA_IDX_OFST                (AMSDATA_STR+4)
    #define AMSDATA_REQ_LEN                 (AMSDATA_STR+8)
    
    #define AMSDATA_RESP_READ_LEN           (AMSDATA_STR+4)
    #define AMSDATA_RESP_READ_PURE_DATA     (AMSDATA_STR+8)
    
    #define AMSWRITE_DATA_STR               (AMSDATA_STR+12)

                    /* Define dump line length. Number of chars in one dump line (line length is more then  */
                    /* twice as: Space from begin of line, Hex, Space (betweens Hex to ascci) Asscii...     */
    #define ADS_AMS_BYTES_IN_DUMP_LINE      16
    

                    /* AMS ADS Command ID. Other commands are not defined or are used Beckhoff internally.  */
                    /* Therefore the Command Id  is only allowed to contain the above enumerated values!    */
                    /* Commad Id, type appearing both (exchange between) 'unsigned short'   */
                    /* and 'eCMDId' be carfull because enumerated type is 'int',            */
                    /* in ADS/AMS header the command appear in size of 'short'              */
    typedef enum CMDId_TAg
    {
        eCMDIdInvalid                    =0x0000,
        eCMDIdReadDeviceInfo             =0x0001,
        eCMDIdRead                       =0x0002,
        eCMDIdWrite                      =0x0003,
        eCMDIdReadState                  =0x0004,
        eCMDIdWriteControl               =0x0005,
        eCMDIdAddDeviceNotification      =0x0006,
        eCMDIdDeleteDeviceNotification   =0x0007,
        eCMDIdDeviceNotification         =0x0008,
        eCMDIdReadWrite                  =0x0009
    } eCMDId;



                                   /* Set current struct pack, push pack level on compiler stack. Prgma set for pack on alime 0 (=pack)     */
                                   /* #pragma pack(push[,n]) pushes the current alignment setting on an internal stack and then optionally  */
                                   /* sets the new alignment.                                                                               */
                                   /* Also can handle by use of __attribute__ see example:                                                  */
                                   /*       struct  __attribute__((__packed__)) my_struct                                                   */
                                   /*           {                                                                                           */
                                   /*               ...                                                                                     */
                                   /*           };                                                                                          */
                                   /*       int *e __attribute__((aligned(1))) = &a.i;                                                      */
#pragma pack(push, 0)
	typedef struct ADS_AMS_HEADER_TCP_TAG
		{
					/* !!! Exist Only In TCP !!!	*/
														/* These bytes must be set to 0									*/
			ELMO_UINT16	usAMSTcpReserved;
														/* Bytes of packet. Contains AMS-Header and enclosed ADS data.	*/
														/* lENGTH NOT INCUDING THE 6 BYTES OF TCP (2 RESERVED, 4 LENGTH)*/
			ELMO_UINT32	uiAMSTcpLength;
					/* !!! Exist Only In TCP !!!	*/
			
			ELMO_UINT8	cpAMSNetIdTarget[6];    /* AMSNetId of the station, for which the packet is intended.   */
			ELMO_UINT16 usAMSPortTarget;        /* AMSPort of the station, for which the packet is intended.    */
			ELMO_UINT8  cpAMSNetIdSource[6];    /* AMSNetId of the station, from which the packet was sent.     */
			ELMO_UINT16 usAMSPortSource;        /* AMSPort of the station, from which the packet was sent.      */
       
			ELMO_UINT16 usCommandId;                
			ELMO_UINT16 usStateFlags;
			ELMO_UINT32 uiHdrLen;            	/* Size of the data range. The unit is byte.                    */
			ELMO_UINT32 uiErrorCode;            /* AMS error number                                             */
			ELMO_UINT32 uiInvokeId;             /* Free usable 32 bit array. Usually this array serves to send  */
														/* an Id. This Id makes is possible to assign a received        */
														/* response to a request, which was sent before                 */
    } ADS_AMS_HEADER_TCP;
                                   /* Resume the working structure pack (from top of stack)                                                 */
                                   /* #pragma pack(pop) restores the alignment setting to the one saved at the top of the internal stack    */
                                   /* (and removes that stack entry). Note that #pragma pack([n]) does not influence this internal stack;   */
                                   /* thus it is possible to have #pragma pack(push) followed by multiple #pragma pack(n) instances and     */
                                   /* finalized by a single #pragma pack(pop).                                                              */
#pragma pack(pop)

    

                    /* Type of function the class calls for get user data as*/
                    /* response to Client requst Read data from Server      */
    typedef ELMO_INT32 (* ADSAMS_CLBK_REQ_READ)(ELMO_UINT8      ucDataToClient[],
                                         ELMO_UINT32       		uiDataToClientSize,
                                         ELMO_UINT32       		uiIndexGroup,
                                         ELMO_UINT32       		uiIndexOffset,
                                         ELMO_PUINT32			puiResult,
                                         ADS_AMS_HEADER_TCP*    ptAmsHdrFromClient,     /* Copy of the org.         */
                                         ADS_AMS_HEADER_TCP**   pptAmsHdrToClient);     /* If Null take from Rcv    */

                    /* Type of function the class calls for serve client    */
                    /* requsted Write data to server (client sends data)    */
    typedef ELMO_INT32 (* ADSAMS_CLBK_REQ_WRITE)(ELMO_UINT8		ucDataFromClient[],
                                         ELMO_UINT32       		uiDataFromClientSize,
                                         ELMO_UINT32       		uiIndexGroup,
                                         ELMO_UINT32       		uiIndexOffset,
                                         ELMO_PUINT32      		puiResult,
                                         ADS_AMS_HEADER_TCP*    ptAmsHdrFromClient,     /* Copy of the org.         */
                                         ADS_AMS_HEADER_TCP**   pptAmsHdrToClient);     /* If Null take from Rcv    */ 


                    /* Macros for GET/PUT data from/into stream.        */
                    /* Also change from Little-Endian to Big-Endian.    */
                                                        /* See: GNU "Statement Expressions" */
    #define GET_USHORT(ucStream, offSetOnStream)                   		\
            ({                                                          \
                ELMO_UINT16       us1;                                  \
                ELMO_UINT16       us0;                                  \
                us1 = (ELMO_UINT16)ucStream[offSetOnStream + 1] << 8;   \
                us0 = (ELMO_UINT16)ucStream[offSetOnStream + 0] << 0;   \
                                    /* Macro return value */            \
                us0 + us1;                                              \
            })                                                          \

                                                        /* See: GNU "Statement Expressions" */
    #define GET_USINT(ucStream, offSetOnStream)                         \
            ({                                                          \
                ELMO_UINT32         ui3;                                \
                ELMO_UINT32         ui2;                                \
                ELMO_UINT32         ui1;                                \
                ELMO_UINT32         ui0;                                \
                ui3 = (ELMO_UINT16)ucStream[offSetOnStream + 3] << 24;  \
                ui2 = (ELMO_UINT16)ucStream[offSetOnStream + 2] << 16;  \
                ui1 = (ELMO_UINT16)ucStream[offSetOnStream + 1] <<  8;  \
                ui0 = (ELMO_UINT16)ucStream[offSetOnStream + 0] <<  0;  \
                                    /* Macro return value */            \
                ui0 + ui1 + ui2 + ui3;                                  \
            })                                                          \


    #define PUT_USHORT(ucStream, offSetOnStream, usVal)                 \
            ({                                                          \
                ELMO_UINT8        uc1;                                  \
                ELMO_UINT8        uc0;                                  \
                uc1 = (ELMO_UINT8)((usVal >> 8) & 0xff);                \
                uc0 = (ELMO_UINT8)((usVal >> 0) & 0xff);                \
                ucStream[offSetOnStream + 1] = uc1;                     \
                ucStream[offSetOnStream + 0] = uc0;                     \
            })                                                          \


    #define PUT_USINT(ucStream, offSetOnStream, uiVal)                  \
            ({                                                          \
                ELMO_UINT8        uc3;                                  \
                ELMO_UINT8        uc2;                                  \
                ELMO_UINT8        uc1;                                  \
                ELMO_UINT8        uc0;                                  \
                uc3 = (ELMO_UINT8)((uiVal >> 24) & 0xff);               \
                uc2 = (ELMO_UINT8)((uiVal >> 16) & 0xff);               \
                uc1 = (ELMO_UINT8)((uiVal >>  8) & 0xff);               \
                uc0 = (ELMO_UINT8)((uiVal >>  0) & 0xff);               \
                ucStream[offSetOnStream + 3] = uc3;                     \
                ucStream[offSetOnStream + 2] = uc2;                     \
                ucStream[offSetOnStream + 1] = uc1;                     \
                ucStream[offSetOnStream + 0] = uc0;                     \
            })                                                          \




                            /* Implement ADS/AMS READ,WRITE protocol    */
    class DLLMMCPP_API CMMCAdsAmsTcpProtocol
    {
    public:
        CMMCAdsAmsTcpProtocol() : _iServerSock(-1), _bClbkThreadAlive(false)
        {
            _giDbgLclFlag = 0;
            _giDbgCommuFlag = 0;

            /* From MMCTCP ???? */

            #ifdef WIN32
                static ELMO_INT32 err = WSAStartup(MAKEWORD(2,0),&m_wsaData); //one-time initialization for a proccess.
                if(err != MMC_OK) {
                    cout<<"Socket Initialization Error. Program aborted\n";
                    return;
                }
            #endif

            memset(&_saddConnect, 0, sizeof (_saddConnect));
            _iSock=-1;
            /* From MMCTCP ???? */
        };

        ~ CMMCAdsAmsTcpProtocol()
        {
            /* From MMCTCP ???? */
            _bClbkThreadAlive = false; //quit call-back mode thread.
            if (_iServerSock!=-1) //if function as server
            {
                this->Close(_iServerSock); //close listener file descriptor.
            }
            if (_iSock!=-1)
            {
                this->Close(_iSock); //close client file descriptor.
            }
            /* From MMCCPP ???? */
        };


                            /* Application Initialize (connection to ADS/AMS) supplay:  */
                            /*  port number,                                            */
                            /*  ClientReqRead - call back function, response to Client  */
                            /*          requst Read Data from server.                   */
                            /*  ClientReqWrite - Call back function, response to Client */
                            /*          requst Write Data to server.                    */
        ELMO_INT32     CMMCAdsAmsTcpProtocolInit(ELMO_UINT16 usPort,
                                          ADSAMS_CLBK_REQ_READ  fnClbkRespClientReqRead,
                                          ADSAMS_CLBK_REQ_WRITE fnClbkClientReqWrite);

                            /* Dbg Service, when iDbgCommuFlag != 0 print communication */
        void    AdsAmsSetDbg        (ELMO_INT32 iDbgLclFlag, ELMO_INT32 iDbgCommuFlag);
							/* Dbg service, print string in Hex & Ascii format           */
        void    DumpStreamBuffer    (ELMO_UINT8 pucStrStream[], ELMO_UINT32 uiNumCharForPrint);


/* ??????????????????????????????????????????????????? */
                    /* Original takes From MMCTCP ???? */
                    /* Original takes From MMCTCP ???? */
#include    "AdsAmsTcpProtocolTcp.hpp"
                    /* Original takes From MMCTCP ???? */
                    /* Original takes From MMCTCP ???? */
/* ??????????????????????????????????????????????????? */

    private:
        ELMO_INT32  DecodeAmsHeader     (ELMO_UINT8 ucStrStream[]);
        ELMO_INT32  DecodeGrpOffsetLen  (ELMO_UINT8 ucStrStream[]);
        ELMO_INT32	ResponseClientReq	(eCMDId    CmdId, ELMO_UINT8 ucRcvBuf[], ELMO_UINT32 uiRcvBufSize, ELMO_UINT8 ucSndBuf[], ELMO_PUINT8 pusRespSize);
                                            /* Call from thread while soket receive data */
        ELMO_INT32  SocketfnClbk        (ELMO_INT32 iSock, ELMO_INT8 sSockEvent, ELMO_UINT8 pucReadBuf[], ELMO_INT32 iReadBufSize, ...);

                                            /* Buffers for TCP Send communication to client */
        ELMO_UINT8  _gucTcpSndBuf[TCP_SND_BUF_SIZE];
                                            /* Commad Id, type appear both (exchange between) 'unsigned short'  */
                                            /* and 'eCMDId' be carfull because enumerated type is 'int'.        */
        ELMO_UINT16 _gusRcvCommandId;
                        /* From Client point of view                    */
                        /* Rcv => GMAS got the command (read/write...)  */
		ELMO_UINT32	_guiRcvHdrTcpLen;		/* Len appear in TCP header */
        ELMO_UINT16 _gusRcvPortTarget;  	/* Target port  */
        ELMO_UINT16 _gusRcvPortSource;  	/* Source port  */ 
        ELMO_UINT16 _gusRcvStateFlags;  	/* State Flags  */
        ELMO_UINT32 _guiRcvHdrLen;      	/* Whole range of 'DATA AREA', E.g.: Including: IndexGroup, InexOffset, Length and "Pure Data" */
        ELMO_UINT32 _guiRcvErrorCode;   	/* Error Code   */
        ELMO_UINT32 _guiRcvInvokeId;    	/* Invoke Id.   */
        
                        /* From Last relevant Rcv Command */
        ELMO_UINT32 _guiIndexGroup;     	/* Index Group of the data which should be written.     */
        ELMO_UINT32 _guiIndexOffset;    	/* Index Offset, in which the data should be written.   */
        
                                            /* Length of the data (in bytes) which are written.     */ 
                                            /*   "pure data within data area" Not Including         */
                                            /*   IndexGroup, InexOffset, Length.... E.g. when pure  */
                                            /*   data is 'a' this field be 1.                       */
        ELMO_UINT32    	_guiPureDataLen;


        ELMO_INT32      _giSizeRcv;
        ELMO_INT32      _giSizeForSnd;

                                            /* Pointer to user function call in responce to Client  */
                                            /* requst Read (return roud data from server to client).*/
        ADSAMS_CLBK_REQ_READ   _fnClbkRespClientReqRead;
                                            /* Pointer to user function call while Client requst    */
                                            /* Write (return client data to server).                */
        ADSAMS_CLBK_REQ_WRITE  _fnClbkClientReqWrite;
                                            /* Trace & debug flags                                  */
        ELMO_INT32              _giDbgLclFlag;
        ELMO_INT32              _giDbgCommuFlag;
    };
#endif  /* #ifndef WIN32 */

#endif  /* #define __AdsAmsTcpProtocol_h__ */

