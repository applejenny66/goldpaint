/*
============================================================================
 Name : AdsAmsTcpProtocolTcp.h
 Author : Haim H.
 Version :      14Jul2015
 Description :  Implement ADS / AMS Beckhoof protocol above Tcp.
                GMAS implementation for be Server responce to ADS client.
                This part of header takes from MMCTcp.h and little change
                for AdsAms.
============================================================================
*/

#ifndef __AdsAmsTcpProtocolTcp_HPP__
#define __AdsAmsTcpProtocolTcp_HPP__

#if   (OS_PLATFORM == LINUXIPC32_PLATFORM)

#define MMCPP_TCP_MAX_CLIENTS   17
#define MMCPP_TCP_MAX_SIZE 		1500 //In accordance with MTU definitoins


    /**! \fn int Create(unsigned int uiPort, bool bClbk);
    *   \brief creates singleton socket listener
    *   creates singleton socket listener, which is invoked within dedicated thread
    *   if bClbk is true, otherwise within the main thread.
    *   \param uiPort socket listener port
    *   \param fnClbk call-back function address. if not NULL then callback mode of operation. otherwise normal.
    *   \return 0 on success, error id otherwise.
    */
//    int Create(unsigned int uiPort, SOCK_CLBK fnClbk=NULL, int iMsgMaxSize=512)throw (CMMCException);
    ELMO_INT32 CreateSocketfnClbk(ELMO_UINT32 uiPort, ELMO_INT32 iMsgMaxSize=512)throw (CMMCException);

    /**! \fn int Accept();
    *   \brief fetch new connection if exists.
    *   \return new connection from socket listener if exists, otherwise -1.
    */
    ELMO_INT32 Accept();

    /**! \fn int Receive();
    *   \brief receives data from given socket connection.
    *   \param iSock socket connection by which message is being read.
    *   \param usSize expected size (bytes) to read
    *   \param pData message buffer to read message into.
    *   \param lDelayMilSec, if missing no delay, other: Mil-Sec read timeout.
    *   \return number of bytes actually received or -1 if error.
    */
    ELMO_INT32 Receive(ELMO_INT32 iSock, ELMO_UINT16 usSize, ELMO_PVOID pData, ELMO_LINT32 lDelayMilSec=0)throw (CMMCException);

    /**! \fn Send (int iSock, unsigned short usSize, void * pData, bool& bFail)
     * \brief sends data on a non blocking client socket
     * \param iSock (IN) client socket
     * \param usSize IN) number of bytes to send
     * \param pData pData (IN) pointer to data to send
     * \param (OUT) set to TRUE on error: socket must be closed
     * \return number of bytes actually sent, -1 otherwise.
     */
    ELMO_INT32 Send (ELMO_INT32 iSock, ELMO_UINT16 usSize, ELMO_PVOID pData)throw (CMMCException);

    /**! \fn Connect(char* szAddr, unsigned short usPort, int& iSocket,bool& bWait)
     * \brief creates a non blocking socket connected to a remote server
     * \param szAddr (IN) IP address of the server
     * \param usPort (IN) Ethernet port number of the server
     * \parm iSocket (OUT) created socket if OK
     * \param bWait (OUT) set to TRUE if connect is not fully completed
     * \return: OK or ERROR
     */
    ELMO_INT32 Connect(ELMO_PINT8 szAddr, ELMO_UINT16 usPort, ELMO_INT32& iSocket, ELMO_BOOL& bWait, ELMO_INT32 iMsgMaxSize=512)throw (CMMCException);

    /**! \fn int Close();
    *   \brief close client socket connection.
    *   \param iSock client socket connection to close.
    *   \return 0 on success, error id otherwise.
    */
    ELMO_INT32 Close(ELMO_INT32 iSock);
    /**! \fn int Close();
    *   \brief close server listener socket connection.
    *   \return 0 on success, error id otherwise.
    */
    ELMO_INT32 Close();

    /**! \fn bool IsWritable(int iSock);
     * \brief checks for errors and whether or not connection is ready for write operation.
     * \param iSock client socket connection to check.
     * return true if writable, otherwise false.
     */
    ELMO_BOOL IsWritable(ELMO_INT32 iSock)throw (CMMCException);

    /**! \fn bool IsReadable(int iSock, int iTimeOut=0);
     * \brief checks for errors and whether or not connection is ready for read operation.
     * \param iSock socket connection to check.
     * return true if readable, otherwise false.
     */
    ELMO_BOOL IsReadable(ELMO_INT32 iSock, ELMO_INT32 iTimeOut=0)throw (CMMCException);


protected:
	//#ifndef WIN32
	#if   (OS_PLATFORM == LINUXIPC32_PLATFORM)
		friend ELMO_PVOID fnTCPClbkThread(ELMO_PVOID);   //call-back argument for pthread_create within RunClbkThread.
	#else
		friend DWORD WINAPI fnTCPClbkThread( LPVOID lpParam );
	#endif

    /**! \fn int IsPending(int iSock, bool& bFail)
     * \brief check pending connection on a non blocking socket
     *  actuall checks for errors and whether or not connection is ready for write operation.
     * \param iSock client socket connection to check.
     * \return: OK (0) if iSock ready for write operation or ERROR otherwise (still pending for instance)
     */
    ELMO_INT32 IsPending(ELMO_INT32 iSock)throw (CMMCException);
    ELMO_INT32 RunClbkThread();
    ELMO_INT32 CreateClbk()throw (CMMCException);
    void ClbkThread();
private:
    typedef void (CMMCAdsAmsTcpProtocol::* PCLBKTHREAD)(ELMO_PVOID);
    PCLBKTHREAD	_pThreadClbk;
    ELMO_INT32  _iServerSock;
    ELMO_INT32  _iSock;         //client socket connection
//    SOCK_CLBK   _fnClbk;
    ELMO_UINT32 _uiPort;
    ELMO_INT32	_iAddrLen;
    ELMO_BOOL	_bClbkThreadAlive;
    ELMO_INT32	_iClientConnectionsArr[MMCPP_TCP_MAX_CLIENTS];
    ELMO_INT32	_iMsgMaxSize;
    struct sockaddr_in  _saddConnect ;

	//#ifdef WIN32
	#if   (OS_PLATFORM == WIN32_PLATFORM) || (OS_PLATFORM == WIN64_PLATFORM)
		WSADATA m_wsaData;
		HANDLE  _thread;
	#else
		pthread_t _thread;
	#endif

#endif		/* #if   (OS_PLATFORM == LINUXIPC32_PLATFORM) */

#endif  /* #ifndef __AdsAmsTcpProtocolTcp_H__ */

