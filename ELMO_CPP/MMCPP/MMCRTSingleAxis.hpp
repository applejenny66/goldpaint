/*
 * MMCRTSingleAxis.hpp
 *
 *  Created on: Mar 31, 2016
 *      Author: ZivB
 */

#ifndef MMCRTSINGLEAXIS_H_
#define MMCRTSINGLEAXIS_H_

//#ifndef WIN32
#include "MMC_APP/MMC_definitions.h"
#include "MMCPP/MMCAxis.hpp"
#include "MMCPP/MMCNode.hpp"
#include "MMCPP/MMCMotionAxis.hpp"
#include "MMCPP/MMCPPGlobal.hpp"
#include "MMCPP/MMCSingleAxis.hpp"

class DLLMMCPP_API CMMCRTSingleAxis : public CMMCSingleAxis
{
public:
	virtual ~CMMCRTSingleAxis();

	CMMCRTSingleAxis();
	CMMCRTSingleAxis(CMMCSingleAxis& axis);

	void InitAxisData(const ELMO_PINT8 cName, MMC_CONNECT_HNDL uHandle) throw (CMMCException);
	void SetUser607A(ELMO_INT32 iUser607Aval) 	throw (CMMCException);
	void SetUser60FF(ELMO_INT32 iUser60FFval) 	throw (CMMCException);
	void SetUser60B1(ELMO_INT32 iUser60B1val) 	throw (CMMCException);
	void SetUser60B2(ELMO_DOUBLE iUser60B2val) 	throw (CMMCException);
	void SetUser6071(ELMO_DOUBLE dUser6071val) 	throw (CMMCException);
	void SetUserMB1 (ELMO_INT32 iUserMB1) 		throw (CMMCException);
	void SetUserMB2 (ELMO_INT32 iUserMB2) 		throw (CMMCException);
	void SetUserCW  (ELMO_UINT16 usUserCWval) 	throw (CMMCException);


	ELMO_INT32 		GetUser607A() 		throw (CMMCException);
	ELMO_INT32 		GetUser60FF() 		throw (CMMCException);
	ELMO_INT32 		GetUser60B1() 		throw (CMMCException);
	ELMO_DOUBLE 	GetUser60B2() 		throw (CMMCException);
	ELMO_DOUBLE 	GetUser6071() 		throw (CMMCException);
	ELMO_INT32 		GetUserMB1() 		throw (CMMCException);
	ELMO_INT32 		GetUserMB2() 		throw (CMMCException);
	ELMO_DOUBLE 	GetActualPosition() throw (CMMCException);
	ELMO_DOUBLE 	GetActualVelocity() throw (CMMCException);
	ELMO_DOUBLE 	GetdPos() 			throw (CMMCException);
	ELMO_DOUBLE 	GetdVel() 			throw (CMMCException);
	ELMO_INT16		GetAnalogInput() 	throw (CMMCException);
	ELMO_UINT32 	GetDigitalInputs() 	throw (CMMCException);
	void 			SetDigitalOutPuts(ELMO_UINT32 uiDigitalOutputs) throw (CMMCException);
	ELMO_DOUBLE 	GetActualCurrent() 	throw (CMMCException);
	ELMO_UINT32 	GetPLCOpenStatus() 	throw (CMMCException);
	ELMO_UINT16 	GetControlWord() 	throw (CMMCException);
	ELMO_UINT16 	GetStatusWord() 	throw (CMMCException);
	ELMO_ULINT32	GetCycleCounter()	throw (CMMCException);

	ELMO_INT32 		GetUserCW() 		throw (CMMCException);

	ELMO_INT16 		GetCpldAdcChA1() 	throw (CMMCException);
	ELMO_INT16 		GetCpldAdcChA2() 	throw (CMMCException);
	ELMO_INT16 		GetCpldAdcChB1() 	throw (CMMCException);
	ELMO_INT16 		GetCpldAdcChB2() 	throw (CMMCException);

protected:

	ELMO_PUINT8  m_pShmPtr;
	ELMO_ULINT32 m_ulnode_list_param_base;
	ELMO_ULINT32 m_ulGlobalParam_base;

	ELMO_ULINT32 m_uldPosOffset;
	ELMO_ULINT32 m_ulUser607AOffset;
	ELMO_ULINT32 m_ulUser60FFOffset;
	ELMO_ULINT32 m_ulUser60B1Offset;
	ELMO_ULINT32 m_ulUser6071Offset;
	ELMO_ULINT32 m_ulUser60B2Offset;
	ELMO_ULINT32 m_ulMB1Offset;
	ELMO_ULINT32 m_ulMB2Offset;
	ELMO_ULINT32 m_ulTargetVelOffset;
	ELMO_ULINT32 m_ulActualPosOffset;
	ELMO_ULINT32 m_ulActualVelUUOffset;
	ELMO_ULINT32 m_ulAnsalogInputOffset;
	ELMO_ULINT32 m_uiDigitalInputsOffset;
	ELMO_ULINT32 m_ulActualCurrentOffset;
	ELMO_ULINT32 m_ulStatusOffset;
	ELMO_ULINT32 m_ulControlWordOffset;
	ELMO_ULINT32 m_ulStatusWordOffset;
	ELMO_ULINT32 m_ulUserCWOffset;

	ELMO_ULINT32 m_CycleCounterOffset;

	double m_dbTorquemAToRCRatio;
	float m_fMotorRatedCurrent;
	ELMO_ULINT32 m_ulCycleTime;
};

#define USHORTPARAMETER(base,offset) *((ELMO_PUINT16 )((ELMO_PUINT8)base + offset))
#define SHORTPARAMETER(base, offset) *((ELMO_PINT16  )((ELMO_PUINT8)base + offset))
#define INTPARAMETER(base,   offset) *((ELMO_PINT32  )((ELMO_PUINT8)base + offset))
#define UINTPARAMETER(base,  offset) *((ELMO_PUINT32 )((ELMO_PUINT8)base + offset))
#define ULONGPARAMETER(base, offset) *((ELMO_PULINT32)((ELMO_PUINT8)base + offset))
#define DOUBLEPARAMETER(base,offset) *((ELMO_PDOUBLE )((ELMO_PUINT8)base + offset))

//#endif //WIN32
#endif /* MMCRTSINGLEAXIS_H_ */
