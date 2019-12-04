/*
 * MMCRTGroupAxis.hpp
 *
 *  Created on: 07/07/2016
 *      Author: zivb
 */

#ifndef MMCRTGROUPAXIS_H_
#define MMCRTGROUPAXIS_H_

#include "MMC_APP/MMC_definitions.h"
#include "MMCPP/MMCAxis.hpp"
#include "MMCPP/MMCNode.hpp"
#include "MMCPP/MMCMotionAxis.hpp"
#include "MMCPP/MMCPPGlobal.hpp"
#include "MMCPP/MMCGroupAxis.hpp"

class DLLMMCPP_API CMMCRTGroupAxis:public CMMCGroupAxis
{
public:
	virtual ~CMMCRTGroupAxis();

	CMMCRTGroupAxis();
	CMMCRTGroupAxis(CMMCGroupAxis& axis);

	void InitAxisData(const ELMO_PINT8 cName, MMC_CONNECT_HNDL uHandle) throw (CMMCException);

	void GetPos(ELMO_PDOUBLE dArr, NC_AXIS_IN_GROUP_TYPE_ENUM_EX startIndex, NC_AXIS_IN_GROUP_TYPE_ENUM_EX endIndex)throw (CMMCException);
	void GetVel(ELMO_PDOUBLE dArr, NC_AXIS_IN_GROUP_TYPE_ENUM_EX startIndex, NC_AXIS_IN_GROUP_TYPE_ENUM_EX endIndex)throw (CMMCException);

protected:
	ELMO_PUINT8		m_pShmPtr;
	ELMO_ULINT32	m_ulnode_list_param_base;
	ELMO_ULINT32 	m_ulGlobalParam_base;

	ELMO_ULINT32 	m_ulaPosOffset;
	ELMO_ULINT32	m_ulaVelOffset;
};

#define SHORTPARAMETER(base, offset) *((ELMO_PINT16  )((ELMO_PUINT8)base + offset))
#define INTPARAMETER(base,   offset) *((ELMO_PINT32  )((ELMO_PUINT8)base + offset))
#define UINTPARAMETER(base,  offset) *((ELMO_PUINT32 )((ELMO_PUINT8)base + offset))
#define ULONGPARAMETER(base, offset) *((ELMO_PULINT32)((ELMO_PUINT8)base + offset))
#define DOUBLEPARAMETER(base,offset) *((ELMO_PDOUBLE )((ELMO_PUINT8)base + offset))

#endif /* MMCRTGROUPAXIS_H_ */
