#ifndef _LINUX_SAMPLE_
#define _LINUX_SAMPLE_

void Initial_StartFieldBus(void);
void FieldBusBasicFunction_test(void);
void Configuration_test(void);
void GetMotionStatus_test(void);
void AxisInitial(int AxisID);
void HomeMove(void);
void SetEndLimit(void);
void EMGTest(void);
void LogPos(void *loginfo);
void P2PTest(void);
void LineTest(void);
void HomeMoveTest(void);
void JogMoveTest(void);

void PVT_withThread_sample(void);
void PT_withThread_sample(void);


#endif