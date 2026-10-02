#ifndef ETHERCAT_APP_H
#define ETHERCAT_APP_H

#include "ecatappl.h"
#include "objdef.h"

extern TOBJECT OBJMEM ApplicationObjDic[];

void APPL_AckErrorInd(UINT16 stateTrans);
UINT16 APPL_StartMailboxHandler(void);
UINT16 APPL_StopMailboxHandler(void);
UINT16 APPL_StartInputHandler(UINT16 *pIntMask);
UINT16 APPL_StopInputHandler(void);
UINT16 APPL_StartOutputHandler(void);
UINT16 APPL_StopOutputHandler(void);
UINT16 APPL_GenerateMapping(UINT16 *pInputSize, UINT16 *pOutputSize);
void APPL_InputMapping(UINT16 *pData);
void APPL_OutputMapping(UINT16 *pData);
void APPL_Application(void);
UINT16 APPL_GetDeviceID(void);

#endif
