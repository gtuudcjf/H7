/* H743/LAN9252 compatibility declarations for Beckhoff SSC 5.11.
 * Project-owned replacement; the F407 hardware implementation is not imported.
 * Task 7 implements these functions over lan9252_port without changing SSC. */
#ifndef H743_SSC_EL9800HW_H
#define H743_SSC_EL9800HW_H

#include "ecat_def.h"
#include "esc.h"

UINT8 HW_Init(void);
void HW_Release(void);
UINT16 HW_GetALEventRegister(void);
void HW_EscRead(MEM_ADDR *pData, UINT16 Address, UINT16 Len);
void HW_EscWrite(MEM_ADDR *pData, UINT16 Address, UINT16 Len);

/* Monotonic microseconds, modulo 2^32. Never reset the underlying counter. */
UINT32 HW_GetTimer(void);
/* Sticky transport/SM fault; cleared only by explicit reinitialization. */
BOOL HW_AccessFault(void);
void HW_RecordSyncManTimeout(UINT8 channel);
/* Bounded fixed pool; capacity/lifetime requirements are documented in README. */
void *HW_AllocMailboxBuffer(UINT16 size);
void HW_FreeMailboxBuffer(void *pointer);

#define ECAT_TIMER_INC_P_MS 1000U
#define DISABLE_ESC_INT() ((void)0)
#define ENABLE_ESC_INT() ((void)0)
#define HW_GetALEventRegister_Isr HW_GetALEventRegister
#define HW_EscReadIsr HW_EscRead
#define HW_EscWriteIsr HW_EscWrite

#define HW_EscReadByte(value, address) HW_EscRead((MEM_ADDR *)&(value), (UINT16)(address), 1U)
#define HW_EscReadWord(value, address) HW_EscRead((MEM_ADDR *)&(value), (UINT16)(address), 2U)
#define HW_EscReadDWord(value, address) HW_EscRead((MEM_ADDR *)&(value), (UINT16)(address), 4U)
#define HW_EscWriteByte(value, address) HW_EscWrite((MEM_ADDR *)&(value), (UINT16)(address), 1U)
#define HW_EscWriteWord(value, address) HW_EscWrite((MEM_ADDR *)&(value), (UINT16)(address), 2U)
#define HW_EscWriteDWord(value, address) HW_EscWrite((MEM_ADDR *)&(value), (UINT16)(address), 4U)
#define HW_EscReadWordIsr HW_EscReadWord
#define HW_EscReadDWordIsr HW_EscReadDWord
#define HW_EscWriteWordIsr HW_EscWriteWord
#define HW_EscWriteDWordIsr HW_EscWriteDWord
#define HW_EscReadMbxMem(data, address, length) HW_EscRead((MEM_ADDR *)(data), (UINT16)(address), (UINT16)(length))
#define HW_EscWriteMbxMem(data, address, length) HW_EscWrite((MEM_ADDR *)(data), (UINT16)(address), (UINT16)(length))

#endif
