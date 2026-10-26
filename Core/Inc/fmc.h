#ifndef HBI_FMC_H
#define HBI_FMC_H
#include "main.h"
#define LAN9252_HBI_BASE UINT32_C(0x6c000000)
extern SRAM_HandleTypeDef hsram4;
void MX_FMC_Init(void);
#endif
