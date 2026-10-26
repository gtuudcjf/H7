#include "main.h"
extern uint16_t hbi_memory[512];
#define LAN9252_HBI_BASE ((uintptr_t)hbi_memory)
extern SRAM_HandleTypeDef hsram4;
void MX_FMC_Init(void);
