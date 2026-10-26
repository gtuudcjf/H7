#ifndef TEST_EL9800HW_H
#define TEST_EL9800HW_H
#include <stdint.h>
typedef uint16_t UINT16;
uint8_t HW_Init(void);
void HW_Release(void);
uint32_t HW_GetTimer(void);
uint8_t HW_AccessFault(void);
void HW_EscRead(uint8_t *p, uint16_t address, uint16_t length);
#endif
