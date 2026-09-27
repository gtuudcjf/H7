#ifndef TELEMETRY_TEST_HAL_H
#define TELEMETRY_TEST_HAL_H
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { uint32_t gState; } UART_HandleTypeDef;
#define HAL_UART_STATE_BUSY_TX 0x21U
#define HAL_UART_STATE_READY 0x20U
#define OTG_FS_IRQn 67
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t);
uint32_t NVIC_GetEnableIRQ(int);
void NVIC_DisableIRQ(int);
void NVIC_EnableIRQ(int);
#define __DSB() ((void)0)
#define __ISB() ((void)0)
#endif
