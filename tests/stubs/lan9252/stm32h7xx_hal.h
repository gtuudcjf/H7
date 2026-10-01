#ifndef TEST_LAN9252_HAL_H
#define TEST_LAN9252_HAL_H
#include <stdint.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { int unused; } SPI_HandleTypeDef;
typedef struct { volatile uint32_t CTRL, CYCCNT, LAR; } DWT_Type;
typedef struct { volatile uint32_t DEMCR; } CoreDebug_Type;
extern DWT_Type test_dwt;
extern CoreDebug_Type test_debug;
extern uint32_t SystemCoreClock;
#define DWT (&test_dwt)
#define CoreDebug (&test_debug)
#define CoreDebug_DEMCR_TRCENA_Msk 0x01000000U
#define DWT_CTRL_CYCCNTENA_Msk 1U
#define GPIOA ((void *)1)
#define GPIO_PIN_4 16U
#define GPIO_PIN_RESET 0U
#define GPIO_PIN_SET 1U
uint32_t HAL_GetTick(void);
void HAL_GPIO_WritePin(void *port, uint16_t pin, uint32_t state);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi,
    const uint8_t *tx, uint8_t *rx, uint16_t length, uint32_t timeout);
#endif
