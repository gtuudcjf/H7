#ifndef TEST_DRV8323_HAL_H
#define TEST_DRV8323_HAL_H
#include <stdint.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { uint32_t unused; } SPI_HandleTypeDef;
typedef struct { uint32_t ODR; } GPIO_TypeDef;
#define GPIO_PIN_RESET 0U
#define GPIO_PIN_SET 1U
void HAL_GPIO_WritePin(GPIO_TypeDef *gpio, uint16_t pin, uint32_t state);
void HAL_Delay(uint32_t ms);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi,
    uint8_t *tx, uint8_t *rx, uint16_t count, uint32_t timeout);
#endif
