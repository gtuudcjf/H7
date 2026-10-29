#ifndef ENCODER_TEST_HAL_H
#define ENCODER_TEST_HAL_H
#include <stdint.h>
#include <stddef.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT } HAL_StatusTypeDef;
typedef struct { void *Instance; uint32_t ErrorCode; } SPI_HandleTypeDef;
#define SPI4 ((void *)4)
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __enable_irq(void);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_SPI_Abort_IT(SPI_HandleTypeDef *);
#endif
