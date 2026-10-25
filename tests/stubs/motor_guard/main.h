#ifndef TEST_MOTOR_GUARD_MAIN_H
#define TEST_MOTOR_GUARD_MAIN_H
#include <stdint.h>
#include "ethercat_build.h"
typedef enum { HAL_OK=0, HAL_ERROR=1 } HAL_StatusTypeDef;
typedef struct { uint32_t ODR; } GPIO_TypeDef;
extern GPIO_TypeDef test_drv_gpio;
#define DRV_ENA_GPIO_Port (&test_drv_gpio)
#define DRV_ENA_Pin 16U
#define GPIO_PIN_RESET 0U
#define GPIO_PIN_SET 1U
uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
void HAL_GPIO_WritePin(GPIO_TypeDef *gpio, uint32_t pin, uint32_t state);
#endif
