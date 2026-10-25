#ifndef TEST_DRV8323_MAIN_H
#define TEST_DRV8323_MAIN_H
#include "stm32h7xx_hal.h"
extern GPIO_TypeDef test_drv_gpio;
#define DRV_CS_GPIO_Port (&test_drv_gpio)
#define DRV_CS_Pin 2U
#define DRV_ENA_GPIO_Port (&test_drv_gpio)
#define DRV_ENA_Pin 16U
#define DRV_CAL_GPIO_Port (&test_drv_gpio)
#define DRV_CAL_Pin 32U
#endif
