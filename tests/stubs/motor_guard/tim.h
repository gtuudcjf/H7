#include <stdint.h>
typedef struct { uint32_t BDTR; } TestTimer;
typedef struct { TestTimer *Instance; } TIM_HandleTypeDef;
extern TIM_HandleTypeDef htim8;
#define TIM_BDTR_MOE 0x8000U
#define TIM_CHANNEL_4 12U
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *handle, uint32_t channel);
