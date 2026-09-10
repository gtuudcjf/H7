/**
 * @file drv8323_registers.h
 * @brief 本工程使用的 DRV8323 寄存器字段定义。
 *
 * 该头文件不依赖 STM32 HAL，寄存器组合值可以在主机测试中直接检查。
 */
#ifndef MOTOR_DRV8323_REGISTERS_H
#define MOTOR_DRV8323_REGISTERS_H

#include <stdint.h>

#define DRV8323_REG_DRIVER_CONTROL       (0x02U)
#define DRV8323_REG_OCP_CONTROL          (0x05U)
#define DRV8323_REG_CSA_CONTROL          (0x06U)

#define DRV8323_DRIVER_CONTROL_6X        (0x0001U)
#define DRV8323_OCP_CONTROL_DEFAULT      (0x031FU)

/* CSA_CONTROL: bit9=VREF_DIV；bits7:6=CSA_GAIN；bits1:0=SEN_LVL。 */
#define DRV8323_CSA_VREF_DIV_2           (1U << 9)
#define DRV8323_CSA_GAIN_SHIFT           (6U)
#define DRV8323_CSA_GAIN_MASK            (3U << DRV8323_CSA_GAIN_SHIFT)
#define DRV8323_CSA_GAIN_5_V_PER_V       (0U << DRV8323_CSA_GAIN_SHIFT)
#define DRV8323_CSA_GAIN_10_V_PER_V      (1U << DRV8323_CSA_GAIN_SHIFT)
#define DRV8323_CSA_GAIN_20_V_PER_V      (2U << DRV8323_CSA_GAIN_SHIFT)
#define DRV8323_CSA_GAIN_40_V_PER_V      (3U << DRV8323_CSA_GAIN_SHIFT)
#define DRV8323_CSA_SENSE_LEVEL_MAX      (3U)

#define DRV8323_CSA_CONTROL_DEFAULT      \
    ((uint16_t)(DRV8323_CSA_VREF_DIV_2 | \
                DRV8323_CSA_GAIN_5_V_PER_V | \
                DRV8323_CSA_SENSE_LEVEL_MAX))

#endif /* MOTOR_DRV8323_REGISTERS_H */
