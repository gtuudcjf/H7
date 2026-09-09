/**
 * @file drv8323.h
 * @brief DRV8323 的最小 SPI 寄存器驱动，不绑定具体 MCU 外设和引脚。
 *
 * 本层只负责 DRV8323 的 16 位 SPI 帧和 ENABLE 控制。SPI2、PC1、PC4 等
 * H7 板级资源统一放在 drv8323_board.c 中绑定，避免以后更换 PCB 时修改
 * 通用驱动。该分层也便于后续电流环复用寄存器读写接口读取故障和 CSA 配置。
 */
#ifndef MOTOR_DRV8323_H
#define MOTOR_DRV8323_H

#include "stm32h7xx_hal.h"

typedef struct
{
    SPI_HandleTypeDef *spi;    /**< 已由 CubeMX 初始化的 SPI 句柄。 */
    GPIO_TypeDef *cs_port;     /**< 软件片选 nSCS 所在 GPIO 端口。 */
    uint16_t cs_pin;           /**< 软件片选 nSCS 引脚掩码。 */
    GPIO_TypeDef *enable_port; /**< DRV8323 ENABLE 所在 GPIO 端口。 */
    uint16_t enable_pin;       /**< DRV8323 ENABLE 引脚掩码。 */
} Drv8323Device;

/**
 * @brief 将一个 DRV8323 设备对象与已经初始化的 MCU 资源绑定。
 * @param device 待绑定的设备对象，不能为 NULL。
 * @param spi SPI 句柄；必须配置为主机、16 位、MSB First、CPOL=0、CPHA=2EDGE。
 * @param cs_port nSCS GPIO 端口。
 * @param cs_pin nSCS GPIO 引脚。
 * @param enable_port ENABLE GPIO 端口。
 * @param enable_pin ENABLE GPIO 引脚。
 * @note 绑定完成后 nSCS 被置高、ENABLE 被置低，使驱动保持未选中和休眠状态。
 */
void Drv8323_Bind(Drv8323Device *device,
                  SPI_HandleTypeDef *spi,
                  GPIO_TypeDef *cs_port,
                  uint16_t cs_pin,
                  GPIO_TypeDef *enable_port,
                  uint16_t enable_pin);

/**
 * @brief 写一个 DRV8323 SPI 寄存器。
 * @param device 已完成绑定的设备对象。
 * @param address 4 位寄存器地址，允许范围 0x0～0xF。
 * @param data 11 位寄存器数据，允许范围 0x000～0x7FF。
 * @retval HAL_OK SPI 传输完成。
 * @retval HAL_ERROR 参数非法或底层 SPI 句柄无效。
 * @retval HAL_BUSY/HAL_TIMEOUT HAL SPI 返回的忙或超时状态。
 * @warning HAL_OK 只表示 SPI 外设完成传输，不代表芯片已经接受配置；严格确认需要读回。
 */
HAL_StatusTypeDef Drv8323_WriteRegister(Drv8323Device *device,
                                        uint8_t address,
                                        uint16_t data);

/**
 * @brief 读取一个 DRV8323 SPI 寄存器。
 * @param device 已完成绑定的设备对象。
 * @param address 4 位寄存器地址，允许范围 0x0～0xF。
 * @param data 返回低 11 位有效寄存器数据，不能为 NULL。
 * @return HAL SPI 状态。
 */
HAL_StatusTypeDef Drv8323_ReadRegister(Drv8323Device *device,
                                       uint8_t address,
                                       uint16_t *data);

/**
 * @brief 唤醒 DRV8323，并写入与原 F407 工程等效的 6x PWM 配置。
 * @return HAL SPI 状态；任意一次写失败都会将 ENABLE 拉低。
 * @note 此函数包含毫秒级等待，只能用于初始化/状态切换，禁止在快速控制中断中调用。
 */
HAL_StatusTypeDef Drv8323_EnableAndConfigure6Pwm(Drv8323Device *device);

/**
 * @brief 拉低 ENABLE，使 DRV8323 进入关闭/休眠过程。
 * @note 调用者应先停止六路 PWM，使 INHx/INLx 先进入低电平安全状态。
 */
void Drv8323_Disable(Drv8323Device *device);

#endif /* MOTOR_DRV8323_H */
