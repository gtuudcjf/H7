/**
 * @file drv8323.c
 * @brief DRV8323 16 位 SPI 通信与当前 6 路 PWM 参数配置。
 *
 * DRV8323 的输入帧格式为：B15=读写位，B14:B11=寄存器地址，
 * B10:B0=11 位数据。芯片在 SCLK 下降沿采样 SDI，因此板级 SPI 使用
 * CPOL=0、CPHA=2EDGE；每次 nSCS 拉低期间必须恰好发送 16 个时钟。
 */
#include "drv8323.h"

#define DRV8323_REG_DRIVER_CONTROL (0x02U) /**< Driver Control 寄存器。 */
#define DRV8323_REG_OCP_CONTROL    (0x05U) /**< OCP Control 寄存器。 */
#define DRV8323_REG_CSA_CONTROL    (0x06U) /**< CSA Control 寄存器。 */
#define DRV8323_SPI_READ_BIT       (0x8000U) /**< B15：1=读，0=写。 */
#define DRV8323_SPI_ADDRESS_SHIFT  (11U)     /**< 地址位位于 B14:B11。 */
#define DRV8323_SPI_DATA_MASK      (0x07FFU) /**< B10:B0 的有效数据掩码。 */

/*
 * 与原 F407 工程实际编译的 Function/DRV8323.c 保持等效：
 *   0x0001：PWM_MODE=00（6x PWM），CLR_FLT=1；
 *   0x02C3：VREF_DIV=1（VREF/2），CSA_GAIN=11（40 V/V），SEN_LVL=11；
 *   0x031F：DEAD_TIME=11（400 ns），OCP_MODE=00（锁存），
 *           OCP_DEG=01（4 us），VDS_LVL=1111（1.88 V）。
 * 高、低侧栅极驱动电流寄存器沿用芯片复位默认值，与原工程初始化流程一致。
 */
#define DRV8323_DRIVER_CONTROL_6X (0x0001U)
#define DRV8323_CSA_CONTROL       (0x02C3U)
#define DRV8323_OCP_CONTROL       (0x031FU)

static HAL_StatusTypeDef Drv8323_TransferWord(Drv8323Device *device,
                                               uint16_t tx_word,
                                               uint16_t *rx_word)
{
    HAL_StatusTypeDef status;

    if ((device == 0) || (device->spi == 0) || (rx_word == 0))
    {
        return HAL_ERROR;
    }

    /* nSCS 的下降沿开始一帧；SCLK 空闲电平必须保持为低。 */
    HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_RESET);

    /*
     * SPI2 配置为 16 位数据帧，因此 HAL 的 Size 参数表示“1 个半字”，
     * 收发缓冲区也必须以 uint16_t 保存。原 F407 代码在 16 位模式下使用
     * Size=2 和两字节数组，可能让 HAL 发送两个半字并越过数组边界；H7
     * 这里固定为 Size=1，确保 nSCS 低电平期间只有完整的 16 个 SCLK。
     */
    status = HAL_SPI_TransmitReceive(device->spi,
                                     (uint8_t *)&tx_word,
                                     (uint8_t *)rx_word,
                                     1U,
                                     10U);

    /*
     * 无论 SPI 成功、忙或超时，都必须释放 nSCS，避免驱动芯片一直停留在
     * 未结束帧状态。连续两帧之间 nSCS 高电平必须满足芯片规定的最短时间。
     */
    HAL_GPIO_WritePin(device->cs_port, device->cs_pin, GPIO_PIN_SET);
    return status;
}

void Drv8323_Bind(Drv8323Device *device,
                  SPI_HandleTypeDef *spi,
                  GPIO_TypeDef *cs_port,
                  uint16_t cs_pin,
                  GPIO_TypeDef *enable_port,
                  uint16_t enable_pin)
{
    if (device == 0)
    {
        return;
    }

    /* 这里只保存依赖，不重复初始化 CubeMX 已经配置好的 SPI/GPIO。 */
    device->spi = spi;
    device->cs_port = cs_port;
    device->cs_pin = cs_pin;
    device->enable_port = enable_port;
    device->enable_pin = enable_pin;
    /* 先结束片选，再禁止驱动；这是上电阶段的默认安全状态。 */
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(enable_port, enable_pin, GPIO_PIN_RESET);
}

HAL_StatusTypeDef Drv8323_WriteRegister(Drv8323Device *device,
                                        uint8_t address,
                                        uint16_t data)
{
    uint16_t rx_word;
    uint16_t tx_word;

    if ((address > 0x0FU) || (data > DRV8323_SPI_DATA_MASK))
    {
        return HAL_ERROR;
    }

    /* 写命令 B15=0，所以只需要组合地址和低 11 位数据。 */
    tx_word = ((uint16_t)address << DRV8323_SPI_ADDRESS_SHIFT) | data;

    /* SPI 为全双工，写寄存器时仍需提供接收半字，但本层暂不使用返回旧值。 */
    return Drv8323_TransferWord(device, tx_word, &rx_word);
}

HAL_StatusTypeDef Drv8323_ReadRegister(Drv8323Device *device,
                                       uint8_t address,
                                       uint16_t *data)
{
    uint16_t rx_word;
    uint16_t tx_word;
    HAL_StatusTypeDef status;

    if ((address > 0x0FU) || (data == 0))
    {
        return HAL_ERROR;
    }

    /* 读命令 B15=1；发送数据字段清零。 */
    tx_word = DRV8323_SPI_READ_BIT | ((uint16_t)address << DRV8323_SPI_ADDRESS_SHIFT);
    status = Drv8323_TransferWord(device, tx_word, &rx_word);
    if (status == HAL_OK)
    {
        /* 返回帧高 5 位无效，只保留 B10:B0。 */
        *data = rx_word & DRV8323_SPI_DATA_MASK;
    }
    return status;
}

HAL_StatusTypeDef Drv8323_EnableAndConfigure6Pwm(Drv8323Device *device)
{
    HAL_StatusTypeDef status;

    if ((device == 0) || (device->enable_port == 0))
    {
        return HAL_ERROR;
    }

    /*
     * ENABLE 高电平使芯片退出休眠。只有在 VM 高于欠压阈值时，内部电源、
     * 电荷泵和 SPI 才会进入工作状态。
     */
    HAL_GPIO_WritePin(device->enable_port, device->enable_pin, GPIO_PIN_SET);

    /* 数据手册给出的 SPI ready / wake-up 时间为 1 ms，5 ms 留出启动裕量。 */
    HAL_Delay(5U);

    /* 先选择 6x PWM 并清除锁存故障，再配置后续 CSA 和过流参数。 */
    status = Drv8323_WriteRegister(device,
                                   DRV8323_REG_DRIVER_CONTROL,
                                   DRV8323_DRIVER_CONTROL_6X);
    if (status != HAL_OK)
    {
        Drv8323_Disable(device);
        return status;
    }
    /* 保持原工程的 VREF/2、40 V/V 电流采样放大器配置。 */
    status = Drv8323_WriteRegister(device, DRV8323_REG_CSA_CONTROL, DRV8323_CSA_CONTROL);
    if (status != HAL_OK)
    {
        Drv8323_Disable(device);
        return status;
    }
    /* 最后设置驱动内部死区、VDS 过流模式、消隐时间和阈值。 */
    status = Drv8323_WriteRegister(device, DRV8323_REG_OCP_CONTROL, DRV8323_OCP_CONTROL);
    if (status != HAL_OK)
    {
        Drv8323_Disable(device);
    }
    return status;
}

void Drv8323_Disable(Drv8323Device *device)
{
    if ((device == 0) || (device->enable_port == 0))
    {
        return;
    }

    /* 拉低后芯片关闭栅极驱动并进入休眠过程；PWM 停止顺序由上层保证。 */
    HAL_GPIO_WritePin(device->enable_port, device->enable_pin, GPIO_PIN_RESET);
}
