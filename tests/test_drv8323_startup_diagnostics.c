/* Real board wrapper + real SPI framing; only peripheral transport and the
 * independent permission source are replaced. No physical timing is tested. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "ethercat_build.h"
#include "drv8323_board.h"
#include "main.h"
#include "spi.h"

GPIO_TypeDef test_drv_gpio;
SPI_HandleTypeDef hspi2;
static uint16_t registers[7], writes[3], read_addresses[7];
static unsigned write_count, read_count, permit;
static int fail_read, fail_write, revoke_read, csa_override;

void HAL_GPIO_WritePin(GPIO_TypeDef *gpio, uint16_t pin, uint32_t state)
{
    if (state) gpio->ODR |= pin;
    else gpio->ODR &= ~(uint32_t)pin;
}
void HAL_Delay(uint32_t ms) { assert(ms == 5U); }
uint8_t MotorSafety_OutputPermitted(void) { return (uint8_t)permit; }
uint8_t MotorSafety_TryEnableDriver(void)
{
    if (permit) test_drv_gpio.ODR |= DRV_ENA_Pin;
    return (uint8_t)permit;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi,
    uint8_t *tx, uint8_t *rx, uint16_t count, uint32_t timeout)
{
    uint16_t word = *(uint16_t *)tx;
    unsigned address = (word >> 11) & 15U;
    assert(spi == &hspi2 && count == 1U && timeout == 10U);
    assert(!(test_drv_gpio.ODR & DRV_CS_Pin));
    assert(test_drv_gpio.ODR & DRV_ENA_Pin);
    assert(!(test_drv_gpio.ODR & DRV_CAL_Pin));
    assert(address < 7U);
    if (word & 0x8000U) {
        assert(write_count == 3U); /* diagnostic follows existing config writes */
        assert(read_count < 7U);
        read_addresses[read_count++] = (uint16_t)address;
        if ((int)address == revoke_read) {
            permit = 0;
            test_drv_gpio.ODR &= ~DRV_ENA_Pin; /* independent guard revoked */
        }
        if ((int)address == fail_read) return HAL_TIMEOUT;
        /* Upper five bits must be ignored by the real read implementation. */
        *(uint16_t *)rx = 0xF800U | registers[address];
        if (address == 6U && csa_override >= 0)
            *(uint16_t *)rx = 0xF800U | (uint16_t)csa_override;
    } else {
        assert(write_count < 3U);
        writes[write_count++] = word;
        if ((int)address == fail_write) return HAL_ERROR;
        registers[address] = word & 0x7FFU;
        if (address == 2U) registers[address] &= ~1U; /* CLR_FLT self-clears */
        *(uint16_t *)rx = 0;
    }
    return HAL_OK;
}
static void Prepare(void)
{
    memset(registers, 0, sizeof(registers));
    registers[0] = 0x101U; registers[1] = 0x040U;
    registers[3] = 0x3FFU; registers[4] = 0x7FFU;
    write_count = read_count = 0; permit = 1;
    fail_read = fail_write = revoke_read = csa_override = -1;
    assert(Drv8323Board_Init() == HAL_OK);
}
static void CheckOriginalWrites(void)
{
    assert(write_count == 3U);
    assert(writes[0] == 0x1001U);
    assert(writes[1] == 0x3203U);
    assert(writes[2] == 0x2B1FU);
}

int main(void)
{
    Prepare();
    assert(Drv8323Board_EnableForPwm() == HAL_OK);
    CheckOriginalWrites();
#if ETHERCAT_MOTOR_OPEN_ANGLE
    const uint16_t expected_registers[7] = {0x101U, 0x040U, 0x000U,
                                          0x3FFU, 0x7FFU, 0x31FU, 0x203U};
    assert(read_count == 7U);
    assert(g_drv8323_startup_diagnostics.session == 1U);
    assert(g_drv8323_startup_diagnostics.active == 0U);
    assert(g_drv8323_startup_diagnostics.configure_status == HAL_OK);
    assert(g_drv8323_startup_diagnostics.valid_mask == 0x7FU);
    assert(g_drv8323_startup_diagnostics.attempted_mask == 0x7FU);
    assert(!g_drv8323_startup_diagnostics.permission_lost);
    for (unsigned i = 0; i < 7U; ++i) {
        assert(read_addresses[i] == i);
        assert(g_drv8323_startup_diagnostics.reg[i] == expected_registers[i]);
        assert(g_drv8323_startup_diagnostics.read_status[i] == HAL_OK);
    }
    {   /* Stop/CAL cleanup preserve the snapshot and perform no further SPI. */
        Drv8323StartupDiagnostics frozen;
        memcpy(&frozen, (const void *)&g_drv8323_startup_diagnostics, sizeof(frozen));
        Drv8323Board_Disable();
        Drv8323Board_SetCurrentCalibration(false);
        assert(read_count == 7U);
        assert(!memcmp(&frozen, (const void *)&g_drv8323_startup_diagnostics, sizeof(frozen)));
    }

    Prepare(); fail_read = 3;
    assert(Drv8323Board_EnableForPwm() == HAL_OK); /* diagnostic is not policy */
    CheckOriginalWrites();
    assert(read_count == 4U); /* no retry or accumulating timeout sequence */
    assert(g_drv8323_startup_diagnostics.session == 2U);
    assert(g_drv8323_startup_diagnostics.valid_mask == 7U);
    assert(g_drv8323_startup_diagnostics.attempted_mask == 15U);
    assert(g_drv8323_startup_diagnostics.read_status[3] == HAL_TIMEOUT);
    assert(g_drv8323_startup_diagnostics.reg[3] == 0xFFFFU);
    for (unsigned i = 4; i < 7U; ++i) {
        assert(g_drv8323_startup_diagnostics.reg[i] == 0xFFFFU);
        assert(g_drv8323_startup_diagnostics.read_status[i] == 0xFFFFFFFFU);
    }
    assert(!g_drv8323_startup_diagnostics.active);

    Prepare(); csa_override = 0x283;
    assert(Drv8323Board_EnableForPwm() == HAL_OK);
    CheckOriginalWrites();
    assert(g_drv8323_startup_diagnostics.reg[6] == 0x283U); /* no silent repair */

    Prepare(); fail_write = 5;
    assert(Drv8323Board_EnableForPwm() == HAL_ERROR);
    assert(read_count == 0U && !(test_drv_gpio.ODR & DRV_ENA_Pin));
    assert(g_drv8323_startup_diagnostics.configure_status == HAL_ERROR);
    assert(g_drv8323_startup_diagnostics.valid_mask == 0U);
    assert(g_drv8323_startup_diagnostics.attempted_mask == 0U);
    assert(!g_drv8323_startup_diagnostics.active);

    Prepare(); revoke_read = 1;
    assert(Drv8323Board_EnableForPwm() == HAL_ERROR);
    assert(read_count == 2U && !(test_drv_gpio.ODR & DRV_ENA_Pin));
    assert(g_drv8323_startup_diagnostics.valid_mask == 3U);
    assert(g_drv8323_startup_diagnostics.permission_lost == 1U);
    assert(!g_drv8323_startup_diagnostics.active);

    Prepare(); revoke_read = 6; /* also recheck after the final SPI transfer */
    assert(Drv8323Board_EnableForPwm() == HAL_ERROR);
    assert(read_count == 7U && !(test_drv_gpio.ODR & DRV_ENA_Pin));
    assert(g_drv8323_startup_diagnostics.valid_mask == 0x7FU);
    assert(g_drv8323_startup_diagnostics.permission_lost == 1U);

    Prepare(); permit = 0;
    assert(Drv8323Board_EnableForPwm() == HAL_ERROR);
    assert(write_count == 0U && read_count == 0U);
    assert(g_drv8323_startup_diagnostics.configure_status == HAL_ERROR);
    assert(g_drv8323_startup_diagnostics.valid_mask == 0U);
    assert(!g_drv8323_startup_diagnostics.active);
    puts("DRV8323 startup diagnostic: raw reads, failures, freeze, next session and revocation passed");
#else
    assert(read_count == 0U);
    puts("Legacy DRV8323 startup: original three writes, no diagnostic reads passed");
#endif
    return 0;
}
