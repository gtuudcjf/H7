/* Exercises the real board adapter; only physical HAL/DWT boundaries are fake. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lan9252_board.h"
#include "spi.h"

DWT_Type test_dwt;
CoreDebug_Type test_debug;
uint32_t SystemCoreClock = 480000000U;
SPI_HandleTypeDef hspi1;
static SPI_TypeDef spi_registers;
static unsigned failure_on_call;
static unsigned tick_timeout_model;
static uint64_t cycles;
static unsigned calls, cs_writes, wire_length;
static uint8_t wire[256];
static uint32_t cs, last_timeout, duration_us;
static HAL_StatusTypeDef response;
static uint32_t response_error;
static unsigned diagnostic_bus, frame_position, frame_offset, split_bad;
static unsigned checks;
#define CHECK(expr) do { ++checks; if (!(expr)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); exit(1); } } while (0)
static void advance_cycles(uint64_t count) {
    cycles += count; test_dwt.CYCCNT = (uint32_t)cycles;
}
uint32_t HAL_GetTick(void) { return (uint32_t)(cycles / 480000U); }
uint32_t HAL_SPI_GetError(const SPI_HandleTypeDef *spi) {
    CHECK(spi == &hspi1); return spi->ErrorCode;
}
void HAL_GPIO_WritePin(void *port, uint16_t pin, uint32_t state) {
    CHECK(port == GPIOA && pin == GPIO_PIN_4); cs = state; ++cs_writes;
    if (state == GPIO_PIN_RESET) frame_position = 0;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi,
    const uint8_t *tx, uint8_t *rx, uint16_t length, uint32_t timeout) {
    unsigned i;
    CHECK(spi == &hspi1 && tx != NULL && rx != NULL);
    CHECK(length > 0 && length <= 32 && timeout > 0 && timeout < 0xffffffffU);
    CHECK(wire_length + length <= sizeof(wire));
    ++calls; last_timeout = timeout;
    spi->ErrorCode = (!failure_on_call || calls == failure_on_call) ? response_error : 0;
    spi->State = 1;
    spi->TxXferCount = 2; spi->RxXferCount = 3;
    if (diagnostic_bus) {
        CHECK(cs == GPIO_PIN_RESET);
        if (frame_position == 0) {
            CHECK(tx[0] == 0x0b || tx[0] == 0x03); /* No write commands. */
            CHECK(tx[1] == 0 && tx[2] == 0x64);
            frame_offset = tx[0] == 0x0b ? 4 : 3;
        }
    }
    for (i = 0; i < length; ++i) {
        wire[wire_length++] = tx[i]; rx[i] = (uint8_t)(0xa0U + i);
        if (diagnostic_bus) {
            static const uint8_t data[4] = {0x21, 0x43, 0x65, 0x87};
            unsigned pos = frame_position + i;
            rx[i] = pos >= frame_offset && pos < frame_offset + 4 ? data[pos - frame_offset] : 0xee;
            if (split_bad && frame_position == 4) rx[i] = 0xff;
        }
    }
    frame_position += length;
    if (tick_timeout_model) {
        uint32_t tick_start = HAL_GetTick();
        /* Model the actual HAL >= millisecond timeout while RX is not yet
         * ready. Both successful and timeout paths keep the real adapter. */
        for (i = 0; i < duration_us; ++i) {
            if ((uint32_t)(HAL_GetTick() - tick_start) >= timeout) {
                spi->ErrorCode = HAL_SPI_ERROR_TIMEOUT;
                spi->TxXferCount = spi->RxXferCount = 0; /* HAL cleanup */
                return HAL_TIMEOUT;
            }
            advance_cycles(480U);
        }
    } else advance_cycles((uint64_t)duration_us * 480U);
    return (!failure_on_call || calls == failure_on_call) ? response : HAL_OK;
}
static void reset_bus(void) {
    calls = cs_writes = wire_length = duration_us = 0;
    response = HAL_OK; response_error = HAL_SPI_ERROR_NONE;
    failure_on_call = 0;
    tick_timeout_model = 0;
    memset(wire, 0xcc, sizeof(wire));
}
static void align_tick_phase(unsigned us) {
    advance_cycles((480000U - cycles % 480000U) % 480000U + (uint64_t)us * 480U);
}
static void test_tick_boundary_budget(const LAN9252Transport *t) {
    LAN9252Context context;
    uint8_t rx[70];
    uint32_t start;
    unsigned phase;
    LAN9252_InitContext(&context, t);
    /* Header finishes at tick phase 995 us; a normally 11-us data call must
     * not fail at the next tick only 5 us later (the observed failure mode). */
    reset_bus(); tick_timeout_model = 1; duration_us = 11;
    align_tick_phase(984);
    start = t->time_us(t->context);
    CHECK(LAN9252_ReadDirect(&context, 0x0304, rx, 4) == LAN9252_OK);
    CHECK(calls == 2 && cs == GPIO_PIN_SET && cs_writes == 2);
    CHECK(t->time_us(t->context) - start == 22);
    CHECK(context.timeout_count == 0);
    /* Sweep every integer-us start phase, including a header crossing Tick. */
    for (phase = 0; phase < 1000; ++phase) {
        reset_bus(); tick_timeout_model = 1; duration_us = 11;
        align_tick_phase(phase);
        CHECK(LAN9252_ReadDirect(&context, 0x0304, rx, 4) == LAN9252_OK);
        CHECK(calls == 2 && context.timeout_count == 0 && cs == GPIO_PIN_SET);
    }
    /* The compensated HAL guard cannot turn a late success into success,
     * reset the shared chunk budget, or introduce a retry. */
    reset_bus(); tick_timeout_model = 1; duration_us = 600;
    align_tick_phase(995); start = t->time_us(t->context);
    CHECK(t->transfer(t->context, NULL, rx, 70, 1000) == LAN9252_ERR_DIRECT_TIMEOUT);
    CHECK(calls == 2 && t->time_us(t->context) - start == 1200);
    reset_bus(); tick_timeout_model = 1; duration_us = 2500;
    align_tick_phase(995); start = t->time_us(t->context);
    CHECK(LAN9252_ReadDirect(&context, 0x0304, rx, 4) == LAN9252_ERR_DIRECT_TIMEOUT);
    CHECK(calls == 1 && cs_writes == 2 && cs == GPIO_PIN_SET);
    CHECK(t->time_us(t->context) - start == 1005);
    CHECK(context.timeout_count == 1);
    reset_bus();
}
int main(void) {
    const LAN9252Transport *t = LAN9252_BoardTransport();
    uint8_t tx[70], rx[70];
    uint32_t before;
    unsigned i;
    LAN9252Context ctx;
    hspi1.Instance = &spi_registers;
    test_tick_boundary_budget(t);
    CHECK(t && t->transfer && t->select && t->deselect && t->time_us);
    CHECK(test_debug.DEMCR & CoreDebug_DEMCR_TRCENA_Msk);
    CHECK(test_dwt.LAR == 0xc5acce55U); /* CM7 counter must be unlocked */
    CHECK(test_dwt.CTRL & DWT_CTRL_CYCCNTENA_Msk);
    before = t->time_us(t->context);
    advance_cycles(240); CHECK(t->time_us(t->context) == before);
    advance_cycles(240); CHECK(t->time_us(t->context) == before + 1U);
    before = t->time_us(t->context);
    advance_cycles(UINT64_C(4800000000)); /* crosses raw cycle wrap */
    CHECK(t->time_us(t->context) - before == 10000000U);
    before = t->time_us(t->context);
    advance_cycles(UINT64_C(0xffffffff) * 480U);
    CHECK(t->time_us(t->context) - before == 0xffffffffU);
    advance_cycles(17U * 480U);
    CHECK(t->time_us(t->context) - before == 16U); /* natural us wrap */
    for (i = 0; i < sizeof(tx); ++i) tx[i] = (uint8_t)(i + 1U);
    reset_bus(); t->select(t->context); CHECK(cs == GPIO_PIN_RESET);
    CHECK(t->transfer(t->context, tx, rx, 70, 4500) == LAN9252_OK);
    CHECK(calls == 3 && cs_writes == 1 && cs == GPIO_PIN_RESET);
    CHECK(last_timeout == 6 && wire_length == 70 && memcmp(wire, tx, 70) == 0);
    CHECK(rx[0] == 0xa0 && rx[31] == 0xbf && rx[32] == 0xa0 && rx[69] == 0xa5);
    CHECK(t->transfer(t->context, NULL, NULL, 5, 500) == LAN9252_OK);
    CHECK(last_timeout == 2 && cs_writes == 1);
    for (i = 70; i < 75; ++i) CHECK(wire[i] == 0);
    t->deselect(t->context); CHECK(cs == GPIO_PIN_SET);
    reset_bus();
    CHECK(t->transfer(t->context, NULL, rx, 5, 1000) == LAN9252_OK);
    CHECK(rx[0] == 0xa0 && rx[4] == 0xa4);
    reset_bus();
    CHECK(t->transfer(t->context, tx, NULL, 5, 1000) == LAN9252_OK);
    CHECK(memcmp(wire, tx, 5) == 0 && cs_writes == 0);
    reset_bus();
    CHECK(t->transfer(t->context, NULL, NULL, 0, 0) == LAN9252_OK && calls == 0);
    CHECK(t->transfer(t->context, tx, rx, 5, 0) == LAN9252_ERR_DIRECT_TIMEOUT && calls == 0);
    response = HAL_ERROR;
    CHECK(t->transfer(t->context, tx, rx, 5, 1000) == LAN9252_ERR_IO && calls == 1);
    response = HAL_BUSY;
    CHECK(t->transfer(t->context, tx, rx, 5, 1000) == LAN9252_ERR_IO && calls == 2);
    response = HAL_TIMEOUT;
    CHECK(t->transfer(t->context, tx, rx, 5, 1000) == LAN9252_ERR_DIRECT_TIMEOUT && calls == 3);
    CHECK(cs_writes == 0);
    reset_bus(); duration_us = 1000;
    CHECK(t->transfer(t->context, tx, rx, 70, 1000) == LAN9252_ERR_DIRECT_TIMEOUT);
    CHECK(calls == 1 && cs_writes == 0); /* late HAL_OK is a deadline failure */
    reset_bus(); duration_us = 600;
    CHECK(t->transfer(t->context, tx, rx, 70, 1000) == LAN9252_ERR_DIRECT_TIMEOUT);
    CHECK(calls == 2); /* one shared budget, never reset per chunk */
    reset_bus(); duration_us = 100;
    CHECK(t->transfer(t->context, tx, rx, 70, 2500) == LAN9252_OK);
    CHECK(last_timeout == 4); /* ceil remaining ms plus one Tick guard */
    before = t->time_us(t->context);
    advance_cycles((uint64_t)(uint32_t)(0xfffffff0U - before) * 480U);
    CHECK(t->time_us(t->context) == 0xfffffff0U);
    reset_bus(); duration_us = 100;
    CHECK(t->transfer(t->context, tx, rx, 5, 500) == LAN9252_OK);
    CHECK(t->time_us(t->context) == 84U); /* deadline subtraction crosses wrap */
    reset_bus();
    LAN9252_InitContext(&ctx, t); CHECK(cs_writes == 1 && cs == GPIO_PIN_SET);
    CHECK(LAN9252_ReadDirect(&ctx, 0x0064, rx, 4) == LAN9252_OK);
    CHECK(calls == 2 && cs_writes == 3 && cs == GPIO_PIN_SET);
    CHECK(wire_length == 8 && wire[0] == 0x0b && wire[1] == 0 && wire[2] == 0x64);
    CHECK(wire[3] == 0 && wire[7] == 0 && rx[0] == 0xa0 && rx[3] == 0xa3);
    reset_bus(); response = HAL_ERROR;
    CHECK(LAN9252_ReadDirect(&ctx, 0x0064, rx, 4) == LAN9252_ERR_IO);
    CHECK(calls == 1 && cs_writes == 2 && cs == GPIO_PIN_SET);
    LAN9252_InitContext(&ctx, t);
    reset_bus(); response = HAL_ERROR; response_error = HAL_SPI_ERROR_FLAG;
    duration_us = 1000; /* H7 EOT wait expires, then HAL returns HAL_ERROR. */
    CHECK(LAN9252_ReadDirect(&ctx, 0x0064, rx, 4) == LAN9252_ERR_DIRECT_TIMEOUT);
    CHECK(ctx.last_error == LAN9252_ERR_DIRECT_TIMEOUT && ctx.timeout_count == 1);
    CHECK(calls == 1 && cs_writes == 2 && cs == GPIO_PIN_SET);
    reset_bus(); response = HAL_ERROR; response_error = HAL_SPI_ERROR_OVR;
    CHECK(LAN9252_ReadDirect(&ctx, 0x0064, rx, 4) == LAN9252_ERR_IO);
    CHECK(ctx.last_error == LAN9252_ERR_IO && ctx.timeout_count == 1);
    CHECK(calls == 1 && cs_writes == 2 && cs == GPIO_PIN_SET);
    reset_bus(); response = HAL_ERROR; response_error = HAL_SPI_ERROR_FLAG;
    duration_us = 100; /* Millisecond tick edge can precede the DWT budget. */
    CHECK(LAN9252_ReadDirect(&ctx, 0x0064, rx, 4) == LAN9252_ERR_DIRECT_TIMEOUT);
    CHECK(ctx.timeout_count == 2 && calls == 1 && cs_writes == 2 && cs == GPIO_PIN_SET);
    reset_bus(); response = HAL_ERROR; duration_us = 1000;
    CHECK(LAN9252_ReadDirect(&ctx, 0x0064, rx, 4) == LAN9252_ERR_DIRECT_TIMEOUT);
    CHECK(ctx.timeout_count == 3 && calls == 1 && cs_writes == 2 && cs == GPIO_PIN_SET);
    reset_bus(); response = HAL_BUSY; response_error = HAL_SPI_ERROR_FLAG;
    CHECK(t->transfer(t->context, tx, rx, 5, 1000) == LAN9252_ERR_IO);
    CHECK(calls == 1 && cs_writes == 0); /* busy must not consume stale FLAG */
    reset_bus(); response = HAL_TIMEOUT;
    CHECK(LAN9252_ReadDirect(&ctx, 0x0064, rx, 4) == LAN9252_ERR_DIRECT_TIMEOUT);
    CHECK(calls == 1 && cs_writes == 2 && cs == GPIO_PIN_SET);
    /* Diagnostic regressions: literal frames, exact RX offset, HAL failure
     * validity, CS cleanup, and a split-only failure must stay distinguishable. */
    reset_bus(); diagnostic_bus = 1;
    LAN9252_BoardRunSpiDiagnostics();
    CHECK(g_lan9252_spi_debug.done == 1 && g_lan9252_spi_debug.passed_mask == 7);
    CHECK(calls == 4 && cs == GPIO_PIN_SET);
    CHECK(g_lan9252_spi_debug.fast_split.hal_calls == 2);
    CHECK(g_lan9252_spi_debug.fast_single.hal_calls == 1);
    CHECK(g_lan9252_spi_debug.normal_single.hal_calls == 1);
    CHECK(g_lan9252_spi_debug.fast_split.value == 0x87654321U);
    CHECK(g_lan9252_spi_debug.fast_single.value == 0x87654321U);
    CHECK(g_lan9252_spi_debug.normal_single.value == 0x87654321U);
    CHECK(g_lan9252_spi_debug.fast_split.rx[4] == 0x21);
    CHECK(g_lan9252_spi_debug.normal_single.rx[3] == 0x21);
    CHECK(g_lan9252_spi_debug.normal_single.hal_status[1] == 0xffffffffU);
    {
        static const uint8_t frames[23] = {
            0x0b,0,0x64,0,0,0,0,0, 0x0b,0,0x64,0,0,0,0,0,
            0x03,0,0x64,0,0,0,0
        };
        CHECK(wire_length == 23 && memcmp(wire, frames, 23) == 0);
    }
    reset_bus(); split_bad = 1;
    LAN9252_BoardRunSpiDiagnostics();
    CHECK(g_lan9252_spi_debug.passed_mask == 6);
    CHECK(g_lan9252_spi_debug.fast_split.value == 0xffffffffU);
    CHECK(g_lan9252_spi_debug.fast_split.value_valid == 1);
    split_bad = 0;
    reset_bus(); response = HAL_TIMEOUT; response_error = HAL_SPI_ERROR_FLAG;
    LAN9252_BoardRunSpiDiagnostics();
    CHECK(g_lan9252_spi_debug.done == 1 && g_lan9252_spi_debug.passed_mask == 0);
    CHECK(calls == 3 && cs == GPIO_PIN_SET); /* Split data phase not attempted. */
    CHECK(g_lan9252_spi_debug.fast_split.hal_calls == 1);
    CHECK(g_lan9252_spi_debug.fast_split.hal_status[0] == HAL_TIMEOUT);
    CHECK(g_lan9252_spi_debug.fast_split.hal_status[1] == 0xffffffffU);
    CHECK(g_lan9252_spi_debug.fast_split.hal_error[0] == HAL_SPI_ERROR_FLAG);
    CHECK(!g_lan9252_spi_debug.fast_split.value_valid);
    CHECK(!g_lan9252_spi_debug.fast_single.value_valid);
    CHECK(!g_lan9252_spi_debug.normal_single.value_valid);
    reset_bus(); response = HAL_ERROR; response_error = HAL_SPI_ERROR_OVR;
    LAN9252_BoardRunSpiDiagnostics();
    CHECK(g_lan9252_spi_debug.fast_single.hal_status[0] == HAL_ERROR);
    CHECK(g_lan9252_spi_debug.fast_single.hal_error[0] == HAL_SPI_ERROR_OVR);
    CHECK(g_lan9252_spi_debug.passed_mask == 0 && cs == GPIO_PIN_SET);
    reset_bus(); response = HAL_BUSY; response_error = HAL_SPI_ERROR_FLAG;
    LAN9252_BoardRunSpiDiagnostics();
    CHECK(g_lan9252_spi_debug.normal_single.hal_status[0] == HAL_BUSY);
    CHECK(!g_lan9252_spi_debug.normal_single.value_valid);
    CHECK(g_lan9252_spi_debug.passed_mask == 0 && cs == GPIO_PIN_SET);
    diagnostic_bus = 0;
    /* Catch a missing latch, mislabelled HAL path, or later overwrite. Only
     * reset the debug storage here to model a new MCU boot between cases. */
    memset((void *)&g_lan9252_hal_fault, 0, sizeof(g_lan9252_hal_fault));
    memset((void *)&g_lan9252_first_fault, 0, sizeof(g_lan9252_first_fault));
    reset_bus();
    CHECK(LAN9252_ReadDirect(&ctx, 0x64, rx, 4) == LAN9252_OK);
    CHECK(!g_lan9252_first_fault.valid && !g_lan9252_hal_fault.valid);
    reset_bus(); duration_us = 100;
    response = HAL_ERROR; response_error = HAL_SPI_ERROR_FLAG; failure_on_call = 2;
    spi_registers.SR = 0x1234; spi_registers.CR1 = 0x1000;
    spi_registers.CR2 = 4; spi_registers.CFG1 = 0x30070007;
    spi_registers.CFG2 = 0x84400000;
    CHECK(LAN9252_ReadDirect(&ctx, 0x0304, rx, 4) == LAN9252_ERR_DIRECT_TIMEOUT);
    CHECK(g_lan9252_first_fault.valid == 1);
    CHECK(g_lan9252_first_fault.address == 0x0304);
    CHECK(g_lan9252_first_fault.command == 0x0b && g_lan9252_first_fault.phase == 2);
    CHECK(g_lan9252_first_fault.reason == 2 && g_lan9252_first_fault.status == 3);
    CHECK(g_lan9252_first_fault.elapsed_us == 200);
    CHECK(g_lan9252_hal_fault.valid == 1 && g_lan9252_hal_fault.reason == 3);
    CHECK(g_lan9252_hal_fault.hal_status == HAL_ERROR);
    CHECK(g_lan9252_hal_fault.hal_error == HAL_SPI_ERROR_FLAG);
    CHECK(g_lan9252_hal_fault.elapsed_us == 100);
    CHECK(g_lan9252_hal_fault.timeout_us == 900 && g_lan9252_hal_fault.timeout_ms == 2);
    CHECK(g_lan9252_hal_fault.chunk_length == 4);
    CHECK(g_lan9252_hal_fault.spi_sr == 0x1234 && g_lan9252_hal_fault.spi_cr1 == 0x1000);
    CHECK(g_lan9252_hal_fault.spi_cr2 == 4 && g_lan9252_hal_fault.spi_cfg1 == 0x30070007);
    CHECK(g_lan9252_hal_fault.spi_cfg2 == 0x84400000);
    CHECK(g_lan9252_hal_fault.hal_state == 1);
    CHECK(g_lan9252_hal_fault.tx_remaining == 2 && g_lan9252_hal_fault.rx_remaining == 3);
    CHECK(calls == 2 && cs_writes == 2 && cs == GPIO_PIN_SET); /* No retry. */
    reset_bus(); spi_registers.SR = 0;
    CHECK(LAN9252_ReadDirect(&ctx, 0x64, rx, 4) == LAN9252_OK);
    response = HAL_TIMEOUT;
    CHECK(LAN9252_ReadDirect(&ctx, 0x50, rx, 4) == LAN9252_ERR_DIRECT_TIMEOUT);
    CHECK(g_lan9252_first_fault.address == 0x0304 && g_lan9252_first_fault.elapsed_us == 200);
    CHECK(g_lan9252_hal_fault.reason == 3 && g_lan9252_hal_fault.spi_sr == 0x1234);
    {
        /* Independently check every adapter return path, including the
         * observed sub-ms HAL timeout and HAL_OK finishing too late. */
        static const struct { HAL_StatusTypeDef hal; uint32_t error, us, budget, reason; LAN9252Status want; } rows[] = {
            {HAL_TIMEOUT, HAL_SPI_ERROR_TIMEOUT, 410, 1000, 2, LAN9252_ERR_DIRECT_TIMEOUT},
            {HAL_OK, 0, 1000, 1000, 4, LAN9252_ERR_DIRECT_TIMEOUT},
            {HAL_ERROR, HAL_SPI_ERROR_OVR, 100, 1000, 5, LAN9252_ERR_IO},
            {HAL_BUSY, HAL_SPI_ERROR_FLAG, 100, 1000, 5, LAN9252_ERR_IO},
            {HAL_OK, 0, 0, 0, 1, LAN9252_ERR_DIRECT_TIMEOUT}
        };
        for (i = 0; i < sizeof(rows)/sizeof(rows[0]); ++i) {
            memset((void *)&g_lan9252_hal_fault, 0, sizeof(g_lan9252_hal_fault));
            memset((void *)&g_lan9252_first_fault, 0, sizeof(g_lan9252_first_fault));
            reset_bus(); response = rows[i].hal; response_error = rows[i].error; duration_us = rows[i].us;
            CHECK(t->transfer(t->context, tx, rx, 4, rows[i].budget) == rows[i].want);
            CHECK(g_lan9252_hal_fault.valid == 1 && g_lan9252_hal_fault.reason == rows[i].reason);
            CHECK(g_lan9252_hal_fault.elapsed_us == rows[i].us);
            CHECK(g_lan9252_hal_fault.hal_status == (rows[i].budget ? (uint32_t)rows[i].hal : 0xffffffffU));
            CHECK(g_lan9252_hal_fault.hal_error == (rows[i].budget ? rows[i].error : 0xffffffffU));
            CHECK(cs_writes == 0); /* Snapshot never controls CS. */
        }
    }
    /* A later adapter failure cannot masquerade as HAL evidence for an
     * earlier driver-only deadline that never failed inside the adapter. */
    memset((void *)&g_lan9252_hal_fault, 0, sizeof(g_lan9252_hal_fault));
    g_lan9252_first_fault.valid = 1;
    reset_bus(); response = HAL_TIMEOUT;
    CHECK(t->transfer(t->context, tx, rx, 4, 1000) == LAN9252_ERR_DIRECT_TIMEOUT);
    CHECK(g_lan9252_hal_fault.valid == 0);
    printf("test_lan9252_board: PASS (%u checks)\n", checks);
    return 0;
}
