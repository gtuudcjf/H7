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
static uint64_t cycles;
static unsigned calls, cs_writes, wire_length;
static uint8_t wire[256];
static uint32_t cs, last_timeout, duration_us;
static HAL_StatusTypeDef response;
static uint32_t response_error;
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
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi,
    const uint8_t *tx, uint8_t *rx, uint16_t length, uint32_t timeout) {
    unsigned i;
    CHECK(spi == &hspi1 && tx != NULL && rx != NULL);
    CHECK(length > 0 && length <= 32 && timeout > 0 && timeout < 0xffffffffU);
    CHECK(wire_length + length <= sizeof(wire));
    ++calls; last_timeout = timeout;
    spi->ErrorCode = response_error;
    for (i = 0; i < length; ++i) {
        wire[wire_length++] = tx[i]; rx[i] = (uint8_t)(0xa0U + i);
    }
    advance_cycles((uint64_t)duration_us * 480U);
    return response;
}
static void reset_bus(void) {
    calls = cs_writes = wire_length = duration_us = 0;
    response = HAL_OK; response_error = HAL_SPI_ERROR_NONE;
    memset(wire, 0xcc, sizeof(wire));
}
int main(void) {
    const LAN9252Transport *t = LAN9252_BoardTransport();
    uint8_t tx[70], rx[70];
    uint32_t before;
    unsigned i;
    LAN9252Context ctx;
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
    CHECK(last_timeout == 4 && wire_length == 70 && memcmp(wire, tx, 70) == 0);
    CHECK(rx[0] == 0xa0 && rx[31] == 0xbf && rx[32] == 0xa0 && rx[69] == 0xa5);
    CHECK(t->transfer(t->context, NULL, NULL, 5, 500) == LAN9252_OK);
    CHECK(last_timeout == 1 && cs_writes == 1);
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
    CHECK(last_timeout == 2); /* floor remaining time; never round up */
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
    printf("test_lan9252_board: PASS (%u checks)\n", checks);
    return 0;
}
