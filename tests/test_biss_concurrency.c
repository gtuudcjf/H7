#include <assert.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "biss_encoder.h"

/* Keep the real CRC/parser; inject a higher-priority control interrupt at
 * its entry. Include transport verbatim to intercept only its memcpy calls
 * and schedule a reader at the shared raw-frame publication boundary. */
#define BissFrame_Parse17 ParseRealFrame
#include "../Core/Src/motor/biss_frame.c"
#undef BissFrame_Parse17
static void *TransportMemcpy(void *destination, const void *source, size_t count);
#define memcpy TransportMemcpy
#include "../Core/Src/motor/biss_encoder.c"
#undef memcpy

static uint32_t test_primask, abort_count, reader_count;
static uint8_t *receive_buffer, *transmit_buffer;
static bool inject_parser, probe_publication, reader_pending, mixed_snapshot;
static HAL_StatusTypeDef injected_start, next_start_result = HAL_OK;
static HAL_StatusTypeDef next_abort_result = HAL_OK;

static void ReadFromControlInterrupt(void)
{
    BissEncoderSnapshot snapshot;
    BissFrame17 frame;
    reader_pending = false;
    ++reader_count;
    assert(BissEncoder_GetSnapshot(&snapshot));
    if (!ParseRealFrame(snapshot.raw, &frame) ||
        frame.position_raw != snapshot.position_raw ||
        snapshot.valid_age_ticks != 0U || !snapshot.ready)
        mixed_snapshot = true;
}

uint32_t __get_PRIMASK(void) { return test_primask; }
void __disable_irq(void) { test_primask = 1U; }
void __enable_irq(void)
{
    test_primask = 0U;
    if (reader_pending) ReadFromControlInterrupt();
}

static void *TransportMemcpy(void *destination, const void *source, size_t count)
{
    size_t index;
    for (index = 0U; index < count; ++index)
        ((uint8_t *)destination)[index] = ((const uint8_t *)source)[index];
    if (probe_publication && count == BISS_FRAME_RAW_BYTES)
    {
        probe_publication = false;
        reader_pending = true;
        if (test_primask == 0U) ReadFromControlInterrupt();
    }
    return destination;
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *spi, uint8_t *tx,
                                             uint8_t *rx, uint16_t length)
{
    HAL_StatusTypeDef result = next_start_result;
    assert(spi->Instance == SPI4 && tx != NULL && length == 6U);
    next_start_result = HAL_OK;
    receive_buffer = rx;
    transmit_buffer = tx;
    return result;
}

HAL_StatusTypeDef HAL_SPI_Abort_IT(SPI_HandleTypeDef *spi)
{
    HAL_StatusTypeDef result = next_abort_result;
    assert(spi->Instance == SPI4);
    ++abort_count;
    next_abort_result = HAL_OK;
    return result;
}

bool BissFrame_Parse17(const uint8_t *raw, BissFrame17 *frame)
{
    if (inject_parser)
    {
        uint32_t index;
        BissEncoderSnapshot snapshot;
        inject_parser = false;
        assert(test_primask == 0U); /* CRC work must remain interruptible. */
        for (index = 0U; index < BISS_ENCODER_DMA_TIMEOUT_TICKS + 2U; ++index)
            BissEncoder_ControlTick();
        assert(BissEncoder_GetSnapshot(&snapshot));
        assert(snapshot.valid_age_ticks == BISS_ENCODER_DMA_TIMEOUT_TICKS + 2U);
        assert(snapshot.timeout_count == 0U && abort_count == 0U);
        assert(snapshot.busy); /* DMA or parser still owns the frame. */
        injected_start = BissEncoder_StartRead();
        /* If ownership was released early, model DMA independently writing
         * a new position byte before its new CRC byte arrives. */
        if (injected_start == HAL_OK) receive_buffer[2] ^= 1U;
    }
    return ParseRealFrame(raw, frame);
}

static void FillFrame(uint32_t position)
{
    uint32_t index;
    uint8_t crc = BissFrame_CalculateCrc6((position << 2U) | 3U, 19U);
    uint64_t bits = ((uint64_t)1U << 34U) | ((uint64_t)position << 16U) |
                    ((uint64_t)3U << 14U) | ((uint64_t)crc << 8U) | 0xFFU;
    for (index = 0U; index < 6U; ++index)
        receive_buffer[index] = (uint8_t)(bits >> ((5U - index) * 8U));
}

static void CompleteValid(uint32_t position)
{
    assert(BissEncoder_StartRead() == HAL_OK);
    FillFrame(position);
    BissEncoder_OnTransferComplete();
}

static void PrepareReady(void)
{
    static SPI_HandleTypeDef spi = {SPI4, 0U};
    uint32_t index;
    assert(BissEncoder_Init(&spi) == HAL_OK);
    for (index = 0U; index < BISS_ENCODER_READY_FRAME_COUNT; ++index)
        CompleteValid(0x1234U);
}

static void TestOwnership(void)
{
    BissEncoderSnapshot snapshot;
    PrepareReady();
    inject_parser = true;
    CompleteValid(0x1234U);
    assert(injected_start == HAL_BUSY);
    assert(BissEncoder_GetSnapshot(&snapshot));
    assert(snapshot.valid_count == 33U && snapshot.position_raw == 0x1234U);
    assert(snapshot.crc_error_count == 0U && snapshot.valid_age_ticks == 0U);
    assert(!snapshot.busy && snapshot.ready && test_primask == 0U);
    CompleteValid(0x1334U); /* Ownership is released after publication. */
}

static void TestPublication(void)
{
    PrepareReady();
    probe_publication = true;
    CompleteValid(0x1334U);
    assert(reader_count == 1U && !reader_pending && !mixed_snapshot);
    assert(test_primask == 0U);
}

static void TestRecovery(void)
{
    BissEncoderSnapshot snapshot;
    uint32_t index;
    PrepareReady();
    assert(BissEncoder_StartRead() == HAL_OK);
    FillFrame(0x1334U);
    receive_buffer[2] ^= 1U;
    BissEncoder_OnTransferComplete();
    assert(BissEncoder_GetSnapshot(&snapshot));
    assert(snapshot.crc_error_count == 1U && snapshot.position_raw == 0x1234U);
    assert(snapshot.valid_count == 32U && !snapshot.busy && !snapshot.ready);
    CompleteValid(0x1334U);
    assert(BissEncoder_StartRead() == HAL_OK);
    receive_buffer[BISS_FRAME_RAW_BYTES] = 0U;
    BissEncoder_OnTransferComplete();
    assert(BissEncoder_GetSnapshot(&snapshot));
    assert(snapshot.dma_guard_error_count == 1U && !snapshot.busy);
    CompleteValid(0x1434U);
    assert(BissEncoder_StartRead() == HAL_OK);
    BissEncoder_OnTransferError(17U);
    assert(BissEncoder_GetSnapshot(&snapshot));
    assert(snapshot.spi_error_count == 1U && snapshot.last_spi_error == 17U);
    assert(!snapshot.busy && !snapshot.ready);
    CompleteValid(0x1534U);
    assert(BissEncoder_StartRead() == HAL_OK);
    BissEncoder_OnAbortComplete();
    CompleteValid(0x1634U);
    next_start_result = HAL_ERROR;
    assert(BissEncoder_StartRead() == HAL_ERROR);
    CompleteValid(0x1734U);
    test_primask = 1U;
    CompleteValid(0x1834U);
    assert(BissEncoder_GetSnapshot(&snapshot) && test_primask == 1U);
    test_primask = 0U;
    PrepareReady();
    assert(BissEncoder_GetSnapshot(&snapshot) && snapshot.ready && !snapshot.busy);
    /* StartRead's guard failure must break the consecutive-valid streak too. */
    transmit_buffer[BISS_FRAME_RAW_BYTES] = 0U;
    assert(BissEncoder_StartRead() == HAL_ERROR && test_primask == 0U);
    CompleteValid(0x1934U);
    assert(BissEncoder_GetSnapshot(&snapshot));
    assert(snapshot.dma_guard_error_count == 1U && !snapshot.ready && !snapshot.busy);
    next_start_result = HAL_BUSY;
    test_primask = 1U;
    assert(BissEncoder_StartRead() == HAL_BUSY && test_primask == 1U);
    assert(BissEncoder_GetSnapshot(&snapshot) && !snapshot.busy);
    test_primask = 0U;
    CompleteValid(0x1A34U);
    assert(BissEncoder_StartRead() == HAL_OK);
    next_abort_result = HAL_ERROR;
    for (index = 0U; index <= BISS_ENCODER_DMA_TIMEOUT_TICKS; ++index)
        BissEncoder_ControlTick();
    assert(BissEncoder_GetSnapshot(&snapshot));
    assert(snapshot.timeout_count == 1U && snapshot.spi_error_count == 1U && !snapshot.busy);
    CompleteValid(0x1B34U);
}

int main(int argc, char **argv)
{
    assert(BissEncoder_StartRead() == HAL_ERROR && test_primask == 0U);
    if (argc == 1 || strcmp(argv[1], "ownership") == 0) TestOwnership();
    if (argc == 1 || strcmp(argv[1], "publication") == 0) TestPublication();
    if (argc == 1 || strcmp(argv[1], "recovery") == 0) TestRecovery();
    puts("BiSS buffer ownership/publication/recovery tests passed");
    return 0;
}
