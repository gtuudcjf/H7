/**
 * @file test_biss_frame.c
 * @brief 17 位 BiSS-C 帧同步、状态位与 CRC6 的主机侧单元测试。
 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "biss_frame.h"

static void SetBit(uint8_t raw[BISS_FRAME_RAW_BYTES], uint8_t bit_index, uint8_t value)
{
    const uint8_t byte_index = (uint8_t)(bit_index / 8U);
    const uint8_t mask = (uint8_t)(1U << (7U - (bit_index % 8U)));

    if (value != 0U)
    {
        raw[byte_index] |= mask;
    }
    else
    {
        raw[byte_index] &= (uint8_t)~mask;
    }
}

static void WriteBits(uint8_t raw[BISS_FRAME_RAW_BYTES],
                      uint8_t *bit_index,
                      uint32_t value,
                      uint8_t bit_count)
{
    uint8_t index;

    for (index = 0U; index < bit_count; ++index)
    {
        const uint8_t shift = (uint8_t)(bit_count - 1U - index);
        SetBit(raw, *bit_index, (uint8_t)((value >> shift) & 1U));
        ++(*bit_index);
    }
}

static void BuildFrame(uint32_t position,
                       bool error_ok,
                       bool warning_ok,
                       uint8_t leading_idle_bits,
                       uint8_t raw[BISS_FRAME_RAW_BYTES])
{
    uint8_t bit_index = leading_idle_bits;
    uint32_t payload;
    uint8_t crc;

    memset(raw, 0xFF, BISS_FRAME_RAW_BYTES);
    WriteBits(raw, &bit_index, 0U, BISS_ACK_BITS);
    WriteBits(raw, &bit_index, 1U, 1U); /* Start */
    WriteBits(raw, &bit_index, 0U, 1U); /* CDS */
    WriteBits(raw, &bit_index, position, BISS_POSITION_BITS);
    WriteBits(raw, &bit_index, error_ok ? 1U : 0U, 1U);
    WriteBits(raw, &bit_index, warning_ok ? 1U : 0U, 1U);

    payload = (position << 2U) |
              (error_ok ? 2U : 0U) |
              (warning_ok ? 1U : 0U);
    crc = BissFrame_CalculateCrc6(payload, 19U);
    WriteBits(raw, &bit_index, crc, BISS_CRC_BITS);
}

static void Test_CrcMatchesRlsKnownAnswers(void)
{
    /* 19 bits: position[16:0], active-low Error, active-low Warning. */
    assert(BissFrame_CalculateCrc6(0x00003U, 19U) == 0x3AU);
    assert(BissFrame_CalculateCrc6(0x48D17U, 19U) == 0x0DU);
    assert(BissFrame_CalculateCrc6(0x7FFFFU, 19U) == 0x20U);
}

static void Test_ParsesValidPositionsAtDifferentResponseOffsets(void)
{
    static const uint32_t positions[] = {0U, 0x12345U, BISS_POSITION_MAX};
    uint8_t raw[BISS_FRAME_RAW_BYTES];
    BissFrame17 frame;
    unsigned int index;

    for (index = 0U; index < (sizeof(positions) / sizeof(positions[0])); ++index)
    {
        BuildFrame(positions[index], true, true, (uint8_t)index, raw);
        assert(BissFrame_Parse17(raw, &frame));
        assert(frame.status == BISS_FRAME_OK);
        assert(frame.position_raw == positions[index]);
        assert(frame.error_ok);
        assert(frame.warning_ok);
        assert(frame.payload_bit_index == (uint8_t)(index + BISS_ACK_BITS + 2U));
    }
}

static void Test_WarningDoesNotInvalidatePosition(void)
{
    uint8_t raw[BISS_FRAME_RAW_BYTES];
    BissFrame17 frame;

    BuildFrame(0x12345U, true, false, 2U, raw);
    assert(BissFrame_Parse17(raw, &frame));
    assert(frame.status == BISS_FRAME_OK);
    assert(!frame.warning_ok);
}

static void Test_EncoderErrorAndCrcErrorAreRejected(void)
{
    uint8_t raw[BISS_FRAME_RAW_BYTES];
    BissFrame17 frame;

    BuildFrame(0x15555U, false, true, 1U, raw);
    assert(!BissFrame_Parse17(raw, &frame));
    assert(frame.status == BISS_FRAME_ENCODER_ERROR);

    BuildFrame(0x15555U, true, true, 1U, raw);
    /* lead(1)+ACK(13)+Start/CDS(2)+payload(19) 后的首个 CRC 位。 */
    raw[4] ^= 0x10U;
    assert(!BissFrame_Parse17(raw, &frame));
    assert(frame.status == BISS_FRAME_CRC_ERROR);
}

static void Test_BrokenAckAndStartAreRejected(void)
{
    uint8_t raw[BISS_FRAME_RAW_BYTES];
    BissFrame17 frame;

    BuildFrame(0x15555U, true, true, 0U, raw);
    SetBit(raw, 6U, 1U);
    assert(!BissFrame_Parse17(raw, &frame));
    assert(frame.status == BISS_FRAME_SYNC_ERROR);

    BuildFrame(0x15555U, true, true, 0U, raw);
    SetBit(raw, BISS_ACK_BITS, 0U);
    assert(!BissFrame_Parse17(raw, &frame));
    assert(frame.status == BISS_FRAME_SYNC_ERROR);
}

static void Test_BadArgumentsAreRejected(void)
{
    uint8_t raw[BISS_FRAME_RAW_BYTES] = {0U};
    BissFrame17 frame;

    assert(!BissFrame_Parse17(NULL, &frame));
    assert(frame.status == BISS_FRAME_BAD_ARGUMENT);
    assert(!BissFrame_Parse17(raw, NULL));
    assert(BissFrame_CalculateCrc6(0U, 0U) == 0U);
    assert(BissFrame_CalculateCrc6(0U, 33U) == 0U);
}

int main(void)
{
    Test_CrcMatchesRlsKnownAnswers();
    Test_ParsesValidPositionsAtDifferentResponseOffsets();
    Test_WarningDoesNotInvalidatePosition();
    Test_EncoderErrorAndCrcErrorAreRejected();
    Test_BrokenAckAndStartAreRejected();
    Test_BadArgumentsAreRejected();

    puts("BiSS frame tests passed");
    return 0;
}
