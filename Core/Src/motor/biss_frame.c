/**
 * @file biss_frame.c
 * @brief RLS 17 位 BiSS-C 帧定位和 CRC6 校验。
 */
#include "biss_frame.h"

#include <string.h>

#define BISS_STATUS_BITS            (2U)
#define BISS_START_BITS             (1U)
#define BISS_CDS_BITS               (1U)
#define BISS_PAYLOAD_BITS           (BISS_POSITION_BITS + BISS_STATUS_BITS)
#define BISS_RESPONSE_BITS          \
    (BISS_ACK_BITS + BISS_START_BITS + BISS_CDS_BITS + \
     BISS_PAYLOAD_BITS + BISS_CRC_BITS)

static uint8_t BissFrame_GetBit(const uint8_t raw[BISS_FRAME_RAW_BYTES],
                                uint8_t bit_index)
{
    const uint8_t byte_index = (uint8_t)(bit_index / 8U);
    const uint8_t shift = (uint8_t)(7U - (bit_index % 8U));

    return (uint8_t)((raw[byte_index] >> shift) & 1U);
}

static uint32_t BissFrame_GetBits(const uint8_t raw[BISS_FRAME_RAW_BYTES],
                                  uint8_t bit_index,
                                  uint8_t bit_count)
{
    uint32_t value = 0U;
    uint8_t index;

    for (index = 0U; index < bit_count; ++index)
    {
        value = (value << 1U) | BissFrame_GetBit(raw, (uint8_t)(bit_index + index));
    }
    return value;
}

uint8_t BissFrame_CalculateCrc6(uint32_t payload, uint8_t bit_count)
{
    uint8_t remainder = 0U;
    uint8_t index;

    if ((bit_count == 0U) || (bit_count > 32U))
    {
        return 0U;
    }

    /*
     * 多项式为 x^6+x+1。0x43 中最高项是隐含的，因此在六位余数
     * 寄存器中反馈的低阶部分是 0x03。RLS 编码器在线上传输其反码。
     */
    for (index = 0U; index < bit_count; ++index)
    {
        const uint8_t input_bit =
            (uint8_t)((payload >> (bit_count - 1U - index)) & 1U);
        const uint8_t feedback = (uint8_t)(((remainder >> 5U) & 1U) ^ input_bit);

        remainder = (uint8_t)((remainder << 1U) & 0x3FU);
        if (feedback != 0U)
        {
            remainder ^= 0x03U;
        }
    }

    return (uint8_t)((~remainder) & 0x3FU);
}

bool BissFrame_Parse17(const uint8_t raw[BISS_FRAME_RAW_BYTES], BissFrame17 *frame)
{
    uint8_t ack_start;
    uint8_t zero_index;
    uint8_t payload_index = 0U;
    uint32_t position;
    uint32_t payload;
    uint8_t received_crc;
    uint8_t calculated_crc;
    bool error_ok;
    bool warning_ok;
    bool sync_found = false;

    if (frame == 0)
    {
        return false;
    }

    memset(frame, 0, sizeof(*frame));
    frame->status = BISS_FRAME_BAD_ARGUMENT;
    if (raw == 0)
    {
        return false;
    }
    memcpy(frame->raw, raw, BISS_FRAME_RAW_BYTES);

    /* 最长响应为 40 位，因此在 48 位窗口内最多允许 8 位前导空闲。 */
    for (ack_start = 0U;
         ack_start <= (BISS_FRAME_RAW_BITS - BISS_RESPONSE_BITS);
         ++ack_start)
    {
        bool ack_is_low = true;

        if ((ack_start > 0U) && (BissFrame_GetBit(raw, (uint8_t)(ack_start - 1U)) == 0U))
        {
            continue;
        }

        for (zero_index = 0U; zero_index < BISS_ACK_BITS; ++zero_index)
        {
            if (BissFrame_GetBit(raw, (uint8_t)(ack_start + zero_index)) != 0U)
            {
                ack_is_low = false;
                break;
            }
        }

        if (ack_is_low &&
            (BissFrame_GetBit(raw, (uint8_t)(ack_start + BISS_ACK_BITS)) != 0U))
        {
            payload_index = (uint8_t)(ack_start + BISS_ACK_BITS +
                                      BISS_START_BITS + BISS_CDS_BITS);
            sync_found = true;
            break;
        }
    }

    if (!sync_found)
    {
        frame->status = BISS_FRAME_SYNC_ERROR;
        return false;
    }

    position = BissFrame_GetBits(raw, payload_index, BISS_POSITION_BITS);
    error_ok = BissFrame_GetBit(raw, (uint8_t)(payload_index + BISS_POSITION_BITS)) != 0U;
    warning_ok = BissFrame_GetBit(
                     raw,
                     (uint8_t)(payload_index + BISS_POSITION_BITS + 1U)) != 0U;
    received_crc = (uint8_t)BissFrame_GetBits(
        raw,
        (uint8_t)(payload_index + BISS_PAYLOAD_BITS),
        BISS_CRC_BITS);
    payload = (position << BISS_STATUS_BITS) |
              (error_ok ? 2U : 0U) |
              (warning_ok ? 1U : 0U);
    calculated_crc = BissFrame_CalculateCrc6(payload, BISS_PAYLOAD_BITS);

    frame->payload_bit_index = payload_index;
    frame->received_crc = received_crc;
    frame->calculated_crc = calculated_crc;
    frame->error_ok = error_ok;
    frame->warning_ok = warning_ok;

    if (!error_ok)
    {
        frame->status = BISS_FRAME_ENCODER_ERROR;
        return false;
    }
    if (received_crc != calculated_crc)
    {
        frame->status = BISS_FRAME_CRC_ERROR;
        return false;
    }

    frame->position_raw = position;
    frame->status = BISS_FRAME_OK;
    return true;
}
