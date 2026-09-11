/**
 * @file encoder_angle.h
 * @brief 将有效的 17 位单圈位置转换为 FOC 使用的归一化电角度。
 */
#ifndef ENCODER_ANGLE_H
#define ENCODER_ANGLE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t zero_raw;
    int8_t direction;
    uint8_t pole_pairs;
} EncoderAngleConfig;

typedef struct
{
    uint32_t position_raw;
    float mechanical_angle_pu;
    float electrical_angle_pu;
} EncoderAngleSample;

/**
 * @brief 配置编码器零点、正方向和电机极对数。
 * @param direction 只能为 +1 或 -1。
 */
bool EncoderAngle_Init(EncoderAngleConfig *config,
                       uint32_t zero_raw,
                       int8_t direction,
                       uint8_t pole_pairs);

/**
 * @brief 计算相对零点的机械角和电角度，结果范围均为 [0, 1)。
 */
bool EncoderAngle_Update(const EncoderAngleConfig *config,
                         uint32_t position_raw,
                         EncoderAngleSample *sample);

#endif /* ENCODER_ANGLE_H */
