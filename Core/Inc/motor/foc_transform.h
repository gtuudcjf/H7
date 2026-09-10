/**
 * @file foc_transform.h
 * @brief 与硬件无关的 Clarke/Park 坐标变换。
 */
#ifndef FOC_TRANSFORM_H
#define FOC_TRANSFORM_H

#include <stdbool.h>

#include "current_sense.h"

typedef struct
{
    float alpha;
    float beta;
} FocAlphaBeta;

typedef struct
{
    float d;
    float q;
} FocDq;

bool Foc_Clarke(const CurrentPhaseCurrents *phase, FocAlphaBeta *alpha_beta);
bool Foc_Park(const FocAlphaBeta *alpha_beta, float electrical_angle_pu, FocDq *dq);

#endif /* FOC_TRANSFORM_H */
