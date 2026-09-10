/**
 * @file foc_transform.c
 * @brief 幅值不变 Clarke/Park 变换，与现有 SVPWM 逆 Park 方向配对。
 */
#include "foc_transform.h"

#include <math.h>

#define FOC_ONE_BY_SQRT3_F (0.5773502691896258f)
#define FOC_TWO_PI_F       (6.2831853071795865f)

bool Foc_Clarke(const CurrentPhaseCurrents *phase, FocAlphaBeta *alpha_beta)
{
    if ((phase == 0) || (alpha_beta == 0) ||
        !isfinite(phase->ia_a) || !isfinite(phase->ib_a) || !isfinite(phase->ic_a))
    {
        return false;
    }

    /* 两采样电阻形式：alpha=Ia，beta=(Ia+2Ib)/sqrt(3)。 */
    alpha_beta->alpha = phase->ia_a;
    alpha_beta->beta = (phase->ia_a + (2.0f * phase->ib_a)) * FOC_ONE_BY_SQRT3_F;
    return isfinite(alpha_beta->alpha) && isfinite(alpha_beta->beta);
}

bool Foc_Park(const FocAlphaBeta *alpha_beta, float electrical_angle_pu, FocDq *dq)
{
    float angle_pu;
    float angle_rad;
    float sin_angle;
    float cos_angle;

    if ((alpha_beta == 0) || (dq == 0) ||
        !isfinite(alpha_beta->alpha) || !isfinite(alpha_beta->beta) ||
        !isfinite(electrical_angle_pu))
    {
        return false;
    }

    angle_pu = fmodf(electrical_angle_pu, 1.0f);
    if (angle_pu < 0.0f)
    {
        angle_pu += 1.0f;
    }

    angle_rad = angle_pu * FOC_TWO_PI_F;
    sin_angle = sinf(angle_rad);
    cos_angle = cosf(angle_rad);

    dq->d = (alpha_beta->alpha * cos_angle) + (alpha_beta->beta * sin_angle);
    dq->q = (-alpha_beta->alpha * sin_angle) + (alpha_beta->beta * cos_angle);
    return isfinite(dq->d) && isfinite(dq->q);
}
