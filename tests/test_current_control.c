/**
 * @file test_current_control.c
 * @brief 电流采样、FOC 坐标变换和电流 PI 的主机侧单元测试。
 */
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "current_sense.h"
#include "foc_transform.h"
#include "motor_params.h"

#define TEST_EPSILON (0.0002f)

static void AssertNear(float actual, float expected, float tolerance)
{
    assert(fabsf(actual - expected) <= tolerance);
}

static CurrentSenseConfig DefaultSenseConfig(void)
{
    CurrentSenseConfig config;

    config.adc_reference_v = MOTOR_ADC_REFERENCE_V;
    config.adc_full_scale_count = MOTOR_ADC_FULL_SCALE_COUNT;
    config.shunt_resistance_ohm = MOTOR_SHUNT_RESISTANCE_OHM;
    config.amplifier_gain_v_per_v = MOTOR_CSA_GAIN_V_PER_V;
    config.phase_a_polarity = MOTOR_CURRENT_IA_POLARITY;
    config.phase_b_polarity = MOTOR_CURRENT_IB_POLARITY;
    return config;
}

static void Test_CurrentSenseZeroAtCalibratedOffset(void)
{
    const CurrentSenseConfig config = DefaultSenseConfig();
    const CurrentSenseOffsets offsets = {2048.0f, 2048.0f};
    CurrentPhaseCurrents current;

    assert(CurrentSense_Convert(&config, &offsets, 2048U, 2048U, &current));
    AssertNear(current.ia_a, 0.0f, TEST_EPSILON);
    AssertNear(current.ib_a, 0.0f, TEST_EPSILON);
    AssertNear(current.ic_a, 0.0f, TEST_EPSILON);
}

static void Test_CurrentSenseConvertsAmperesAndReconstructsPhaseC(void)
{
    const CurrentSenseConfig config = DefaultSenseConfig();
    const CurrentSenseOffsets offsets = {2048.0f, 2048.0f};
    CurrentPhaseCurrents current;
    const float amp_per_count = MOTOR_CURRENT_A_PER_COUNT;

    assert(CurrentSense_Convert(&config, &offsets, 2172U, 1986U, &current));
    AssertNear(current.ia_a, 124.0f * amp_per_count, TEST_EPSILON);
    AssertNear(current.ib_a, -62.0f * amp_per_count, TEST_EPSILON);
    AssertNear(current.ic_a, -62.0f * amp_per_count, TEST_EPSILON);
    AssertNear(current.ia_a + current.ib_a + current.ic_a, 0.0f, TEST_EPSILON);
}

static void Test_CurrentSenseHonorsConfiguredPolarity(void)
{
    CurrentSenseConfig config = DefaultSenseConfig();
    const CurrentSenseOffsets offsets = {2048.0f, 2048.0f};
    CurrentPhaseCurrents current;

    config.phase_a_polarity = -1.0f;
    config.phase_b_polarity = -1.0f;
    assert(CurrentSense_Convert(&config, &offsets, 2172U, 1986U, &current));
    assert(current.ia_a < 0.0f);
    assert(current.ib_a > 0.0f);
    AssertNear(current.ia_a + current.ib_a + current.ic_a, 0.0f, TEST_EPSILON);
}

static void Test_CurrentSenseRejectsInvalidConfiguration(void)
{
    CurrentSenseConfig config = DefaultSenseConfig();
    const CurrentSenseOffsets offsets = {2048.0f, 2048.0f};
    CurrentPhaseCurrents current;

    config.amplifier_gain_v_per_v = 0.0f;
    assert(!CurrentSense_Convert(&config, &offsets, 2048U, 2048U, &current));
    config = DefaultSenseConfig();
    config.adc_reference_v = NAN;
    assert(!CurrentSense_Convert(&config, &offsets, 2048U, 2048U, &current));
}

static void Test_ClarkeAndParkAtZeroAngle(void)
{
    const CurrentPhaseCurrents phase = {1.0f, -0.5f, -0.5f};
    FocAlphaBeta alpha_beta = {0.0f, 0.0f};
    FocDq dq;

    assert(Foc_Clarke(&phase, &alpha_beta));
    AssertNear(alpha_beta.alpha, 1.0f, TEST_EPSILON);
    AssertNear(alpha_beta.beta, 0.0f, TEST_EPSILON);
    assert(Foc_Park(&alpha_beta, 0.0f, &dq));
    AssertNear(dq.d, 1.0f, TEST_EPSILON);
    AssertNear(dq.q, 0.0f, TEST_EPSILON);
}

static void Test_ParkAtQuarterElectricalTurn(void)
{
    const FocAlphaBeta alpha_beta = {1.0f, 0.0f};
    FocDq dq;

    assert(Foc_Park(&alpha_beta, 0.25f, &dq));
    AssertNear(dq.d, 0.0f, TEST_EPSILON);
    AssertNear(dq.q, -1.0f, TEST_EPSILON);
}

static void Test_TransformsRejectNonFiniteInput(void)
{
    const CurrentPhaseCurrents phase = {NAN, 0.0f, 0.0f};
    const FocAlphaBeta invalid_alpha_beta = {INFINITY, 0.0f};
    FocAlphaBeta alpha_beta = {0.0f, 0.0f};
    FocDq dq;

    assert(!Foc_Clarke(&phase, &alpha_beta));
    assert(!Foc_Park(&invalid_alpha_beta, 0.0f, &dq));
    assert(!Foc_Park(&alpha_beta, NAN, &dq));
}

int main(void)
{
    Test_CurrentSenseZeroAtCalibratedOffset();
    Test_CurrentSenseConvertsAmperesAndReconstructsPhaseC();
    Test_CurrentSenseHonorsConfiguredPolarity();
    Test_CurrentSenseRejectsInvalidConfiguration();
    Test_ClarkeAndParkAtZeroAngle();
    Test_ParkAtQuarterElectricalTurn();
    Test_TransformsRejectNonFiniteInput();

    puts("current-control math tests passed");
    return 0;
}
