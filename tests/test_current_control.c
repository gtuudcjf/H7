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
#include "current_pi.h"
#include "drv8323_registers.h"
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

static void Test_Drv8323CsaConfigurationUsesFiveVoltPerVoltGain(void)
{
    assert((DRV8323_CSA_CONTROL_DEFAULT & DRV8323_CSA_GAIN_MASK) ==
           DRV8323_CSA_GAIN_5_V_PER_V);
    assert((DRV8323_CSA_CONTROL_DEFAULT & DRV8323_CSA_VREF_DIV_2) != 0U);
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

static void Test_CurrentCalibrationDiscardsAndAveragesSamples(void)
{
    CurrentSenseCalibration calibration;
    CurrentSenseOffsets offsets;

    CurrentSenseCalibration_Start(&calibration, 2U, 4U);
    assert(!CurrentSenseCalibration_AddSample(&calibration, 100U, 100U));
    assert(!CurrentSenseCalibration_AddSample(&calibration, 4000U, 4000U));
    assert(!CurrentSenseCalibration_AddSample(&calibration, 2046U, 2050U));
    assert(!CurrentSenseCalibration_AddSample(&calibration, 2048U, 2052U));
    assert(!CurrentSenseCalibration_AddSample(&calibration, 2050U, 2054U));
    assert(CurrentSenseCalibration_AddSample(&calibration, 2052U, 2056U));
    assert(CurrentSenseCalibration_GetOffsets(&calibration, 4095.0f, 512.0f, 16U, &offsets));
    AssertNear(offsets.phase_a_count, 2049.0f, TEST_EPSILON);
    AssertNear(offsets.phase_b_count, 2053.0f, TEST_EPSILON);
}

static void Test_CurrentCalibrationRejectsRailAndNoisyOffsets(void)
{
    CurrentSenseCalibration calibration;
    CurrentSenseOffsets offsets;

    CurrentSenseCalibration_Start(&calibration, 0U, 2U);
    assert(!CurrentSenseCalibration_AddSample(&calibration, 10U, 2048U));
    assert(CurrentSenseCalibration_AddSample(&calibration, 12U, 2048U));
    assert(!CurrentSenseCalibration_GetOffsets(&calibration, 4095.0f, 512.0f, 16U, &offsets));

    CurrentSenseCalibration_Start(&calibration, 0U, 2U);
    assert(!CurrentSenseCalibration_AddSample(&calibration, 2000U, 2000U));
    assert(CurrentSenseCalibration_AddSample(&calibration, 2100U, 2100U));
    assert(!CurrentSenseCalibration_GetOffsets(&calibration, 4095.0f, 512.0f, 16U, &offsets));
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

static CurrentPiConfig TestPiConfig(void)
{
    CurrentPiConfig config;

    config.kp_v_per_a = 1.0f;
    config.ki_v_per_a_s = 10.0f;
    config.kaw_per_s = 20.0f;
    config.output_limit_v = 5.0f;
    return config;
}

static void Test_CurrentPiZeroAndProportionalResponse(void)
{
    CurrentPiController controller;
    CurrentPiResult result;
    const FocDq zero = {0.0f, 0.0f};
    const FocDq reference = {1.0f, -2.0f};

    assert(CurrentPi_Init(&controller, &(CurrentPiConfig){1.0f, 0.0f, 0.0f, 5.0f}));
    assert(CurrentPi_StepDq(&controller, &zero, &zero, 0.01f, &result));
    AssertNear(result.voltage_v.d, 0.0f, TEST_EPSILON);
    AssertNear(result.voltage_v.q, 0.0f, TEST_EPSILON);

    assert(CurrentPi_StepDq(&controller, &reference, &zero, 0.01f, &result));
    AssertNear(result.voltage_v.d, 1.0f, TEST_EPSILON);
    AssertNear(result.voltage_v.q, -2.0f, TEST_EPSILON);
}

static void Test_CurrentPiIntegratesAndResets(void)
{
    CurrentPiController controller;
    CurrentPiResult result;
    const FocDq reference = {1.0f, 0.0f};
    const FocDq feedback = {0.0f, 0.0f};

    assert(CurrentPi_Init(&controller, &(CurrentPiConfig){0.0f, 10.0f, 0.0f, 5.0f}));
    assert(CurrentPi_StepDq(&controller, &reference, &feedback, 0.01f, &result));
    assert(CurrentPi_StepDq(&controller, &reference, &feedback, 0.01f, &result));
    AssertNear(result.voltage_v.d, 0.1f, TEST_EPSILON);
    CurrentPi_Reset(&controller);
    AssertNear(controller.integrator_d_v, 0.0f, TEST_EPSILON);
    AssertNear(controller.integrator_q_v, 0.0f, TEST_EPSILON);
}

static void Test_CurrentPiUsesCircularVoltageLimit(void)
{
    CurrentPiController controller;
    CurrentPiResult result;
    const FocDq reference = {10.0f, 10.0f};
    const FocDq feedback = {0.0f, 0.0f};

    assert(CurrentPi_Init(&controller, &(CurrentPiConfig){1.0f, 0.0f, 0.0f, 5.0f}));
    assert(CurrentPi_StepDq(&controller, &reference, &feedback, 0.01f, &result));
    assert(result.saturated);
    AssertNear(hypotf(result.voltage_v.d, result.voltage_v.q), 5.0f, TEST_EPSILON);
    AssertNear(result.voltage_v.d, result.voltage_v.q, TEST_EPSILON);
}

static void Test_CurrentPiAntiWindupKeepsIntegratorBounded(void)
{
    CurrentPiController controller;
    CurrentPiResult result;
    const CurrentPiConfig config = TestPiConfig();
    const FocDq reference = {10.0f, 0.0f};
    const FocDq feedback = {0.0f, 0.0f};
    unsigned int index;

    assert(CurrentPi_Init(&controller, &config));
    for (index = 0U; index < 1000U; ++index)
    {
        assert(CurrentPi_StepDq(&controller, &reference, &feedback, 0.001f, &result));
    }
    assert(fabsf(controller.integrator_d_v) < 10.0f);
}

static void Test_CurrentPiPreloadProducesBumplessOutput(void)
{
    CurrentPiController controller;
    CurrentPiResult result;
    const FocDq reference = {2.0f, -1.0f};
    const FocDq feedback = {0.0f, 0.0f};
    const FocDq requested_voltage = {0.5f, -0.25f};

    assert(CurrentPi_Init(&controller, &(CurrentPiConfig){1.0f, 0.0f, 0.0f, 5.0f}));
    assert(CurrentPi_PreloadOutput(&controller, &reference, &feedback, &requested_voltage));
    assert(CurrentPi_StepDq(&controller, &reference, &feedback, 0.01f, &result));
    AssertNear(result.voltage_v.d, requested_voltage.d, TEST_EPSILON);
    AssertNear(result.voltage_v.q, requested_voltage.q, TEST_EPSILON);
}

static void Test_CurrentPiRejectsInvalidParameters(void)
{
    CurrentPiController controller;
    CurrentPiResult result;
    CurrentPiConfig config = TestPiConfig();
    const CurrentPiConfig valid_config = TestPiConfig();
    const FocDq zero = {0.0f, 0.0f};

    config.kp_v_per_a = NAN;
    assert(!CurrentPi_Init(&controller, &config));
    assert(CurrentPi_Init(&controller, &valid_config));
    assert(!CurrentPi_StepDq(&controller, &zero, &zero, 0.0f, &result));
}

int main(void)
{
    Test_CurrentSenseZeroAtCalibratedOffset();
    Test_Drv8323CsaConfigurationUsesFiveVoltPerVoltGain();
    Test_CurrentSenseConvertsAmperesAndReconstructsPhaseC();
    Test_CurrentSenseHonorsConfiguredPolarity();
    Test_CurrentSenseRejectsInvalidConfiguration();
    Test_CurrentCalibrationDiscardsAndAveragesSamples();
    Test_CurrentCalibrationRejectsRailAndNoisyOffsets();
    Test_ClarkeAndParkAtZeroAngle();
    Test_ParkAtQuarterElectricalTurn();
    Test_TransformsRejectNonFiniteInput();
    Test_CurrentPiZeroAndProportionalResponse();
    Test_CurrentPiIntegratesAndResets();
    Test_CurrentPiUsesCircularVoltageLimit();
    Test_CurrentPiAntiWindupKeepsIntegratorBounded();
    Test_CurrentPiPreloadProducesBumplessOutput();
    Test_CurrentPiRejectsInvalidParameters();

    puts("current-control math tests passed");
    return 0;
}
