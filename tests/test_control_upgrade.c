#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "motor_params.h"
#include "motor_runtime_policy.h"
#include "speed_pi.h"
#include "position_controller.h"
#include "current_sense.h"
#include "encoder_calibration.h"

static void Near(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.00002f);
}

static void TestTimingAndProtectionWindows(void)
{
    uint32_t phase = 0U;
    uint32_t reads = 0U;
    uint32_t speed_phase = 0U;
    uint32_t speed_updates = 0U;
    uint32_t ticks;
    Near(MOTOR_CONTROL_PERIOD_S, 0.00005f);
    Near(MOTOR_SPEED_CONTROL_PERIOD_S, 0.0005f);
    assert(MOTOR_SPEED_CONTROL_DIVIDER == 10U);
    assert(MOTOR_ENCODER_CONTROL_DIVIDER == 2U);
    for (ticks = 0U; ticks < 20000U; ++ticks)
    {
        bool due = MotorRuntimePolicy_EncoderReadDue(&phase, true);
        assert(due == ((ticks % 2U) == 0U));
        reads += due ? 1U : 0U;
        due = MotorRuntimePolicy_SpeedTaskDue(&speed_phase);
        assert(due == ((ticks % 10U) == 9U));
        speed_updates += due ? 1U : 0U;
    }
    assert(reads == 10000U);
    assert(speed_updates == 2000U);
    assert(!MotorRuntimePolicy_SpeedTaskDue(0));
    assert(MotorRuntimePolicy_EncoderReadDue(&phase, true));
    assert(!MotorRuntimePolicy_EncoderReadDue(&phase, false));
    assert(phase == 0U);
    assert(MotorRuntimePolicy_EncoderReadDue(&phase, true));
    assert(!MotorRuntimePolicy_EncoderReadDue(0, true));
    Near((MOTOR_CURRENT_ADC_TIMEOUT_TICKS + 1U) * MOTOR_CONTROL_PERIOD_S, 0.0004f);
    Near((MOTOR_ENCODER_STALE_LIMIT_TICKS + 1U) * MOTOR_CONTROL_PERIOD_S, 0.0011f);
    Near((MOTOR_ENCODER_DMA_TIMEOUT_TICKS + 1U) * MOTOR_CONTROL_PERIOD_S, 0.0003f);
    Near((MOTOR_ENCODER_CAL_INVALID_LIMIT_TICKS + 1U) * MOTOR_CONTROL_PERIOD_S, 0.0101f);
    Near(MOTOR_CURRENT_OVERCURRENT_CONFIRM_COUNT * MOTOR_CONTROL_PERIOD_S, 0.0002f);
    Near(MOTOR_CURRENT_CALIBRATION_DISCARD_COUNT * MOTOR_CONTROL_PERIOD_S, 0.0016f);
    Near(MOTOR_CURRENT_CALIBRATION_SAMPLE_COUNT * MOTOR_CONTROL_PERIOD_S, 0.0256f);
    assert(MOTOR_ENCODER_ALIGN_SAMPLE_COUNT == 128U);
}

static void TestCalibrationSamplingAndMissingFrameBoundary(void)
{
    CurrentSenseCalibration offsets;
    EncoderCalibration encoder;
    EncoderCalibrationInput missing = {true, false, 0U};
    uint32_t index;
    const uint32_t samples = MOTOR_CURRENT_CALIBRATION_DISCARD_COUNT +
                             MOTOR_CURRENT_CALIBRATION_SAMPLE_COUNT;
    CurrentSenseCalibration_Start(&offsets, MOTOR_CURRENT_CALIBRATION_DISCARD_COUNT,
                                   MOTOR_CURRENT_CALIBRATION_SAMPLE_COUNT);
    for (index = 0U; index < samples; ++index)
    {
        assert(CurrentSenseCalibration_AddSample(&offsets, 2048U, 2048U) ==
               (index == samples - 1U));
    }
    assert(offsets.sample_count == 512U);
    EncoderCalibration_Init(&encoder);
    assert(EncoderCalibration_Start(&encoder));
    for (index = 0U; index < MOTOR_ENCODER_CAL_INVALID_LIMIT_TICKS; ++index)
    {
        (void)EncoderCalibration_Step(&encoder, &missing, MOTOR_CONTROL_PERIOD_S);
        assert(encoder.state != ENCODER_CAL_FAILED);
    }
    (void)EncoderCalibration_Step(&encoder, &missing, MOTOR_CONTROL_PERIOD_S);
    assert(encoder.state == ENCODER_CAL_FAILED);
    assert(encoder.failure == ENCODER_CAL_FAILURE_ENCODER_MISSING);
}

static void TestBusVoltageAndLimits(void)
{
    assert(MOTOR_BUS_VOLTAGE_SELECT_V == 24 || MOTOR_BUS_VOLTAGE_SELECT_V == 48);
    Near(MOTOR_NOMINAL_VBUS_V, (float)MOTOR_BUS_VOLTAGE_SELECT_V);
    Near(MOTOR_POLE_VOLTAGE_LIMIT_START_PU * MOTOR_NOMINAL_VBUS_V, 3.6f);
    Near(MOTOR_POLE_VOLTAGE_LIMIT_MAX_PU * MOTOR_NOMINAL_VBUS_V, 10.8f);
    Near(MOTOR_CURRENT_SAMPLING_VOLTAGE_LIMIT_PU, 0.45f);
    assert(MOTOR_POLE_VOLTAGE_LIMIT_MAX_PU <= MOTOR_CURRENT_SAMPLING_VOLTAGE_LIMIT_PU);
    Near(MOTOR_OPEN_LOOP_START_UQ_V, 1.92f);
    Near(MOTOR_OPEN_VOLTAGE_SLEW_V_PER_S, 120.0f);
}

static void TestPhysicalSpeedControllerEquivalence(void)
{
    const float rad_s_to_rad_s = 6.283185307179586f / 60.0f;
    const SpeedPiConfig config = {
        MOTOR_SPEED_PI_KP_A_PER_RAD_S, MOTOR_SPEED_PI_KI_A_PER_RAD,
        MOTOR_SPEED_PI_KAW_PER_S, MOTOR_SPEED_IQ_LIMIT_A
    };
    SpeedPiController pi;
    SpeedPiResult result;
    PositionController position;
    PositionControllerResult command;
    const PositionControllerConfig position_config = {
        MOTOR_POSITION_KP_RAD_S_PER_DEG, MOTOR_POSITION_SPEED_LIMIT_RAD_S,
        MOTOR_POSITION_TOLERANCE_DEG
    };
    Near(MOTOR_SPEED_COMMAND_LIMIT_RAD_S, 100.0f * rad_s_to_rad_s);
    Near(MOTOR_SPEED_COMMAND_SLEW_RAD_S2, 20.0f * rad_s_to_rad_s);
    assert(SpeedPi_Init(&pi, &config));
    assert(SpeedPi_Step(&pi, 10.0f * rad_s_to_rad_s, 0.0f,
                        MOTOR_SPEED_CONTROL_PERIOD_S, &result));
    Near(result.proportional_a, 0.03f * 10.0f);
    Near(result.integrator_a, 0.015f * 10.0f * 0.0005f);
    assert(SpeedPi_PreloadOutput(&pi, 20.0f * rad_s_to_rad_s,
                                 10.0f * rad_s_to_rad_s, 0.4f));
    assert(SpeedPi_Step(&pi, 20.0f * rad_s_to_rad_s, 10.0f * rad_s_to_rad_s,
                        MOTOR_SPEED_CONTROL_PERIOD_S, &result));
    Near(result.iq_command_a, 0.4f);
    assert(PositionController_Init(&position, &position_config));
    assert(PositionController_Step(&position, 1.0f, 359.0f / 360.0f, &command));
    Near(command.speed_target_rad_s, 0.8f * rad_s_to_rad_s);
    assert(PositionController_Step(&position, 180.0f, 0.0f, &command));
    Near(command.speed_target_rad_s, 20.0f * rad_s_to_rad_s);
}

int main(void)
{
    TestTimingAndProtectionWindows();
    TestCalibrationSamplingAndMissingFrameBoundary();
    TestBusVoltageAndLimits();
    TestPhysicalSpeedControllerEquivalence();
    puts("20k/rad-s/bus configuration tests passed");
    return 0;
}
