# H743 Encoder Speed Current Loop Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a fourth motor-control mode that estimates motor-shaft speed from the existing 17-bit BiSS-C encoder and closes a 1 kHz speed PI around the validated 10 kHz encoder-angle current loop.

**Architecture:** Two HAL-independent modules implement encoder-difference speed estimation and a bounded PI controller. `motor_control` schedules those modules every ten fast ticks, preserves the existing ADC-driven current loop, and treats speed control as a new source of `Iq_ref`; the first three modes remain behaviorally unchanged.

**Tech Stack:** C99, STM32H7 HAL, STM32H743, SPI4 DMA BiSS-C encoder, TIM8/ADC1 10 kHz current loop, 1 kHz divided speed loop, Keil MDK-ARM, PowerShell integration tests, host-side C math tests when a native C compiler is available.

**Spec:** `docs/superpowers/specs/2026-09-18-h7-encoder-speed-current-loop-design.md`

## Global Constraints

- Add `MOTOR_CONTROL_ENCODER_SPEED_CURRENT` as a fourth implemented control mode; do not redefine `MOTOR_CONTROL_ENCODER_ANGLE_CURRENT`.
- All speed commands and feedback are motor-shaft mechanical rpm because the encoder is mounted before the reducer.
- Current control remains in the ADC injected-complete path at 10 kHz; speed control runs at 1 kHz and may only produce `Iq_ref`.
- Speed estimation runs in encoder modes 3 and 4; speed PI runs only in mode 4.
- Use only validated, sequence-changing encoder snapshots. Do not perform blocking SPI, delays, logging, or PWM writes in the speed modules.
- Initial speed-command range is `±100 rpm`; speed-command slew is `20 rpm/s`; speed PI output is limited to `±0.6 A` and can never exceed `MOTOR_CURRENT_COMMAND_LIMIT_A`.
- Preserve all existing current, encoder, calibration, Flash, driver, startup, stop, and fault behavior.
- Do not change the default startup `.mode` to speed mode in this implementation.
- Keep generated Keil products and unrelated existing working-tree changes out of commits.
- The current machine has no native GCC/Clang host runner. Always add the portable C unit tests, use Keil/ArmClang for production compilation, and run the C tests when a native compiler becomes available; PowerShell integration tests and the full Keil build are mandatory now.

---

### Task 1: Implement sequence-aware motor-shaft speed estimation

**Files:**
- Create: `Core/Inc/motor/speed_estimator.h`
- Create: `Core/Src/motor/speed_estimator.c`
- Create: `tests/test_speed_estimator.c`
- Modify: `Core/Inc/motor/motor_params.h`

**Interfaces:**
- Consumes: 17-bit raw motor-shaft position, encoder sequence, calibrated direction, and one 1 ms scheduler period per call.
- Produces: `SpeedEstimatorUpdateStatus`, raw rpm, filtered rpm, and an explicit `ready` state.

- [ ] **Step 1: Write the failing estimator tests**

Create `tests/test_speed_estimator.c` against this public interface:

```c
typedef struct
{
    uint32_t counts_per_turn;
    int8_t direction;
    float filter_cutoff_hz;
} SpeedEstimatorConfig;

typedef enum
{
    SPEED_ESTIMATOR_UPDATE_INVALID = 0,
    SPEED_ESTIMATOR_UPDATE_NO_NEW_SAMPLE,
    SPEED_ESTIMATOR_UPDATE_PRIMED,
    SPEED_ESTIMATOR_UPDATE_READY
} SpeedEstimatorUpdateStatus;

typedef struct
{
    SpeedEstimatorConfig config;
    uint32_t last_position_raw;
    uint32_t last_sequence;
    float elapsed_s;
    float raw_rpm;
    float filtered_rpm;
    bool has_position;
    bool ready;
} SpeedEstimator;

bool SpeedEstimator_Init(SpeedEstimator *estimator,
                         const SpeedEstimatorConfig *config);
void SpeedEstimator_Reset(SpeedEstimator *estimator);
SpeedEstimatorUpdateStatus SpeedEstimator_Update(
    SpeedEstimator *estimator,
    uint32_t position_raw,
    uint32_t sequence,
    float scheduler_period_s);
```

Test these exact behaviors:

```c
/* First sample primes; 1 count/ms is about 0.4577637 rpm. */
assert(SpeedEstimator_Update(&e, 1000U, 10U, 0.001f) ==
       SPEED_ESTIMATOR_UPDATE_PRIMED);
assert(SpeedEstimator_Update(&e, 1001U, 11U, 0.001f) ==
       SPEED_ESTIMATOR_UPDATE_READY);
AssertNear(e.raw_rpm, 60.0f / 131072.0f / 0.001f, 0.001f);

/* Repeated sequence consumes elapsed time but must not publish fake zero speed. */
previous_rpm = e.raw_rpm;
assert(SpeedEstimator_Update(&e, 1001U, 11U, 0.001f) ==
       SPEED_ESTIMATOR_UPDATE_NO_NEW_SAMPLE);
AssertNear(e.raw_rpm, previous_rpm, 0.0001f);
assert(SpeedEstimator_Update(&e, 1003U, 12U, 0.001f) ==
       SPEED_ESTIMATOR_UPDATE_READY);
AssertNear(e.raw_rpm, 60.0f * 2.0f / 131072.0f / 0.002f, 0.001f);

/* Both wrap directions use the shortest 17-bit delta. */
PrimeAt(&e, 131071U, 20U);
UpdateAt(&e, 0U, 21U);
assert(e.raw_rpm > 0.0f);
PrimeAt(&e, 0U, 30U);
UpdateAt(&e, 131071U, 31U);
assert(e.raw_rpm < 0.0f);

/* Calibrated direction reverses the reported mechanical speed sign. */
config.direction = -1;
assert(SpeedEstimator_Init(&e, &config));
PrimeAt(&e, 100U, 1U);
UpdateAt(&e, 110U, 2U);
assert(e.raw_rpm < 0.0f);
```

Also reject null pointers, `counts_per_turn < 2`, directions other than `±1`, non-positive/non-finite cutoff, raw positions outside the configured range, and zero/non-finite scheduler periods. Verify the low-pass output converges toward a constant raw speed without overshoot.

- [ ] **Step 2: Confirm the test fails before implementation**

If a native C compiler is available, run:

```powershell
gcc -std=c99 -Wall -Wextra -Werror -ICore/Inc/motor tests/test_speed_estimator.c Core/Src/motor/speed_estimator.c -lm -o tests/test_speed_estimator.exe
```

Expected: FAIL because `speed_estimator.h/.c` do not exist. On the current machine, record that `Get-Command gcc,clang` returns no compiler and continue with the mandatory source-contract test and Keil compile in Task 4; do not claim that the host executable ran.

- [ ] **Step 3: Implement the estimator with explicit timing**

Use signed 32-bit delta arithmetic and a half-turn wrap:

```c
delta = (int32_t)position_raw - (int32_t)estimator->last_position_raw;
half_turn = (int32_t)(estimator->config.counts_per_turn / 2U);
if (delta > half_turn)
{
    delta -= (int32_t)estimator->config.counts_per_turn;
}
else if (delta < -half_turn)
{
    delta += (int32_t)estimator->config.counts_per_turn;
}

directed_delta = delta * (int32_t)estimator->config.direction;
estimator->raw_rpm =
    ((float)directed_delta * 60.0f) /
    ((float)estimator->config.counts_per_turn * estimator->elapsed_s);
```

Implement a finite-safe first-order low-pass using the actual elapsed time since the last new sequence:

```c
tau_s = 1.0f / (2.0f * 3.14159265358979323846f *
                estimator->config.filter_cutoff_hz);
alpha = estimator->elapsed_s / (tau_s + estimator->elapsed_s);
estimator->filtered_rpm +=
    alpha * (estimator->raw_rpm - estimator->filtered_rpm);
```

The first sample stores position/sequence and clears elapsed time. Every valid scheduler call adds to `elapsed_s`; a repeated sequence returns `NO_NEW_SAMPLE`; a new sequence computes speed and resets elapsed time. `Reset` preserves the config but clears all dynamic state.

- [ ] **Step 4: Add centralized speed parameters**

Append to `motor_params.h` with comments identifying motor/load retuning requirements:

```c
#define MOTOR_ENCODER_COUNTS_PER_TURN       (131072U)
#define MOTOR_SPEED_CONTROL_DIVIDER         (10U)
#define MOTOR_SPEED_CONTROL_PERIOD_S        \
    (MOTOR_CONTROL_PERIOD_S * (float)MOTOR_SPEED_CONTROL_DIVIDER)
#define MOTOR_SPEED_COMMAND_LIMIT_RPM       (100.0f)
#define MOTOR_SPEED_COMMAND_SLEW_RPM_PER_S  (20.0f)
#define MOTOR_SPEED_FILTER_CUTOFF_HZ        (20.0f)
```

Derive the speed period from the existing current-control period and the integer divider as shown;
do not duplicate a second independent `0.001f` constant that could drift when TIM8 changes.

- [ ] **Step 5: Run available checks and commit the estimator**

When GCC is available:

```powershell
& .\tests\test_speed_estimator.exe
```

Expected: `speed estimator tests passed`.

Always run:

```powershell
git diff --check -- Core/Inc/motor/speed_estimator.h Core/Src/motor/speed_estimator.c tests/test_speed_estimator.c Core/Inc/motor/motor_params.h
```

Then commit only these files:

```powershell
git add Core/Inc/motor/speed_estimator.h Core/Src/motor/speed_estimator.c tests/test_speed_estimator.c Core/Inc/motor/motor_params.h
git commit -m "feat: estimate motor shaft speed"
```

### Task 2: Implement the bounded speed PI controller

**Files:**
- Create: `Core/Inc/motor/speed_pi.h`
- Create: `Core/Src/motor/speed_pi.c`
- Create: `tests/test_speed_pi.c`
- Modify: `Core/Inc/motor/motor_params.h`

**Interfaces:**
- Consumes: target rpm, filtered feedback rpm, explicit sample period, gains with physical units, and an ampere output limit.
- Produces: bounded `Iq_ref` plus proportional, integral, error, unsaturated-output, and saturation diagnostics.

- [ ] **Step 1: Write failing PI tests**

Define the interface:

```c
typedef struct
{
    float kp_a_per_rpm;
    float ki_a_per_rpm_s;
    float kaw_per_s;
    float output_limit_a;
} SpeedPiConfig;

typedef struct
{
    SpeedPiConfig config;
    float integrator_a;
} SpeedPiController;

typedef struct
{
    float error_rpm;
    float proportional_a;
    float integrator_a;
    float unsaturated_a;
    float iq_command_a;
    bool saturated;
} SpeedPiResult;

bool SpeedPi_Init(SpeedPiController *controller,
                  const SpeedPiConfig *config);
bool SpeedPi_Step(SpeedPiController *controller,
                  float reference_rpm,
                  float feedback_rpm,
                  float sample_period_s,
                  SpeedPiResult *result);
bool SpeedPi_PreloadOutput(SpeedPiController *controller,
                           float reference_rpm,
                           float feedback_rpm,
                           float requested_iq_a);
bool SpeedPi_SetGains(SpeedPiController *controller,
                      float kp_a_per_rpm,
                      float ki_a_per_rpm_s,
                      float kaw_per_s);
bool SpeedPi_SetOutputLimit(SpeedPiController *controller,
                            float output_limit_a);
void SpeedPi_Reset(SpeedPiController *controller);
```

Required assertions:

```c
assert(SpeedPi_Init(&pi, &(SpeedPiConfig){0.005f, 0.10f, 10.0f, 0.6f}));
assert(SpeedPi_Step(&pi, 10.0f, 0.0f, 0.001f, &result));
AssertNear(result.proportional_a, 0.05f, 0.0001f);
AssertNear(pi.integrator_a, 0.001f, 0.0001f);

/* Positive and negative saturation must be symmetric. */
assert(SpeedPi_Step(&pi, 1000.0f, 0.0f, 0.001f, &result));
AssertNear(result.iq_command_a, 0.6f, 0.0001f);
assert(result.saturated);

SpeedPi_Reset(&pi);
assert(SpeedPi_PreloadOutput(&pi, 20.0f, 10.0f, 0.30f));
assert(SpeedPi_Step(&pi, 20.0f, 10.0f, 0.001f, &result));
AssertNear(result.iq_command_a, 0.30f, 0.001f);
```

Run 10,000 saturated steps and assert that `integrator_a` remains finite and bounded. Reject NaN/Inf, negative gains, zero/negative limit, zero/negative period, and null pointers. Verify changing the output limit clamps the existing integrator.

- [ ] **Step 2: Confirm the test fails before implementation**

When GCC is available:

```powershell
gcc -std=c99 -Wall -Wextra -Werror -ICore/Inc/motor tests/test_speed_pi.c Core/Src/motor/speed_pi.c -lm -o tests/test_speed_pi.exe
```

Expected: FAIL because the module is missing. On the current machine, record the absent native compiler as in Task 1.

- [ ] **Step 3: Implement PI, limiting, and anti-windup**

Use the same sign convention as the current PI: `error = reference - feedback`. Calculate:

```c
proportional = kp * error;
unsaturated = proportional + integrator;
saturated = Clamp(unsaturated, output_limit);
correction = ki * error + kaw * (saturated - unsaturated);
integrator = Clamp(integrator + correction * dt, output_limit);
```

Publish `result->integrator_a` after the update. `SpeedPi_PreloadOutput` sets:

```c
integrator = Clamp(requested_iq_a - kp * (reference_rpm - feedback_rpm),
                   output_limit_a);
```

Every public function validates arguments before modifying controller state.

- [ ] **Step 4: Add conservative first-run parameters**

Add to `motor_params.h`:

```c
#define MOTOR_SPEED_PI_KP_A_PER_RPM    (0.005f)
#define MOTOR_SPEED_PI_KI_A_PER_RPM_S  (0.10f)
#define MOTOR_SPEED_PI_KAW_PER_S       (10.0f)
#define MOTOR_SPEED_IQ_LIMIT_A         (0.6f)
```

Comment that these values are bring-up values, not copied F407 final gains, and must be retuned after motor, load, reducer, inertia, or friction changes.

- [ ] **Step 5: Run available checks and commit the PI**

When GCC is available:

```powershell
& .\tests\test_speed_pi.exe
```

Expected: `speed PI tests passed`.

Always run `git diff --check` on the four task files, then:

```powershell
git add Core/Inc/motor/speed_pi.h Core/Src/motor/speed_pi.c tests/test_speed_pi.c Core/Inc/motor/motor_params.h
git commit -m "feat: add bounded motor speed PI"
```

### Task 3: Integrate the fourth mode and bumpless transitions

**Files:**
- Modify: `Core/Inc/motor/motor_runtime_policy.h`
- Modify: `Core/Src/motor/motor_runtime_policy.c`
- Modify: `Core/Inc/motor/motor_control.h`
- Modify: `Core/Src/motor/motor_control.c`
- Modify: `Core/Src/main.c`
- Modify: `tests/test_motor_runtime_policy.c`
- Create: `tests/test_speed_mode_integration.ps1`
- Modify: `tests/test_motor_control_integration.ps1`
- Modify: `tests/test_encoder_mode_integration.ps1`

**Interfaces:**
- Consumes: `SpeedEstimator`, `SpeedPiController`, existing encoder snapshots, current feedback, current reference slew, and mode requests.
- Produces: mode 4 behavior, public speed APIs, and stable debug fields without changing the first three modes.

- [ ] **Step 1: Extend the failing runtime-policy test**

Add assertions before changing production code:

```c
assert(MotorRuntimePolicy_ClassifyStartMode(
           MOTOR_CONTROL_ENCODER_SPEED_CURRENT) ==
       MOTOR_START_ACTION_CURRENT_CONTROL);
assert(MotorRuntimePolicy_ModeRequestAllowed(
    MOTOR_CONTROL_ENCODER_SPEED_CURRENT));
```

Compile/run when a native compiler is available:

```powershell
gcc -std=c99 -Wall -Wextra -Werror -ICore/Inc/motor tests/test_motor_runtime_policy.c Core/Src/motor/motor_runtime_policy.c -o tests/test_motor_runtime_policy.exe
& .\tests\test_motor_runtime_policy.exe
```

Expected before implementation: compile failure because the enum value does not exist.

- [ ] **Step 2: Write the failing source-level integration contract**

Create `tests/test_speed_mode_integration.ps1` with `Assert-Contains` and `Assert-NotContains` helpers. Require all of the following:

```powershell
Assert-Contains $policyHeader 'MOTOR_CONTROL_ENCODER_SPEED_CURRENT'
Assert-Contains $motorHeader 'MotorControl_SetSpeedCommand\s*\('
Assert-Contains $motorHeader 'MotorControl_SwitchToEncoderSpeedCurrent\s*\('
Assert-Contains $motorHeader 'MotorControl_SetSpeedPiGains\s*\('
Assert-Contains $motorHeader 'MotorControl_SetSpeedIqLimit\s*\('
Assert-Contains $motorSource 'MOTOR_SPEED_CONTROL_DIVIDER'
Assert-Contains $motorSource 'SpeedEstimator_Update\s*\('
Assert-Contains $motorSource 'SpeedPi_Step\s*\('
Assert-Contains $motorSource 'SpeedPi_PreloadOutput\s*\('
Assert-Contains $motorSource 'MOTOR_CONTROL_ENCODER_SPEED_CURRENT'
Assert-Contains $motorSource 'speed_control_tick_count'
Assert-Contains $mainSource 'MotorControl_SetSpeedCommand\s*\('
Assert-Contains $mainSource '\.mode\s*=\s*MOTOR_CONTROL_ENCODER_ANGLE_CURRENT'
Assert-NotContains $mainSource '\.mode\s*=\s*MOTOR_CONTROL_ENCODER_SPEED_CURRENT'
```

Also parse the body of `MotorControl_CurrentSampleComplete` and require that it calls the existing `MotorControl_RunCurrentLoop`; reject direct references there to `SpeedPi_Step`, because speed PI belongs in the divided fast-tick task, not the ADC current callback.

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/test_speed_mode_integration.ps1
```

Expected: FAIL on the missing mode and APIs.

- [ ] **Step 3: Add the enum and classify encoder requirements consistently**

Insert `MOTOR_CONTROL_ENCODER_SPEED_CURRENT` immediately after mode 3 and before `MOTOR_CONTROL_FAULT`. Update every mode predicate in `motor_control.c` so that:

- modes 2, 3, and 4 are current-control modes;
- modes 3 and 4 require a valid calibrated encoder and use encoder electrical angle;
- modes 3 and 4 run the speed estimator telemetry;
- only mode 4 runs speed PI and selects its result as the current-loop target;
- startup waiting for the first encoder snapshot applies to modes 3 and 4;
- invalid/stale encoder behavior for mode 4 is identical to mode 3.

Do not replace scattered expressions blindly. Introduce local predicates such as:

```c
static bool MotorControl_ModeUsesCurrentLoop(MotorControlMode mode);
static bool MotorControl_ModeUsesEncoderAngle(MotorControlMode mode);
static bool MotorControl_ModeUsesSpeedLoop(MotorControlMode mode);
```

Use them in startup, request validation, mode transition, fast tick, ADC callback, and fault checks to prevent future omissions.

- [ ] **Step 4: Add speed state, initialization, reset, and debug publication**

Add module state:

```c
static SpeedEstimator speed_estimator;
static SpeedPiController speed_pi;
static SpeedPiResult speed_pi_result;
static volatile float speed_target_rpm;
static float speed_active_target_rpm;
static uint32_t speed_divider_count;
static uint32_t speed_control_tick_count;
static bool speed_mode_waiting_for_estimator;
```

Initialize from `motor_params.h` in `MotorControl_Init`. Extend `MotorControlDebug` with the exact fields from the spec. Add one helper that resets estimator, PI, divided scheduler, readiness, results, and active target; call it from initialization, stop, fault entry, and fault clear paths.

The estimator config direction must come from the loaded encoder calibration record. If encoder calibration succeeds and publishes a new direction, reset/reinitialize the estimator before any later speed-mode entry.

- [ ] **Step 5: Implement the 1 kHz estimator and speed task**

In the 10 kHz `MotorControl_FastTick`, after obtaining the immutable encoder snapshot and validating `dt_s`, increment the divider only while modes 3 or 4 are active. Every tenth tick:

1. call `SpeedEstimator_Update` with the latest raw position, sequence, and `MOTOR_SPEED_CONTROL_PERIOD_S`;
2. publish raw/filtered rpm when the estimator produces a ready sample;
3. in mode 3, stop after telemetry publication;
4. in mode 4, do not run PI until the estimator is ready;
5. slew `speed_active_target_rpm` toward `speed_target_rpm` by at most `MOTOR_SPEED_COMMAND_SLEW_RPM_PER_S * MOTOR_SPEED_CONTROL_PERIOD_S`;
6. call `SpeedPi_Step` exactly once and write its bounded output to the speed-mode current target with `Id = 0`;
7. increment `speed_control_tick_count` only when a new estimator sample drives a PI step.

If a divided tick sees a repeated encoder sequence, retain the last `Iq_ref` and do not integrate the PI. Existing encoder age logic remains responsible for eventual stale fault detection.

- [ ] **Step 6: Implement bumpless entry and exit**

When starting directly in mode 4, keep speed-mode `Iq_ref = 0` until two new encoder positions make the estimator ready. Initialize `speed_active_target_rpm` from filtered feedback, then ramp toward the requested speed.

When switching from mode 3 to mode 4:

- preserve `current_reference_active.q` while the estimator primes;
- after readiness, set `speed_active_target_rpm` to filtered feedback;
- call `SpeedPi_PreloadOutput(..., current_reference_active.q)` before the first speed PI step;
- then allow the target-speed slew and 1 kHz PI updates.

When leaving mode 4, do not overwrite `current_reference_active`; reset speed PI after the new mode has inherited the active current state. Existing current-command slew then moves from the last speed-generated current to the destination command.

- [ ] **Step 7: Implement public APIs and limits**

`MotorControl_SetSpeedCommand` clamps finite input to `±MOTOR_SPEED_COMMAND_LIMIT_RPM` inside the same short interrupt critical-section pattern used by existing command setters. Non-finite input leaves the previous target unchanged.

`MotorControl_SwitchToEncoderSpeedCurrent` validates finite input, sets the speed command, and requests mode 4. `MotorControl_SetSpeedPiGains` and `MotorControl_SetSpeedIqLimit` validate through the pure PI module and atomically update debug values. The runtime `Iq` limit must satisfy:

```c
0.0f < iq_limit_a &&
iq_limit_a <= MOTOR_SPEED_IQ_LIMIT_A &&
iq_limit_a <= MOTOR_CURRENT_COMMAND_LIMIT_A
```

Do not expose writable pointers to controller internals.

- [ ] **Step 8: Preconfigure a safe speed command without changing the default mode**

In `main.c`, retain:

```c
.mode = MOTOR_CONTROL_ENCODER_ANGLE_CURRENT
```

Add before `MotorControl_Start()`:

```c
MotorControl_SetSpeedCommand(0.0f);
```

Extend the existing mode comments to describe mode 4 and explain that changing only `.mode` selects the preconfigured zero-rpm speed mode. The user must intentionally change the speed command for rotation.

- [ ] **Step 9: Update existing integration expressions and pass policy tests**

The existing PowerShell tests currently enumerate the implemented startup modes. Extend their accepted alternations to include `MOTOR_CONTROL_ENCODER_SPEED_CURRENT`, while preserving their checks for current calibration and encoder gating.

Run:

```powershell
powershell -ExecutionPolicy Bypass -File tests/test_speed_mode_integration.ps1
powershell -ExecutionPolicy Bypass -File tests/test_motor_control_integration.ps1
powershell -ExecutionPolicy Bypass -File tests/test_encoder_mode_integration.ps1
```

Expected: all pass.

- [ ] **Step 10: Commit the integrated fourth mode**

Run `git diff --check` on the task files, inspect `git diff --stat`, then:

```powershell
git add Core/Inc/motor/motor_runtime_policy.h Core/Src/motor/motor_runtime_policy.c Core/Inc/motor/motor_control.h Core/Src/motor/motor_control.c Core/Src/main.c tests/test_motor_runtime_policy.c tests/test_speed_mode_integration.ps1 tests/test_motor_control_integration.ps1 tests/test_encoder_mode_integration.ps1
git commit -m "feat: add encoder speed current mode"
```

### Task 4: Add build integration, documentation, and full regression evidence

**Files:**
- Modify: `MDK-ARM/emptytest.uvprojx`
- Modify: `tests/test_biss_h7_config.ps1`
- Modify: `docs/h7-motor-control-code-analysis.md`
- Create: `docs/h7-speed-loop-bring-up.md`

**Interfaces:**
- Consumes: the completed estimator, PI, and mode-4 integration.
- Produces: a buildable Keil target, discoverable source modules, and an exact low-risk hardware bring-up procedure.

- [ ] **Step 1: Make the project-file test fail for missing new modules**

Extend `tests/test_biss_h7_config.ps1`:

```powershell
Assert-Contains $project 'speed_estimator\.c'
Assert-Contains $project 'speed_pi\.c'
```

Run the script and verify it fails before editing the Keil project.

- [ ] **Step 2: Add both production files to the Motor group**

Add adjacent to `current_pi.c` in `MDK-ARM/emptytest.uvprojx`:

```xml
<File>
  <FileName>speed_estimator.c</FileName>
  <FileType>1</FileType>
  <FilePath>../Core/Src/motor/speed_estimator.c</FilePath>
</File>
<File>
  <FileName>speed_pi.c</FileName>
  <FileType>1</FileType>
  <FilePath>../Core/Src/motor/speed_pi.c</FilePath>
</File>
```

Run `tests/test_biss_h7_config.ps1` again and expect PASS.

- [ ] **Step 3: Document control flow and motor/reducer retuning points**

Update `docs/h7-motor-control-code-analysis.md` to show:

- the new fourth-mode data path;
- estimator execution in modes 3 and 4;
- 1 kHz versus 10 kHz ownership;
- motor-shaft rpm versus reducer-output rpm;
- parameters that must be changed after replacing the motor, encoder, reducer, or load;
- why the F407 coefficient and discrete PI gains were not copied.

Create `docs/h7-speed-loop-bring-up.md` with the exact watch list and procedure:

1. Mode 3, `Id=0`, start at `Iq=+0.2 A`.
2. Increase by `0.1 A` only if reducer friction prevents motion, never above `+0.6 A`.
3. Confirm positive position progression and positive raw/filtered rpm.
4. Stop and repeat with negative current.
5. Mode 4 at `0 rpm`, confirm no sustained saturation.
6. Test `+5`, `+10`, `-5`, `-10`, then `±20 rpm`.
7. Increase toward `±100 rpm` only after stable low-speed evidence.

Include explicit stop criteria: unexpected direction, continuous `speed_pi_saturated`, current at the `0.6 A` limit without acceleration, encoder errors, oscillation, audible impact, or any motor fault.

- [ ] **Step 4: Run all mandatory source and configuration regressions**

```powershell
powershell -ExecutionPolicy Bypass -File tests/test_speed_mode_integration.ps1
powershell -ExecutionPolicy Bypass -File tests/test_motor_control_integration.ps1
powershell -ExecutionPolicy Bypass -File tests/test_encoder_mode_integration.ps1
powershell -ExecutionPolicy Bypass -File tests/test_h7_current_config.ps1
powershell -ExecutionPolicy Bypass -File tests/test_biss_h7_config.ps1
```

Expected: every script reports passed.

If a native C compiler is available, freshly rebuild and run:

```powershell
gcc -std=c99 -Wall -Wextra -Werror -ICore/Inc/motor tests/test_speed_estimator.c Core/Src/motor/speed_estimator.c -lm -o tests/test_speed_estimator.exe
& .\tests\test_speed_estimator.exe
gcc -std=c99 -Wall -Wextra -Werror -ICore/Inc/motor tests/test_speed_pi.c Core/Src/motor/speed_pi.c -lm -o tests/test_speed_pi.exe
& .\tests\test_speed_pi.exe
gcc -std=c99 -Wall -Wextra -Werror -ICore/Inc/motor tests/test_motor_runtime_policy.c Core/Src/motor/motor_runtime_policy.c -o tests/test_motor_runtime_policy.exe
& .\tests\test_motor_runtime_policy.exe
```

On the current machine, explicitly report these as not run because no native compiler exists; pre-existing executables are not fresh evidence.

- [ ] **Step 5: Run a clean Keil build and inspect the result**

Use the installed Keil executable:

```powershell
& 'D:\Keil_v5\UV4\UV4.exe' -b 'MDK-ARM\emptytest.uvprojx' -j0 -o 'codex_speed_build.log'
Get-Content 'codex_speed_build.log'
```

Required result:

```text
0 Error(s), 0 Warning(s)
```

Confirm the log shows `speed_estimator.c` and `speed_pi.c` compiled. Remove only the temporary root-level `codex_speed_build.log` with `apply_patch`; do not delete user-owned `MDK-ARM/emptytest` products.

- [ ] **Step 6: Verify diff scope and commit build/docs integration**

```powershell
git diff --check -- MDK-ARM/emptytest.uvprojx tests/test_biss_h7_config.ps1 docs/h7-motor-control-code-analysis.md docs/h7-speed-loop-bring-up.md
git status --short
git diff --stat
```

Stage only intended source, tests, project metadata, and docs. Never stage `.o`, `.crf`, `.axf`, `.hex`, `.map`, `.d`, `.lnp`, build logs, `.uvguix.*`, or unrelated pre-existing changes.

```powershell
git add MDK-ARM/emptytest.uvprojx tests/test_biss_h7_config.ps1 docs/h7-motor-control-code-analysis.md docs/h7-speed-loop-bring-up.md
git commit -m "docs: add speed-loop bring-up guide"
```

- [ ] **Step 7: Final verification summary**

Record:

- exact passing PowerShell test commands;
- Keil `0 Error(s), 0 Warning(s)` evidence;
- whether native C unit tests ran or were unavailable;
- unchanged default startup mode;
- initial runtime limits (`±100 rpm`, `20 rpm/s`, `±0.6 A`);
- the first hardware step is mode-3 sign validation, not direct full-speed mode-4 operation.
