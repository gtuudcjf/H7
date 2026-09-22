# H7 Single-Turn Position Loop Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a selectable single-turn shortest-path motor-shaft position mode without changing the established current and speed modes.

**Architecture:** A HAL-free position P controller converts calibrated mechanical angle error to a bounded speed command. The new mode feeds that command into the existing 1 kHz speed PI and 10 kHz current PI, reusing their ramp, current limit, encoder validity and fault paths. On entering position mode, it captures the present angle and holds until an explicit command arrives.

**Tech Stack:** STM32H743 C, STM32 HAL, Keil MDK project, PowerShell structural integration tests, host C unit tests.

**Spec:** `docs/superpowers/specs/2026-09-22-h7-single-turn-position-loop-design.md`

## Global Constraints

- The encoder is 17-bit, 131072 counts per motor-shaft turn. This is a single-turn target, not persistent multiturn position.
- Position commands are calibrated motor-shaft degrees in `[0, 360)`, with circular error in `(-180, 180]`; the exact 180-degree tie chooses positive rotation.
- Initial position constants: P gain `0.4 rpm/degree`, speed cap `20 rpm`, tolerance `0.5 degree`. Keep mode 4's `50 rpm` target and its gains unchanged.
- Preserve mode 3 direct `Iq`, mode 4 speed command, `0.7 A` speed-loop `Iq` cap, `10 A/s` speed-mode current slew and all existing protection limits.
- Preserve numeric values of all existing `MotorControlMode` members. Add the new position member after the existing fault member with an explicit unused value.
- On entry, capture present encoder angle and hold; reject commands unless the position mode is running. Do not queue a stale target across stop/fault/mode exit.
- Do not stage Keil outputs, debugger files, `.exe` test binaries or videos. Avoid changing unrelated ADC/PWM/encoder-frame code.
- A host C compiler was not found on `PATH` during planning. Before C test red/green cycles, locate or provision one; prebuilt test `.exe` files do not verify changed source. If unavailable, report this as a verification limit rather than claiming C unit tests passed.

## File Structure

- `Core/Inc/motor/position_controller.h`, `Core/Src/motor/position_controller.c`: pure controller config and shortest-path position-to-speed calculation; no HAL, mode state or hardware access.
- `Core/Inc/motor/motor_runtime_policy.h`, `Core/Src/motor/motor_runtime_policy.c`: identify the new mode as an encoder-based current-control start action without renumbering existing modes.
- `Core/Inc/motor/motor_control.h`, `Core/Src/motor/motor_control.c`: public command API, mode-entry capture, 1 kHz position task, debug state and safety integration. Keep the existing large control file's scope limited to orchestration.
- `Core/Inc/motor/motor_params.h`: only new position constants; `Core/Src/main.c`: only explanatory mode selection comments.
- `MDK-ARM/emptytest.uvprojx`: compile the new controller source.
- `tests/test_position_controller.c`, `tests/test_position_mode_integration.ps1`: behavior and structural regression tests; extend `tests/test_motor_runtime_policy.c` for classification.

## Review Focus

1. Invalid command (`NaN`, infinity, negative, `360`) must return an error and leave the active target unchanged; test in Task 1 and Task 3.
2. Crossing calibrated zero (359→1 and 1→359) must choose the 2-degree path; test in Task 1.
3. Exactly 180 degrees must choose a deterministic positive direction; test in Task 1.
4. Position entry after mode 4 at nonzero speed must capture current angle and reuse PI/current handoff rather than replaying an old target; test in Task 3.
5. Missing/stale encoder or absent calibration must reject position start or enter the existing fault path without enabling unknown-angle torque; test in Task 3.

---

### Task 1: Pure Position-to-Speed Controller

**Files:**
- Create: `Core/Inc/motor/position_controller.h`
- Create: `Core/Src/motor/position_controller.c`
- Create: `tests/test_position_controller.c`

**Interfaces:**
- Consumes: target degrees in `[0, 360)`, measured `mechanical_angle_pu` in `[0, 1)`.
- Produces: `bool PositionController_Init(PositionController *controller, const PositionControllerConfig *config)` and `bool PositionController_Step(const PositionController *controller, float target_deg, float actual_angle_pu, PositionControllerResult *result)`; result fields `error_deg`, `speed_target_rpm`, `within_tolerance`.

- [ ] **Step 1: Locate a GCC-compatible host C compiler.** Run `$motorHostCc = (Get-Command gcc,clang -ErrorAction SilentlyContinue | Select-Object -First 1).Source`; if none is present, provision one before implementing C code. Keep its installation out of this repository, then rerun that command. Do not use MSVC `cl` with the GCC-style commands below.
- [ ] **Step 2: Write a failing C test.** Use `assert()` with config `{0.4f, 20.0f, 0.5f}` and assert 359→1 produces `+2 deg`, `+0.8 rpm`; 1→359 produces `-2 deg`, `-0.8 rpm`; 180-degree tie is positive; 0.25-degree error gives zero speed; 100-degree error caps at `20 rpm`. Also assert initialization or step rejects `NAN`, infinity, negative target, `360.0f`, invalid feedback, null pointers and zero/nonfinite config without changing an existing result.
- [ ] **Step 3: Compile and run red.** Run `& $motorHostCc -std=c11 -Wall -Wextra -Werror -I Core/Inc/motor tests/test_position_controller.c Core/Src/motor/position_controller.c -lm -o "$env:TEMP\test_position_controller.exe"`; test compilation must fail because the new API is absent, not because of unrelated syntax.
- [ ] **Step 4: Implement the minimal pure controller.** Define the three config fields `kp_rpm_per_deg`, `max_speed_rpm`, `tolerance_deg`; check finite ranges. In `Step`, compute `error = target_deg - actual_angle_pu * 360.0f`, add/subtract 360 until it is in `(-180, 180]`, set zero speed within tolerance, otherwise clamp `kp * error` to ±`max_speed_rpm`. Build the result locally and copy it to the caller only after validation succeeds.
- [ ] **Step 5: Run green and commit.** Recompile with the Step 3 command, run `& "$env:TEMP\test_position_controller.exe"` and require exit 0. Stage only the three Task 1 source/test files, then commit `feat: add single-turn position controller`.

### Task 2: Recognize the New Mode Without Renumbering Existing Modes

**Files:**
- Modify: `Core/Inc/motor/motor_runtime_policy.h:10-24`
- Modify: `Core/Src/motor/motor_runtime_policy.c:15-28`
- Modify: `tests/test_motor_runtime_policy.c:35-65`

**Interfaces:**
- Consumes: existing `MotorControlMode` and `MotorRuntimePolicy_ClassifyStartMode()`.
- Produces: `MOTOR_CONTROL_ENCODER_POSITION_CURRENT = 6` classified as `MOTOR_START_ACTION_CURRENT_CONTROL`; existing mode values and classifications stay unchanged.

- [ ] **Step 1: Add failing assertions.** In the policy test assert the new enum is `6`, `MOTOR_CONTROL_FAULT` remains `5`, the new mode classifies as current-control, and `MotorRuntimePolicy_ModeRequestAllowed()` accepts it; retain all existing assertions.
- [ ] **Step 2: Run red.** Compile/run `tests/test_motor_runtime_policy.c` with `motor_runtime_policy.c` using `$motorHostCc` and the same `-std=c11 -Wall -Wextra -Werror -I Core/Inc/motor` flags; expect failure because the new enum is absent.
- [ ] **Step 3: Implement and run green.** Append the explicitly numbered new enum after `MOTOR_CONTROL_FAULT`, extend the existing policy classification condition, compile and run the test, and require exit 0.
- [ ] **Step 4: Commit.** Stage only the three policy files and commit `feat: recognize encoder position mode`.

### Task 3: Integrate Position Mode into Existing Cascade

**Files:**
- Modify: `Core/Inc/motor/motor_params.h:114-133`
- Modify: `Core/Inc/motor/motor_control.h:53-138,179-244`
- Modify: `Core/Src/motor/motor_control.c:154-222,595-815,904-1030,1121-1382,1736-1795`
- Create: `tests/test_position_mode_integration.ps1`
- Modify: `tests/test_speed_mode_integration.ps1`
- Modify: `Core/Src/main.c:62-82`

**Interfaces:**
- Consumes: Task 1 `PositionController_Init/Step`, Task 2 `MOTOR_CONTROL_ENCODER_POSITION_CURRENT`.
- Produces: `HAL_StatusTypeDef MotorControl_SetPositionCommand(float target_deg)` and `HAL_StatusTypeDef MotorControl_SwitchToEncoderPositionCurrent(void)`; new debug fields `position_target_deg`, `position_feedback_deg`, `position_error_deg`, `position_speed_target_rpm`, `position_control_ready`.

- [ ] **Step 1: Write red integration assertions.** The PowerShell test must require the new API declarations/definitions, position-only command validation, position mode in current/encoder/speed predicates, position capture on entry/start, `PositionController_Step` before `SpeedPi_Step` in the 1 kHz task, preserved speed-mode branch using `speed_target_rpm`, and position debug fields. Check mode 3 direct current-reference selection and mode 4 startup target remain intact. Add assertions for absent calibration and stale encoder reuse of existing guards.
- [ ] **Step 2: Run red.** Execute `powershell -NoProfile -ExecutionPolicy Bypass -File tests/test_position_mode_integration.ps1`; expect named missing-position-feature assertions, not a script parsing error. Run `tests/test_speed_mode_integration.ps1` separately and record its baseline result.
- [ ] **Step 3: Add configuration and state.** Add `MOTOR_POSITION_KP_RPM_PER_DEG 0.4f`, `MOTOR_POSITION_SPEED_LIMIT_RPM 20.0f`, `MOTOR_POSITION_TOLERANCE_DEG 0.5f`. Initialize the pure controller inside `MotorControl_Init()` with fail-closed config handling. Maintain a separate `position_target_deg`, `position_speed_target_rpm` and ready flag; do not overwrite mode 4's `speed_target_rpm`.
- [ ] **Step 4: Add safe mode entry and command API.** Include position mode in existing current/encoder/speed predicates. On each entry to position mode, capture the fresh `encoder_angle_sample.mechanical_angle_pu * 360.0f` as target, set position output speed to zero and prepare the existing speed-PI handoff. `MotorControl_SetPositionCommand` validates finite `[0,360)` and `motor_mode == position`, `run_state == RUNNING`, encoder calibration/feedback valid; update target atomically only on success. Stop/fault/mode exit clears ready/target state, so an old target cannot replay.
- [ ] **Step 5: Feed only position mode through the new outer loop.** In the 1 kHz task, after a ready `SpeedEstimator_Update`, call `PositionController_Step` in position mode and use its bounded result as the requested speed; mode 4 continues to use `speed_target_rpm`. Both paths retain the existing speed slew, `SpeedPi_Step`, `Iq` limit and 10 A/s reference slew. Add debug fields without changing existing field meanings. Add a short `main.c` comment showing the new enum, while leaving `.mode = MOTOR_CONTROL_ENCODER_SPEED_CURRENT` and the 50 rpm command untouched.
- [ ] **Step 6: Run green and commit.** Run new position integration test and existing speed/encoder/motor integration scripts; require exit 0. Review `git diff` to confirm no current/speed parameter or PWM/ADC logic change, then stage only Task 3 files and commit `feat: integrate single-turn position mode`.

### Task 4: Keil Integration and Full Verification

**Files:**
- Modify: `MDK-ARM/emptytest.uvprojx` (add `position_controller.c` to motor sources)
- Modify: `docs/h7-motor-control-code-analysis.md` (new mode/control-flow section)
- Modify: `docs/h7-speed-loop-bring-up.md` (position-mode first-run safety note)
- Test: all existing `tests/*.ps1` integration scripts and source-based host C tests.

**Interfaces:**
- Consumes: Tasks 1–3 final source tree.
- Produces: a Keil project that compiles all position sources and documentation for safe first-run verification.

- [ ] **Step 1: Add failing project assertion.** Extend `tests/test_position_mode_integration.ps1` to require `position_controller.c` in `MDK-ARM/emptytest.uvprojx`, run the test and verify this new assertion fails.
- [ ] **Step 2: Add the project entry and document usage.** Add the position source to the existing motor source group. Document units, zero reference, hold-on-entry, explicit command, debug watch fields, 20 rpm/0.7 A first-run limits, and the fact that hardware stability remains unverified.
- [ ] **Step 3: Run full source-based checks.** Run `Get-ChildItem tests -Filter '*.ps1' | ForEach-Object { & powershell -NoProfile -ExecutionPolicy Bypass -File $_.FullName; if ($LASTEXITCODE -ne 0) { throw $_.Name } }`. Rebuild and run the four directly affected host C suites with the following PowerShell; each compilation and executable must exit 0, and prebuilt untracked `.exe` files must not be substituted:

```powershell
$cases = @(
  @('test_position_controller', 'position_controller'),
  @('test_motor_runtime_policy', 'motor_runtime_policy'),
  @('test_speed_estimator', 'speed_estimator'),
  @('test_speed_pi', 'speed_pi')
)
foreach ($case in $cases) {
  $exe = Join-Path $env:TEMP ($case[0] + '.exe')
  & $motorHostCc -std=c11 -Wall -Wextra -Werror -I Core/Inc/motor `
    ("tests/" + $case[0] + '.c') ("Core/Src/motor/" + $case[1] + '.c') `
    -lm -o $exe
  if ($LASTEXITCODE -ne 0) { throw "Compile failed: $($case[0])" }
  & $exe
  if ($LASTEXITCODE -ne 0) { throw "Test failed: $($case[0])" }
}
```

- [ ] **Step 4: Build outside the dirty user worktree.** Create a uniquely named verification copy directly under the workspace root, copy the project with `robocopy` while excluding `.git`, `.worktrees`, videos and generated `MDK-ARM/emptytest/` outputs, then build there with `& 'D:\Keil_v5\UV4\UV4.exe' -b 'MDK-ARM\emptytest.uvprojx' -j0 -o 'position_verify.log'`. Require both process success and `0 Error(s), 0 Warning(s)` in `MDK-ARM/position_verify.log`. Before removing the copy, resolve its exact absolute path and verify it is the uniquely created directory under the workspace root; remove no other path.
- [ ] **Step 5: Review and commit.** Run `git diff --check`, inspect changed paths, compare mode 3/4 constants and logic against their base commit, then stage only the project, docs and final tests. Commit `test: verify position mode integration`.

## Execution Boundary

Software tests and a zero-warning build do not validate physical position stability. After implementation, the user should perform supervised no-load small-angle steps, then zero-crossing steps, monitoring position error, speed, `Iq`, saturation and encoder faults before increasing speed or load. Do not automatically flash firmware or energize the motor during plan execution.
