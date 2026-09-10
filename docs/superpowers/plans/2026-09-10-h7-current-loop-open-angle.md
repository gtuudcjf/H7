# H7 Open-Angle Current-Loop Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a conservative, PWM-synchronized two-shunt current loop to the STM32H743 project while preserving the verified voltage-open-loop path and supporting safe runtime switching between both modes.

**Architecture:** TIM8 CH4 triggers the two-rank ADC1 injected sequence at a deterministic low-side sampling point. Pure modules convert PB1/PB0 samples to phase currents, perform Clarke/Park transforms, and run d/q PI control. `motor_control` owns calibration, startup, mode transitions, fault latching, and routes either open-loop voltage or current-PI voltage into the existing `svpwm -> pwm_3ph` path.

**Tech Stack:** C99, STM32H7 HAL, CubeMX-generated peripheral files, Keil MDK-ARM project, host-side C tests.

**Spec:** `docs/superpowers/specs/2026-09-10-h7-current-loop-open-angle-design.md`

## Global Constraints

- Preserve all user and generated build/debug artifacts already present in the dirty worktree; commit only task-owned source, project, test, and documentation files.
- Keep the default startup behavior in voltage-open-loop mode.
- Do not remove or rewrite the verified `svpwm` and `pwm_3ph` algorithms unless a failing regression test proves a required correction.
- Do not use PA0 as a phase-current channel; only PB1 (`I_A`) and PB0 (`I_B`) participate in the injected current sequence.
- Do not add BKIN configuration.
- Do not perform delays, blocking SPI, UART output, or encoder reads in the 10 kHz fast path.
- Start with conservative PI and voltage limits; expose observable values for Keil/Ozone tuning.

---

### Task 1: Add pure current scaling and FOC transform modules

**Files:**
- Create: `Core/Inc/motor/motor_params.h`
- Create: `Core/Inc/motor/current_sense.h`
- Create: `Core/Src/motor/current_sense.c`
- Create: `Core/Inc/motor/foc_transform.h`
- Create: `Core/Src/motor/foc_transform.c`
- Create: `tests/test_current_control.c`

**Interfaces:**
- `CurrentSense_Convert()` consumes calibrated ADC samples and returns `Ia/Ib/Ic` in amperes.
- `Foc_Clarke()` and `Foc_Park()` convert phase currents to d/q feedback using the open-loop electrical angle.

- [ ] **Step 1: Write failing tests** for 2048-count zero current, one-amp A/B conversion, polarity selection, `Ic = -(Ia + Ib)`, Clarke/Park at 0 and 0.25 pu angle, and non-finite input rejection.
- [ ] **Step 2: Compile the test without the new implementations** and confirm a missing-header or missing-symbol failure.
- [ ] **Step 3: Add centralized constants** for 12-bit ADC, 3.3 V reference, 20 mΩ shunts, 5 V/V CSA gain, 10 pole pairs, current limits, and nominal 48 V bus.
- [ ] **Step 4: Implement the minimum pure C scaling and transform functions** with explicit comments and no HAL dependency.
- [ ] **Step 5: Compile and run the test** and confirm all scaling/transform assertions pass.
- [ ] **Step 6: Commit the pure math implementation and tests.**

### Task 2: Add the conservative d/q PI controller

**Files:**
- Create: `Core/Inc/motor/current_pi.h`
- Create: `Core/Src/motor/current_pi.c`
- Modify: `tests/test_current_control.c`

**Interfaces:**
- `CurrentPi_Init/Reset/SetGains()` manage validated gains.
- `CurrentPi_StepDq()` accepts ampere references/feedback and returns voltage-domain d/q output with circular limiting and back-calculation anti-windup.
- `CurrentPi_PreloadOutput()` initializes integrators for a bumpless open-loop-to-current-loop transfer.

- [ ] **Step 1: Add failing tests** for zero error, proportional response, integral accumulation, positive/negative symmetry, reset, invalid gain rejection, circular voltage limiting, anti-windup recovery, and output preloading.
- [ ] **Step 2: Compile and run the expanded test** and confirm the new PI tests fail for missing implementation.
- [ ] **Step 3: Implement the controller** with defaults `Kp=0.05 V/A`, `Ki=20 V/(A*s)`, `Kaw=100 1/s`, `Ts=100 us`, startup voltage limit 0.15 pu, and absolute limit 0.45 pu.
- [ ] **Step 4: Run the current-control test** and confirm all assertions pass.
- [ ] **Step 5: Run the existing open-loop regression test** and confirm it still passes unchanged.
- [ ] **Step 6: Commit the PI controller and tests.**

### Task 3: Configure DRV8323 current sensing and board calibration control

**Files:**
- Modify: `Core/Inc/motor/drv8323.h`
- Modify: `Core/Src/motor/drv8323.c`
- Modify: `Core/Inc/motor/drv8323_board.h`
- Modify: `Core/Src/motor/drv8323_board.c`
- Modify if required: `Core/Inc/main.h`
- Modify if required: `Core/Src/gpio.c`

**Interfaces:**
- Symbolic CSA register masks replace the hard-coded gain bits.
- `Drv8323Board_SetCurrentCalibration(bool)` controls PC5 `DRV_CAL`.

- [ ] **Step 1: Add compile-time/static checks or a small host test** proving the CSA gain field encodes 5 V/V and preserves `VREF_DIV`.
- [ ] **Step 2: Confirm the check fails against the current 40 V/V configuration.**
- [ ] **Step 3: Replace the CSA magic word with named masks and fields, configure 5 V/V, and retain the existing six-PWM driver settings.**
- [ ] **Step 4: Add a board-layer calibration-pin function** that only owns PC5 and documents its active level.
- [ ] **Step 5: Re-run host tests or compile checks and inspect the register word.**
- [ ] **Step 6: Commit the driver/current-calibration changes.**

### Task 4: Add TIM8-synchronized two-channel ADC injected sampling

**Files:**
- Modify: `Core/Src/adc.c`
- Modify: `Core/Src/tim.c`
- Modify: `Core/Inc/adc.h` if required
- Modify: `Core/Inc/tim.h` if required
- Modify: `emptytest.ioc` where CubeMX has an equivalent persistent setting

**Interfaces:**
- TIM8 CH4 is an internal trigger source near 95% of the up-count half-cycle.
- ADC1 injected rank 1 is PB1/INP5; rank 2 is PB0/INP9; PA0 is removed from the injected sequence.
- The injected end-of-sequence callback occurs once per 10 kHz PWM period.

- [ ] **Step 1: Add a source-level configuration test** that checks two injected ranks, channel order, non-software trigger, CH4 compare point, and absence of PA0 in the injected sequence.
- [ ] **Step 2: Run it against the existing generated files** and confirm it fails.
- [ ] **Step 3: Configure TIM8 CH4 and TRGO2** so a single selected edge triggers ADC near the low-side measurement window.
- [ ] **Step 4: Configure the ADC1 injected sequence** for PB1 then PB0 and enable injected end-of-sequence interrupt operation.
- [ ] **Step 5: Preserve the relevant configuration in `emptytest.ioc`** without altering unrelated CubeMX settings.
- [ ] **Step 6: Re-run the source-level configuration test and perform a target compile.**
- [ ] **Step 7: Commit the synchronized sampling configuration.**

### Task 5: Implement calibration, state machine, fault handling, and runtime mode switching

**Files:**
- Modify: `Core/Inc/motor/motor_control.h`
- Modify: `Core/Src/motor/motor_control.c`
- Modify: `Core/Inc/motor/open_loop.h` if an explicit voltage preload API is needed
- Modify: `Core/Src/motor/open_loop.c` if an explicit voltage preload API is needed
- Modify: `Core/Src/main.c`
- Modify: `tests/test_current_control.c`

**Interfaces:**
- Add `MOTOR_CONTROL_OPEN_VOLTAGE` and `MOTOR_CONTROL_OPEN_ANGLE_CURRENT` modes plus stopped/fault states.
- Add mode request, current command, PI tuning, debug snapshot, clear-fault, ADC completion, and timeout entry points.
- Preserve the existing open-loop command and startup APIs where practical to avoid breaking callers.

- [ ] **Step 1: Add failing state-machine tests** for calibration sample discard/average, calibration failure, 0.5 s alignment, 0.3 A command ramp, 2 A command clamp, two-sample 10 A trip, explicit fault clear, and both mode-switch directions.
- [ ] **Step 2: Add a failing regression test** that the default mode still follows the verified voltage-open-loop path.
- [ ] **Step 3: Implement asynchronous calibration** using PC5 `DRV_CAL`, 16 discarded samples, and 256 averaged samples while PWM outputs remain disabled.
- [ ] **Step 4: Implement the current fast path** `samples -> currents -> Clarke/Park -> PI -> voltage normalization -> SVPWM -> CCR` in the ADC injected callback.
- [ ] **Step 5: Implement boundary-applied runtime mode requests** preserving electrical angle and preloading the destination controller for minimal voltage discontinuity.
- [ ] **Step 6: Add debug/Ozone observability** for raw ADC, offsets, phase/dq currents, references, PI terms, voltage, angle/frequency, state, saturation, and fault code.
- [ ] **Step 7: Keep TIM8 update callback responsible only for voltage-open-loop updates** and ensure current mode updates PWM only from the ADC completion path.
- [ ] **Step 8: Run all host tests and confirm both current-loop and open-loop behavior pass.**
- [ ] **Step 9: Commit the control integration.**

### Task 6: Integrate new sources into Keil and verify the target

**Files:**
- Modify: `MDK-ARM/emptytest.uvprojx`
- Modify only if needed: `MDK-ARM/emptytest.uvoptx`
- Create: `docs/h7-current-loop-bring-up.md`

- [ ] **Step 1: Add all new motor source files to the existing motor/application group** and reuse `../Core/Inc/motor` include path.
- [ ] **Step 2: Run the existing open-loop host test and the complete current-control host test from a clean temporary output path.**
- [ ] **Step 3: Run the Keil command-line target build** and require zero compiler/linker errors.
- [ ] **Step 4: Inspect warnings and resolve all warnings introduced by this change.**
- [ ] **Step 5: Confirm the generated AXF/ELF contains the new current-loop symbols** and remains usable by Ozone.
- [ ] **Step 6: Document the staged first-power procedure**: no-bus calibration, raw-value/polarity checks, zero-current run, alignment, 0.3 A low-speed test, PI tuning order, and safe fallback to voltage open loop.
- [ ] **Step 7: Commit project integration and bring-up documentation.**

### Task 7: Final regression and handoff

- [ ] **Step 1: Run `git diff --check` on all owned source and documentation changes.**
- [ ] **Step 2: Run all host tests again and capture exact pass output.**
- [ ] **Step 3: Rebuild the Keil target and capture code size, errors, and warnings.**
- [ ] **Step 4: Review the final diff for accidental generated/debug artifact inclusion.**
- [ ] **Step 5: Verify the default mode and command still match the proven voltage-open-loop setup.**
- [ ] **Step 6: Perform a final code review against the approved design and resolve findings.**
- [ ] **Step 7: Use the finishing-a-development-branch workflow to present integration choices.**
