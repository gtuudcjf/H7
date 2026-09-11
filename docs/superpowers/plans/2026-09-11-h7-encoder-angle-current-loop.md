# H7 Encoder-Angle Current Loop Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add SPI4 BiSS-C rotor-angle feedback to the existing STM32H743 Id/Iq current loop while preserving both validated open-loop modes and leaving a stable `Iq_ref` input for a later speed loop.

**Architecture:** A HAL-independent BiSS frame parser and angle converter feed a small SPI4 DMA adapter. `motor_control` selects either the existing virtual angle or one validated encoder-angle snapshot, while continuing to reuse the existing current sensing, FOC, PI, SVPWM, PWM, and DRV8323 modules. A separate alignment state machine determines encoder direction and electrical zero; a reserved Flash sector stores the result only while PWM and the driver are off.

**Tech Stack:** C99, STM32H7 HAL, STM32H743 SPI4 + DMA1/DMAMUX, TIM8 10 kHz control tick, ADC1 injected current sampling, Keil MDK-ARM, PowerShell integration checks, host GCC math tests.

**Spec:** `docs/superpowers/specs/2026-09-11-h7-encoder-angle-current-loop-design.md`

## Global Constraints

- SPI4/ENC_01 is the only encoder used by control in this phase: PE2 is MA/SCK, PE5 is SLO/MISO, and PE6 is a GPIO output held low.
- The encoder is 17-bit single-turn BiSS-C: 13 ACK bits, Start, CDS, 17 position bits, active-low Error/Warning, and inverted CRC6 polynomial `0x43`.
- SPI4 starts at about 1 MHz with CPOL high, first-edge sampling, 8-bit frames, MSB first, and software NSS.
- No blocking SPI, busy wait, delay, UART output, or Flash operation is allowed in TIM8, ADC, SPI, or DMA interrupt paths.
- Current control remains 10 kHz and uses one immutable electrical-angle snapshot for both Park and inverse Park in each sample callback.
- Existing `MOTOR_CONTROL_OPEN_VOLTAGE` and `MOTOR_CONTROL_OPEN_ANGLE_CURRENT` behavior must remain available and regression-tested.
- This phase does not implement a position controller or speed PI; future speed control must produce `Iq_ref` through the public current-command interface.
- Encoder calibration is explicit, not automatic at every boot; Flash writes require PWM off, driver disabled, and fast control stopped.
- Keep user-owned Keil build products, `.uvguix.*`, `.vscode`, ELF files, and unrelated working-tree changes out of commits.

---

### Task 1: Implement the HAL-independent 17-bit BiSS-C frame decoder

**Files:**
- Create: `Core/Inc/motor/biss_frame.h`
- Create: `Core/Src/motor/biss_frame.c`
- Create: `tests/test_biss_frame.c`

**Interfaces:**
- Consumes: exactly 6 raw bytes sampled MSB-first from a 48-clock SPI4 transaction.
- Produces: `uint8_t BissFrame_CalculateCrc6(uint32_t payload, uint8_t bit_count)` and `bool BissFrame_Parse17(const uint8_t raw[6], BissFrame17 *frame)`.

- [ ] **Step 1: Write the failing frame and CRC tests**

Define the public result without HAL dependencies:

```c
#define BISS_FRAME_RAW_BYTES        (6U)
#define BISS_POSITION_BITS          (17U)
#define BISS_POSITION_MAX           (131071UL)
#define BISS_ACK_BITS               (13U)

typedef enum
{
    BISS_FRAME_OK = 0,
    BISS_FRAME_BAD_ARGUMENT,
    BISS_FRAME_SYNC_ERROR,
    BISS_FRAME_ENCODER_ERROR,
    BISS_FRAME_CRC_ERROR
} BissFrameStatus;

typedef struct
{
    uint8_t raw[BISS_FRAME_RAW_BYTES];
    uint32_t position_raw;
    uint8_t received_crc;
    uint8_t calculated_crc;
    uint8_t payload_bit_index;
    bool error_ok;
    bool warning_ok;
    BissFrameStatus status;
} BissFrame17;
```

In `tests/test_biss_frame.c`, add a test-only builder that appends 13 zero ACK bits, Start=1, CDS=0, the 17-bit position, Error, Warning, and the calculated inverted CRC to a 48-bit all-one idle buffer. Add assertions for:

```c
BuildFrame(0x00000U, true, true, raw);
assert(BissFrame_Parse17(raw, &frame));
assert(frame.position_raw == 0U);

BuildFrame(0x1FFFFU, true, true, raw);
assert(BissFrame_Parse17(raw, &frame));
assert(frame.position_raw == 0x1FFFFU);

BuildFrame(0x12345U, true, false, raw);
assert(BissFrame_Parse17(raw, &frame));
assert(!frame.warning_ok);

BuildFrame(0x12345U, false, true, raw);
assert(!BissFrame_Parse17(raw, &frame));
assert(frame.status == BISS_FRAME_ENCODER_ERROR);

raw[5] ^= 0x01U;
assert(!BissFrame_Parse17(raw, &frame));
assert(frame.status == BISS_FRAME_CRC_ERROR);
```

Before using the test-only frame builder, lock the CRC convention with independent known-answer assertions derived from the RLS CRCD01 polynomial/LUT definition. The 19 calculation bits are always `position[16:0]`, Error, Warning, in that order; ACK, Start, and CDS are excluded:

```c
assert(BissFrame_CalculateCrc6(0x00003U, 19U) == 0x3AU); /* pos=0, E=W=1 */
assert(BissFrame_CalculateCrc6(0x48D17U, 19U) == 0x0DU); /* pos=0x12345, E=W=1 */
assert(BissFrame_CalculateCrc6(0x7FFFFU, 19U) == 0x20U); /* pos=max, E=W=1 */
```

Also corrupt one ACK bit and Start to verify `BISS_FRAME_SYNC_ERROR`, and pass null pointers to verify `BISS_FRAME_BAD_ARGUMENT`.

- [ ] **Step 2: Run the decoder test and verify it fails**

Run:

```powershell
gcc -std=c99 -Wall -Wextra -Werror -ICore/Inc/motor tests/test_biss_frame.c Core/Src/motor/biss_frame.c -o tests/test_biss_frame.exe
```

Expected: FAIL because `biss_frame.h/.c` or the declared functions do not exist.

- [ ] **Step 3: Implement CRC and frame location**

Implement bit extraction by absolute MSB-first bit index. Search only the leading response window for the first sequence of exactly 13 low ACK bits followed by Start=1; read CDS but do not use it as position data. Extract the next 17 bits, Error, Warning, and six CRC bits.

Implement the RLS CRCD01 BiSS CRC convention exactly: initialise a six-bit remainder to zero; feed `position[16:0]`, Error, and Warning MSB-first; for each bit XOR it with the current remainder bit 5, shift the remainder left within six bits, and XOR `0x03` when that feedback bit is one. Return the one's complement of the final remainder masked with `0x3F`, because the encoder transmits inverted CRC. Compare that value with the six received CRC bits. This is polynomial `x^6 + x + 1` (`0x43` including the implicit degree-six term).

Never publish position on sync, Error, or CRC failure. Warning remains a valid frame with `warning_ok=false`.

- [ ] **Step 4: Run the decoder test**

Run the command from Step 2 and then:

```powershell
& .\tests\test_biss_frame.exe
```

Expected: `BiSS frame tests passed`.

- [ ] **Step 5: Commit the pure decoder**

```powershell
git add Core/Inc/motor/biss_frame.h Core/Src/motor/biss_frame.c tests/test_biss_frame.c
git commit -m "feat: decode 17-bit BiSS-C frames"
```

### Task 2: Convert validated mechanical position into FOC electrical angle

**Files:**
- Create: `Core/Inc/motor/encoder_angle.h`
- Create: `Core/Src/motor/encoder_angle.c`
- Create: `tests/test_encoder_angle.c`
- Modify: `Core/Inc/motor/motor_params.h`

**Interfaces:**
- Consumes: validated `position_raw`, direction `+1/-1`, zero raw count, and pole-pair count.
- Produces: `bool EncoderAngle_Init(...)`, `bool EncoderAngle_Update(...)`, and an immutable `EncoderAngleSample` containing mechanical and electrical pu angles.

- [ ] **Step 1: Write failing angle conversion tests**

Use these public types and calls:

```c
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

bool EncoderAngle_Init(EncoderAngleConfig *config,
                       uint32_t zero_raw,
                       int8_t direction,
                       uint8_t pole_pairs);
bool EncoderAngle_Update(const EncoderAngleConfig *config,
                         uint32_t position_raw,
                         EncoderAngleSample *sample);
```

Test zero, half-turn, `131071 -> 0` wrap, direction inversion, zero offset, and ten pole pairs. Required examples:

```c
assert(EncoderAngle_Init(&cfg, 0U, 1, 10U));
assert(EncoderAngle_Update(&cfg, 65536U, &sample));
AssertNear(sample.mechanical_angle_pu, 0.5f);
AssertNear(sample.electrical_angle_pu, 0.0f);

assert(EncoderAngle_Init(&cfg, 4096U, -1, 10U));
assert(EncoderAngle_Update(&cfg, 4096U, &sample));
AssertNear(sample.electrical_angle_pu, 0.0f);
```

Reject raw values above `0x1FFFF`, direction zero, pole-pair zero, and null pointers.

- [ ] **Step 2: Run the test and verify it fails**

```powershell
gcc -std=c99 -Wall -Wextra -Werror -ICore/Inc/motor tests/test_encoder_angle.c Core/Src/motor/encoder_angle.c -lm -o tests/test_encoder_angle.exe
```

Expected: FAIL because the module is missing.

- [ ] **Step 3: Implement normalized angle conversion**

Define `MOTOR_POLE_PAIRS (10U)` only in `motor_params.h`. Use `1.0f / 131072.0f`; wrap every result into `[0,1)` with a local finite-safe helper. Do not embed F407 `ZEROANGLE 0x4B73` in the converter.

- [ ] **Step 4: Run the angle and existing math tests**

```powershell
& .\tests\test_encoder_angle.exe
gcc -std=c99 -Wall -Wextra -Werror -ICore/Inc/motor tests/test_current_control.c Core/Src/motor/current_sense.c Core/Src/motor/foc_transform.c Core/Src/motor/current_pi.c -lm -o tests/test_current_control.exe
& .\tests\test_current_control.exe
```

Expected: both executables pass.

- [ ] **Step 5: Commit angle conversion**

```powershell
git add Core/Inc/motor/encoder_angle.h Core/Src/motor/encoder_angle.c Core/Inc/motor/motor_params.h tests/test_encoder_angle.c
git commit -m "feat: convert encoder position to electrical angle"
```

### Task 3: Add the SPI4 DMA BiSS transport without changing motor behavior

**Files:**
- Create: `Core/Inc/motor/biss_encoder.h`
- Create: `Core/Src/motor/biss_encoder.c`
- Modify: `Core/Inc/spi.h`
- Modify: `Core/Src/spi.c`
- Modify: `Core/Inc/stm32h7xx_it.h`
- Modify: `Core/Src/stm32h7xx_it.c`
- Modify: `Core/Inc/main.h`
- Modify: `Core/Src/gpio.c`
- Modify: `Core/Src/main.c`
- Modify: `emptytest.ioc`
- Modify: `MDK-ARM/emptytest.uvprojx`
- Create: `tests/test_biss_h7_config.ps1`

**Interfaces:**
- Consumes: `hspi4`, DMA1 Stream0/Stream1, PE6 receive-only board control, and `BissFrame_Parse17`.
- Produces: `BissEncoder_Init`, `BissEncoder_StartRead`, `BissEncoder_OnTransferComplete`, `BissEncoder_OnTransferError`, and `BissEncoder_GetSnapshot`.

- [ ] **Step 1: Write the failing H7 configuration check**

Create a PowerShell test that requires:

```powershell
Assert-Contains $spi 'hspi4\.Init\.CLKPolarity\s*=\s*SPI_POLARITY_HIGH'
Assert-Contains $spi 'hspi4\.Init\.CLKPhase\s*=\s*SPI_PHASE_1EDGE'
Assert-Contains $spi 'hspi4\.Init\.BaudRatePrescaler\s*=\s*SPI_BAUDRATEPRESCALER_128'
Assert-Contains $gpio 'ENC_TX_01_Pin[\s\S]*GPIO_MODE_OUTPUT_PP'
Assert-Contains $main 'HAL_GPIO_WritePin\(ENC_TX_01_GPIO_Port,\s*ENC_TX_01_Pin,\s*GPIO_PIN_RESET\)'
Assert-Contains $spi 'hdma_spi4_rx'
Assert-Contains $spi 'hdma_spi4_tx'
Assert-Contains $irq 'DMA1_Stream0_IRQHandler'
Assert-Contains $irq 'DMA1_Stream1_IRQHandler'
Assert-Contains $project 'biss_frame\.c'
Assert-Contains $project 'biss_encoder\.c'
Assert-Contains $project 'encoder_angle\.c'
```

Also assert that PE6 is not included in the SPI4 alternate-function GPIO mask.

- [ ] **Step 2: Run the integration check and verify it fails**

```powershell
powershell -ExecutionPolicy Bypass -File tests/test_biss_h7_config.ps1
```

Expected: FAIL on missing SPI4/DMA/PE6 requirements.

- [ ] **Step 3: Implement the transport state and snapshot API**

Use one module-owned instance structure:

```c
typedef struct
{
    uint32_t position_raw;
    uint32_t sequence;
    uint32_t valid_age_ticks;
    uint32_t valid_count;
    uint32_t crc_error_count;
    uint32_t frame_error_count;
    uint32_t spi_error_count;
    uint32_t timeout_count;
    uint8_t raw[6];
    bool warning;
    bool ready;
} BissEncoderSnapshot;
```

`BissEncoder_StartRead()` returns immediately and calls `HAL_SPI_TransmitReceive_DMA(..., 6U)`. TX bytes are all zero. Completion parses into a temporary frame, then updates the public snapshot inside a short interrupt-disabled critical section. Require 32 consecutive valid frames for `ready`; reset the consecutive counter on invalid frames. Add `BissEncoder_ControlTick()` to increment age and detect DMA busy for more than two 100 us ticks.

- [ ] **Step 4: Configure SPI4, PE6, DMA, and callbacks**

Set SPI4 prescaler 128 from the existing 120 MHz SPI45 clock for approximately 0.9375 MHz. Configure DMA1 Stream0 as SPI4_RX and Stream1 as SPI4_TX through DMAMUX, byte alignment, normal mode, RX priority high and TX priority medium. Link both handles with `__HAL_LINKDMA` and add IRQ handlers calling `HAL_DMA_IRQHandler`.

In `main.c`, dispatch only SPI4 callbacks directly to the transport module. This keeps Task 3 compilable before the later `motor_control` integration exists:

```c
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI4)
    {
        BissEncoder_OnTransferComplete();
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI4)
    {
        BissEncoder_OnTransferError(hspi->ErrorCode);
    }
}
```

Keep PE6 low before the first SPI transaction. Update `.ioc` and the Keil project to match hand-edited generated files.

- [ ] **Step 5: Run static checks and existing regressions**

```powershell
powershell -ExecutionPolicy Bypass -File tests/test_biss_h7_config.ps1
powershell -ExecutionPolicy Bypass -File tests/test_h7_current_config.ps1
powershell -ExecutionPolicy Bypass -File tests/test_motor_control_integration.ps1
```

Expected: all checks pass; default control mode and 0.8 A validated command remain unchanged.

- [ ] **Step 6: Commit transport plumbing**

```powershell
git add Core/Inc/motor/biss_encoder.h Core/Src/motor/biss_encoder.c Core/Inc/spi.h Core/Src/spi.c Core/Inc/stm32h7xx_it.h Core/Src/stm32h7xx_it.c Core/Inc/main.h Core/Src/gpio.c Core/Src/main.c emptytest.ioc MDK-ARM/emptytest.uvprojx tests/test_biss_h7_config.ps1
git commit -m "feat: acquire BiSS-C angle over SPI4 DMA"
```

### Task 4: Add calibration data validation and reserved-Flash storage

**Files:**
- Create: `Core/Inc/motor/motor_config_store.h`
- Create: `Core/Src/motor/motor_config_store.c`
- Create: `Core/Inc/motor/crc32.h`
- Create: `Core/Src/motor/crc32.c`
- Create: `tests/test_motor_config.c`
- Modify: `MDK-ARM/emptytest.uvprojx`
- Modify: `tests/test_biss_h7_config.ps1`

**Interfaces:**
- Consumes: a stopped-control authorization flag from `motor_control`.
- Produces: `MotorConfigStore_Load`, `MotorConfigStore_Save`, and `MotorCalibrationConfig_IsValid`.

- [ ] **Step 1: Write failing CRC32 and config-validation tests**

Use a 32-byte Flash-word record:

```c
#define MOTOR_CONFIG_MAGIC       (0x4D434647UL)
#define MOTOR_CONFIG_VERSION     (1U)
#define MOTOR_CONFIG_FLASH_ADDR  (0x081E0000UL)

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t electrical_zero_raw;
    int8_t encoder_direction;
    uint8_t pole_pairs;
    uint8_t calibrated;
    uint8_t reserved0;
    uint32_t crc32;
    uint32_t reserved[3];
} MotorCalibrationConfig;
```

Assert `sizeof(MotorCalibrationConfig) == 32`, validate zero values 0 and 131071, directions `-1/+1`, pole pairs 10, and reject corrupted CRC, raw 131072, direction 0, incorrect magic/version/size, and `calibrated=0`.

- [ ] **Step 2: Run the test and verify it fails**

```powershell
gcc -std=c99 -Wall -Wextra -Werror -ICore/Inc/motor tests/test_motor_config.c Core/Src/motor/crc32.c Core/Src/motor/motor_config_store.c -DMOTOR_CONFIG_HOST_TEST -o tests/test_motor_config.exe
```

Expected: FAIL because the modules are missing.

- [ ] **Step 3: Implement pure validation plus guarded H7 Flash access**

Under `MOTOR_CONFIG_HOST_TEST`, compile only record creation and validation. In the H7 path, load by copying from `0x081E0000`; save by requiring an explicit `safe_to_write=true`, unlocking Flash, erasing Bank2 Sector7, programming one `FLASH_TYPEPROGRAM_FLASHWORD`, locking Flash on every exit path, and reading the record back through `MotorCalibrationConfig_IsValid`.

Never erase or program Flash from an ISR. Return `HAL_BUSY` if safe authorization is false.

- [ ] **Step 4: Reserve the final Flash sector in Keil**

Change the Keil IROM size from `0x00200000` to `0x001E0000`. Extend `test_biss_h7_config.ps1` to assert the reduced size and fixed record address. Do not add generated `.o/.crf/.axf/.hex/.map` files.

- [ ] **Step 5: Run storage and integration tests**

```powershell
& .\tests\test_motor_config.exe
powershell -ExecutionPolicy Bypass -File tests/test_biss_h7_config.ps1
```

Expected: config tests and Flash-layout checks pass.

- [ ] **Step 6: Commit storage support**

```powershell
git add Core/Inc/motor/motor_config_store.h Core/Src/motor/motor_config_store.c Core/Inc/motor/crc32.h Core/Src/motor/crc32.c tests/test_motor_config.c tests/test_biss_h7_config.ps1 MDK-ARM/emptytest.uvprojx
git commit -m "feat: persist encoder electrical calibration"
```

### Task 5: Implement the explicit encoder alignment state machine

**Files:**
- Create: `Core/Inc/motor/encoder_calibration.h`
- Create: `Core/Src/motor/encoder_calibration.c`
- Create: `tests/test_encoder_calibration.c`
- Modify: `Core/Inc/motor/motor_params.h`

**Interfaces:**
- Consumes: 10 kHz `dt_s`, validated position samples, current-sense readiness, and callbacks to enable/disable aligned current control.
- Produces: deterministic calibration commands, final `zero_raw/direction`, progress state, and a precise failure reason.

- [ ] **Step 1: Write failing state-machine tests**

Define states:

```c
typedef enum
{
    ENCODER_CAL_IDLE = 0,
    ENCODER_CAL_WAIT_VALID,
    ENCODER_CAL_ALIGN_ZERO,
    ENCODER_CAL_SETTLE_ZERO,
    ENCODER_CAL_DIRECTION_MOVE,
    ENCODER_CAL_SETTLE_FINAL,
    ENCODER_CAL_COMPLETE,
    ENCODER_CAL_FAILED
} EncoderCalibrationState;
```

Drive the state machine with 100 us steps and synthetic positions. Verify:

- Id ramps from 0 to 0.2 A in 200 ms while Iq remains zero.
- 500 ms settling occurs before 128 samples are averaged.
- a positive 1311-count movement during `+0.1 pu` electrical advance selects direction `+1`.
- the equivalent negative movement selects direction `-1`.
- movement below 256 or above 4096 counts fails.
- circular samples around `131071/0` average near zero rather than half a turn.
- sample peak-to-peak above 128 counts fails.
- missing valid samples and total-state timeout fail with zero current command.

- [ ] **Step 2: Run and verify failure**

```powershell
gcc -std=c99 -Wall -Wextra -Werror -ICore/Inc/motor tests/test_encoder_calibration.c Core/Src/motor/encoder_calibration.c -lm -o tests/test_encoder_calibration.exe
```

Expected: FAIL because the calibration module is missing.

- [ ] **Step 3: Implement the pure calibration machine**

Move all values into `motor_params.h`:

```c
#define MOTOR_ENCODER_ALIGN_CURRENT_A          (0.2f)
#define MOTOR_ENCODER_ALIGN_CURRENT_MAX_A      (0.5f)
#define MOTOR_ENCODER_ALIGN_RAMP_S             (0.2f)
#define MOTOR_ENCODER_ALIGN_SETTLE_S           (0.5f)
#define MOTOR_ENCODER_ALIGN_SAMPLE_COUNT       (128U)
#define MOTOR_ENCODER_ALIGN_STABILITY_COUNT    (128U)
#define MOTOR_ENCODER_DIRECTION_STEP_PU        (0.1f)
#define MOTOR_ENCODER_DIRECTION_MIN_COUNT      (256)
#define MOTOR_ENCODER_DIRECTION_MAX_COUNT      (4096)
```

Return a command structure containing `id_ref_a`, `iq_ref_a`, and `forced_electrical_angle_pu`; do not call HAL or write PWM inside this module.

- [ ] **Step 4: Run calibration and math regression tests**

```powershell
& .\tests\test_encoder_calibration.exe
& .\tests\test_biss_frame.exe
& .\tests\test_encoder_angle.exe
& .\tests\test_current_control.exe
```

Expected: all pass.

- [ ] **Step 5: Commit calibration logic**

```powershell
git add Core/Inc/motor/encoder_calibration.h Core/Src/motor/encoder_calibration.c Core/Inc/motor/motor_params.h tests/test_encoder_calibration.c
git commit -m "feat: calibrate encoder electrical zero"
```

### Task 6: Integrate the encoder angle source into the existing current loop

**Files:**
- Modify: `Core/Inc/motor/motor_control.h`
- Modify: `Core/Src/motor/motor_control.c`
- Modify: `Core/Src/main.c`
- Modify: `tests/test_motor_control_integration.ps1`
- Create: `tests/test_encoder_mode_integration.ps1`

**Interfaces:**
- Consumes: `BissEncoderSnapshot`, `EncoderAngleSample`, saved calibration, and the existing current-loop command/PI modules.
- Produces: `MOTOR_CONTROL_ENCODER_ANGLE_CURRENT`, `MotorControl_SetEncoderCurrentCommand`, calibration request/service APIs, and new encoder faults/debug data. SPI/DMA callbacks remain owned by `biss_encoder`, so `motor_control` only consumes validated snapshots.

- [ ] **Step 1: Write failing integration checks**

Require the following symbols and behavior patterns:

```c
MOTOR_CONTROL_ENCODER_ANGLE_CURRENT
MotorControl_SetEncoderCurrentCommand(float id_a, float iq_a)
MotorControl_RequestEncoderCalibration(void)
MotorControl_Service(void)
BissEncoder_GetSnapshot(BissEncoderSnapshot *snapshot)
```

Require separate angle selection and a single current-loop call:

```text
OPEN_ANGLE_CURRENT -> open_loop_state.electrical_angle_pu
ENCODER_ANGLE_CURRENT -> encoder_angle_sample.electrical_angle_pu
MotorControl_RunCurrentLoop(dt_s, electrical_angle_pu)
```

Assert the existing startup remains `MOTOR_CONTROL_OPEN_ANGLE_CURRENT` and its validated `0.8 A, 1 Hz` command remains in `main.c`.

- [ ] **Step 2: Run integration checks and verify failure**

```powershell
powershell -ExecutionPolicy Bypass -File tests/test_encoder_mode_integration.ps1
```

Expected: FAIL because encoder mode integration is absent.

- [ ] **Step 3: Extend modes, run states, faults, and debug snapshot**

Add an explicit encoder mode and compatibility alias:

```c
MOTOR_CONTROL_ENCODER_ANGLE_CURRENT,
#define MOTOR_CONTROL_ENCODER_CURRENT MOTOR_CONTROL_ENCODER_ANGLE_CURRENT
```

Add encoder-ready, stale, frame, CRC, status, not-calibrated, alignment, and config-storage fault codes. Extend `g_motor_control_debug` with raw bytes, parsed fields, counters, position, mechanical/electrical pu, zero, direction, calibration validity/state, and data age.

- [ ] **Step 4: Select one angle snapshot per current callback**

Change the internal function to:

```c
static bool MotorControl_RunCurrentLoop(float dt_s,
                                        float electrical_angle_pu);
```

For open-angle current mode, pass the existing virtual angle. For encoder mode, atomically obtain one valid snapshot, calculate one `EncoderAngleSample`, copy `electrical_angle_pu` to a local variable, and pass that same value through Park, PI, inverse Park, and SVPWM. Do not advance the virtual angle in encoder mode.

At every TIM8 tick call `BissEncoder_ControlTick()` and start a new DMA read if idle. In non-encoder modes, errors update diagnostics only. In encoder mode, ten ticks without new valid data or DMA busy across two ticks enters fault.

- [ ] **Step 5: Add safe mode entry and command API**

Reject entry unless current calibration, 32-frame encoder readiness, fresh data, and stored encoder calibration are all valid. Clamp encoder `Id_ref/Iq_ref` using the existing current limit and slew machinery. On entry, retain current PI configuration but reset/preload output safely; on exit, reset only encoder-mode transient state.

Do not change the behavior of `MotorControl_SetCurrentCommand(id, iq, frequency)` used by the open-angle mode.

- [ ] **Step 6: Integrate calibration service outside interrupts**

`MotorControl_RequestEncoderCalibration()` only latches a request. `MotorControl_Service()` runs from `while(1)` and performs state transitions and Flash saving. The 10 kHz callbacks consume the current calibration command but never write Flash. Calibration completion first disables PWM and DRV, then calls `MotorConfigStore_Save(..., true)` from `MotorControl_Service()`.

- [ ] **Step 7: Run all host and static tests**

```powershell
& .\tests\test_biss_frame.exe
& .\tests\test_encoder_angle.exe
& .\tests\test_encoder_calibration.exe
& .\tests\test_motor_config.exe
powershell -ExecutionPolicy Bypass -File tests/test_h7_current_config.ps1
powershell -ExecutionPolicy Bypass -File tests/test_motor_control_integration.ps1
powershell -ExecutionPolicy Bypass -File tests/test_biss_h7_config.ps1
powershell -ExecutionPolicy Bypass -File tests/test_encoder_mode_integration.ps1
```

Expected: all tests pass and validated open-loop defaults are unchanged.

- [ ] **Step 8: Commit motor-control integration**

```powershell
git add Core/Inc/motor/motor_control.h Core/Src/motor/motor_control.c Core/Src/main.c tests/test_motor_control_integration.ps1 tests/test_encoder_mode_integration.ps1
git commit -m "feat: run current loop from encoder angle"
```

### Task 7: Add Keil sources and build the complete firmware

**Files:**
- Modify: `MDK-ARM/emptytest.uvprojx`
- Modify: `Core/Src/main.c`
- Modify: `docs/h7-encoder-current-loop-bring-up.md`

**Interfaces:**
- Consumes: all modules from Tasks 1-6.
- Produces: a Keil project that compiles them and a hardware test guide that keeps the default validated mode safe.

- [ ] **Step 1: Add all new source files to the Keil Motor group**

Ensure the project contains:

```text
biss_frame.c
biss_encoder.c
encoder_angle.c
encoder_calibration.c
crc32.c
motor_config_store.c
```

Keep `Core/Inc/motor` in the include path and confirm IROM size is `0x001E0000`.

- [ ] **Step 2: Keep the startup default unchanged and service background work**

Retain:

```c
.mode = MOTOR_CONTROL_OPEN_ANGLE_CURRENT;
MotorControl_SetCurrentCommand(0.0f, 0.8f, 1.0f);
```

Add only the nonblocking foreground service:

```c
while (1)
{
    MotorControl_Service();
}
```

Do not enable encoder current mode automatically in the committed first build.

- [ ] **Step 3: Write the bring-up guide**

Document these exact stages:

1. Build and rerun existing open-angle current mode.
2. Disconnect/disable the power stage and verify PE6 low, PE2 idle high, and approximately 0.9375 MHz clock bursts.
3. Rotate the motor by hand and verify CRC-valid 0～131071 position, Error high, and correct wrap.
4. Trigger calibration with current-limited bus supply and observe 0.2 A alignment.
5. Power-cycle and verify stored zero/direction validity.
6. Enter encoder mode at `Id_ref=0`, `Iq_ref=0.1 A`, then test 0.2 A and 0.3 A.
7. Stop immediately for rising `Id`, incorrect torque direction, CRC bursts, stale angle, abnormal noise, or overcurrent.
8. Confirm both prior modes still run after leaving encoder mode.

- [ ] **Step 4: Run the Keil command-line build**

Locate the installed executable with:

```powershell
$uv4 = @('C:\Keil_v5\UV4\UV4.exe', 'D:\Keil_v5\UV4\UV4.exe') | Where-Object { Test-Path $_ } | Select-Object -First 1
if (-not $uv4) { throw 'UV4.exe not found' }
& $uv4 -b 'MDK-ARM\emptytest.uvprojx' -j0
```

Expected: build log reports `0 Error(s), 0 Warning(s)`. If `UV4.exe` uses a different local path, discover it with `where.exe UV4.exe`; do not download another compiler while a licensed Keil installation is present.

- [ ] **Step 5: Run every regression after the Keil build**

```powershell
powershell -ExecutionPolicy Bypass -File tests/test_h7_current_config.ps1
powershell -ExecutionPolicy Bypass -File tests/test_motor_control_integration.ps1
powershell -ExecutionPolicy Bypass -File tests/test_biss_h7_config.ps1
powershell -ExecutionPolicy Bypass -File tests/test_encoder_mode_integration.ps1
```

Expected: all pass. Inspect `git status --short` and exclude generated Keil products from the next commit.

- [ ] **Step 6: Commit project integration and guide**

```powershell
git add MDK-ARM/emptytest.uvprojx Core/Src/main.c docs/h7-encoder-current-loop-bring-up.md
git commit -m "docs: add encoder current-loop bring-up"
```

### Task 8: Final review and handoff

**Files:**
- Review: all files changed since commit `3c1b035`
- Update only if verification exposes a defect.

**Interfaces:**
- Consumes: completed feature branch and all automated evidence.
- Produces: verified firmware source, an unchanged default open-angle current build, and a staged hardware-validation handoff.

- [ ] **Step 1: Review scope and generated-file hygiene**

```powershell
git diff --stat 3c1b035..HEAD
git diff --check 3c1b035..HEAD
git status --short
```

Expected: only planned source, test, project, and documentation files are committed; user-owned `.uvguix.*`, `.vscode`, `.o`, `.crf`, `.axf`, `.hex`, `.map`, `.elf`, and unrelated files remain uncommitted.

- [ ] **Step 2: Re-run the complete automated suite from a clean command session**

Compile and execute all four host C tests, then run all four PowerShell checks and the Keil build. Expected: every host test passes, every PowerShell check passes, and Keil reports zero errors and zero warnings.

- [ ] **Step 3: Inspect safety-critical implementation points**

Confirm directly in code:

- PE6 is low before the first SPI4 transaction.
- SPI4 idle is high and sampling is on the falling edge.
- no interrupt path calls blocking SPI, delay, UART, or Flash routines.
- CRC-invalid frames never update the published angle.
- Park and inverse Park use one local angle value per ADC callback.
- encoder faults only stop the encoder mode.
- calibration always exits with PWM off on success and failure.
- Flash erase targets only Bank2 Sector7 at `0x081E0000`.
- default startup remains the already validated open-angle current mode.

- [ ] **Step 4: Commit only verified fixes, if any**

If review required a source correction, rerun the affected test first, then commit only those named files with a specific message such as:

```powershell
git add Core/Src/motor/biss_encoder.c tests/test_biss_frame.c
git commit -m "fix: reject stale BiSS-C angle frames"
```

If no correction is required, do not create an empty commit.

- [ ] **Step 5: Report the hardware gate clearly**

State that compilation and host/static tests verify software behavior, but encoder timing, CRC stability, electrical-zero calibration, current polarity, and safe torque direction remain hardware validation gates. Give the user the exact first-power sequence from `docs/h7-encoder-current-loop-bring-up.md` and do not claim hardware success before those measurements are completed.
