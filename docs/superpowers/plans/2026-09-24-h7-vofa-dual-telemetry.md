# H743 VOFA+ 双串口三环遥测 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在不改变电机控制行为的前提下，让 USART1 与 USB CDC 独立输出 VOFA+ 可识别的八通道三环波形。

**Architecture:** 纯 C 核心负责 100 Hz 节拍、JustFloat 编码、每端口缓冲区与忙状态；薄的 H7 适配层只在主循环复制现有调试字段，并把发送请求交给 UART 中断和 USB CDC。每端口出错只丢自己的帧，不调用电机故障入口。

**Tech Stack:** STM32H743 HAL、Keil MDK-ARM、USB Device CDC、C99、PowerShell 集成检查、TinyCC 主机断言测试。

**Spec:** `docs/superpowers/specs/2026-09-24-h7-vofa-dual-telemetry-design.md`

## Global Constraints

- 固定 VOFA+ JustFloat 帧：八个小端 IEEE-754 float32，按 Id目标、Id反馈、Iq目标、Iq反馈、活动速度目标、滤波转速、位置目标、位置反馈排序，末尾 `00 00 80 7F`，总长 36 字节。
- 默认两端口均启用，采样间隔 10 ms；USB CDC 与 USART1 可独立启用/停用，也可同时使用。
- USART1 保持 PB14/PB15、115200/8N1；USB FS 保持 PA11/PA12；不改变电机控制器、采样、PWM、BiSS-C 的代码或参数。
- 控制中断不做遥测取样或发送；主循环中不调用阻塞 UART API、不等待 USB、不重发积压样本。
- 两个端口持有各自的 36 字节异步发送缓冲区；未连接、忙或发送失败仅丢弃该端当前帧。
- 仅在复制八个 `g_motor_control_debug` 字段时短暂关中断，恢复进入前的 PRIMASK；所有编码与发送均在临界区外。
- 非有限浮点值只在遥测副本中替换为零并计数；不回写电机控制状态。

## Review Focus

1. `HAL_GetTick()` 在 `uint32_t` 回绕时仍按 10 ms 周期产生新帧：Task 2 的回绕测试。
2. UART 正在发送而 USB 空闲时，USB 仍收到自己的帧且 UART 缓冲区保持不变：Task 2 的独立端口测试。
3. USB 未枚举或断开后重连时不会卡住 busy，也不会影响 UART：Task 3 的接线检查和 Task 2 的错误/完成状态测试。
4. 非有限数据不会把 NaN/Inf 的位型混进 JustFloat 通道：Task 1 的异常值测试。
5. 主循环被 Flash 保存延迟后不会突发补发历史数据：Task 2 的跳时测试。

---

## 文件结构

| 文件 | 职责 |
|---|---|
| `Core/Inc/motor/motor_telemetry_core.h`, `Core/Src/motor/motor_telemetry_core.c` | 与 HAL 无关的节拍、编码、双端口发送状态 |
| `Core/Inc/motor/motor_telemetry.h`, `Core/Src/motor/motor_telemetry.c` | H7 调试快照、UART/CDC 适配及对外开关 |
| `tests/test_motor_telemetry_core.c` | 执行协议、节拍、独立端口断言 |
| `tests/run_motor_telemetry_tests.ps1` | 准备主机 C 编译器并运行断言测试 |
| `tests/test_motor_telemetry_integration.ps1` | 检查 ISR/CDC/主循环/Keil 工程接线 |
| `docs/h7-vofa-telemetry.md` | VOFA+ 通道和双端口使用说明 |

### Task 1: JustFloat 帧编码

**Files:**
- Create: `Core/Inc/motor/motor_telemetry_core.h`
- Create: `Core/Src/motor/motor_telemetry_core.c`
- Create: `tests/test_motor_telemetry_core.c`
- Create: `tests/run_motor_telemetry_tests.ps1`

**Interfaces:**
- Produces: `MotorTelemetryCore_Encode(const float values[8], uint8_t frame[36]) -> uint32_t`，返回被替换为零的非有限值个数。
- Constants: `MOTOR_TELEMETRY_CHANNEL_COUNT=8`, `MOTOR_TELEMETRY_FRAME_BYTES=36`, `MOTOR_TELEMETRY_PERIOD_MS=10`。

- [ ] **Step 1: 先写失败的协议测试。** `test_motor_telemetry_core.c` 先只测试编码：

```c
static void TestJustFloatFrame(void)
{
    const float values[8] = {1.0f, -2.0f, 0.5f, 0.0f, 20.0f, 19.0f, 60.0f, 59.0f};
    uint8_t frame[MOTOR_TELEMETRY_FRAME_BYTES] = {0};
    const uint8_t one[] = {0x00, 0x00, 0x80, 0x3f};
    const uint8_t minus_two[] = {0x00, 0x00, 0x00, 0xc0};
    const uint8_t tail[] = {0x00, 0x00, 0x80, 0x7f};
    assert(MotorTelemetryCore_Encode(values, frame) == 0U);
    assert(memcmp(frame, one, 4) == 0);
    assert(memcmp(frame + 4, minus_two, 4) == 0);
    assert(memcmp(frame + 32, tail, 4) == 0);
}

static void TestNonFiniteValuesBecomeZero(void)
{
    const float values[8] = {NAN, INFINITY, -INFINITY, 4, 5, 6, 7, 8};
    uint8_t frame[MOTOR_TELEMETRY_FRAME_BYTES];
    assert(MotorTelemetryCore_Encode(values, frame) == 3U);
    for (unsigned i = 0; i < 12; ++i) assert(frame[i] == 0U);
}
```

- [ ] **Step 2: 运行测试并确认因编码器尚不存在而失败。** 使用 `tests/run_motor_telemetry_tests.ps1`；脚本先尝试 PATH 中的 `tcc.exe`，否则展开现有 `$env:TEMP\codex_position_tcc_20260922\tcc32.zip` 到 `$env:TEMP\h7_vofa_tcc`，找到其中的 `tcc.exe`，以 `-Wall -Werror -I Core/Inc/motor` 编译测试和核心源文件，然后执行生成的 exe。脚本对编译和执行的非零退出码均抛出异常。命令：

```powershell
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$cc = (Get-Command tcc.exe -ErrorAction SilentlyContinue).Source
if (-not $cc) {
    $archive = Join-Path $env:TEMP 'codex_position_tcc_20260922\tcc32.zip'
    $toolDir = Join-Path $env:TEMP 'h7_vofa_tcc'
    if (-not (Test-Path -LiteralPath $archive)) { throw 'TinyCC archive unavailable.' }
    Expand-Archive -LiteralPath $archive -DestinationPath $toolDir -Force
    $cc = (Get-ChildItem -LiteralPath $toolDir -Filter tcc.exe -Recurse |
           Select-Object -First 1).FullName
}
if (-not $cc) { throw 'TinyCC executable unavailable.' }
$testExe = Join-Path $env:TEMP 'test_motor_telemetry_core.exe'
Push-Location $root
try {
    & $cc -Wall -Werror -I Core/Inc/motor tests/test_motor_telemetry_core.c `
        Core/Src/motor/motor_telemetry_core.c -o $testExe
    if ($LASTEXITCODE -ne 0) { throw 'Telemetry core compile failed.' }
    & $testExe
    if ($LASTEXITCODE -ne 0) { throw 'Telemetry core test failed.' }
} finally { Pop-Location }
```

执行命令：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_motor_telemetry_tests.ps1
```

- [ ] **Step 3: 实现最小编码器。** 用 `memcpy` 把每个有限 float 的 32 位原始位型取到 `uint32_t`，按低字节到高字节写入帧；非有限值写零。用编译时断言限制 `sizeof(float)==4`，并写固定帧尾。避免直接把 float 数组强转成字节流，因为 Keil/主机的对齐与字节序意图需要显式表达。

```c
for (uint32_t i = 0; i < MOTOR_TELEMETRY_CHANNEL_COUNT; ++i) {
    uint32_t bits = 0U;
    if (isfinite(values[i])) memcpy(&bits, &values[i], sizeof(bits));
    else ++invalid_count;
    for (uint32_t b = 0; b < 4U; ++b)
        frame[4U * i + b] = (uint8_t)(bits >> (8U * b));
}
frame[32] = 0x00U; frame[33] = 0x00U;
frame[34] = 0x80U; frame[35] = 0x7fU;
```

- [ ] **Step 4: 同一命令确认绿灯，且扩展测试覆盖全部八个字段的解码回读。** 用 `memcpy` 从每四字节重新组成 float，逐一比较原值，防止只检查头尾而漏掉中间通道。
- [ ] **Step 5: 提交该任务的核心编码与测试。** `git add -- Core/Inc/motor/motor_telemetry_core.h Core/Src/motor/motor_telemetry_core.c tests/test_motor_telemetry_core.c tests/run_motor_telemetry_tests.ps1`，随后 `git commit -m "feat: encode VOFA JustFloat telemetry frames"`。

### Task 2: 双端口节拍与异步缓冲区状态

**Files:**
- Modify: `Core/Inc/motor/motor_telemetry_core.h`
- Modify: `Core/Src/motor/motor_telemetry_core.c`
- Modify: `tests/test_motor_telemetry_core.c`

**Interfaces:**
- `MotorTelemetryCore_Init(MotorTelemetryCore *core, uint32_t now_ms, bool uart_enabled, bool usb_enabled)`。
- `MotorTelemetryCore_SetEnabled(MotorTelemetryCore *core, MotorTelemetryPortId id, bool enabled)`。
- `MotorTelemetryCore_Service(MotorTelemetryCore *core, uint32_t now_ms, const float values[8], const MotorTelemetrySink sinks[2])`；`MotorTelemetrySink` 含发送函数与上下文，发送函数返回 `ACCEPTED`、`SKIP` 或 `ERROR`。
- `MotorTelemetryCore_OnComplete(MotorTelemetryCore *core, MotorTelemetryPortId id)`；`MotorTelemetryCore_OnError(...)`；每端口持久帧缓冲区、`volatile bool busy`、已完成、丢帧、错误计数。

- [ ] **Step 1: 先写失败的行为测试。** 假发送端保存传入指针、帧副本和调用次数；不立即完成，模拟真实异步发送。分别断言：9 ms 不发送、10 ms 双端发送、UART 未完成时只有 USB 继续发送、UART 原缓冲区未被覆盖、USB 忙时 UART 继续发送、两端分别关闭/重开、`OnComplete` 释放 busy、`OnError` 计错并释放 busy、`UINT32_MAX` 时间回绕仍有节拍、从 10 ms 跳到 1000 ms 只派发一次。样例：

```c
MotorTelemetryCore_Init(&core, 0U, true, true);
MotorTelemetryCore_Service(&core, 9U, values, sinks);
assert(uart.calls == 0U && usb.calls == 0U);
MotorTelemetryCore_Service(&core, 10U, values, sinks);
assert(uart.calls == 1U && usb.calls == 1U);
memcpy(previous_uart_frame, core.port[MOTOR_TELEMETRY_UART].frame, 36U);
MotorTelemetryCore_OnComplete(&core, MOTOR_TELEMETRY_USB);
MotorTelemetryCore_Service(&core, 20U, changed_values, sinks);
assert(uart.calls == 1U && usb.calls == 2U);
assert(memcmp(previous_uart_frame,
              core.port[MOTOR_TELEMETRY_UART].frame, 36U) == 0);
```

- [ ] **Step 2: 运行 `tests/run_motor_telemetry_tests.ps1`，确认因核心调度 API 缺失而失败。**
- [ ] **Step 3: 实现最小状态机。** `now_ms - last_ms < 10` 时返回；到期即把 `last_ms` 设置为当前值，绝不循环补发。仅在至少一个端口启用时编码一次，将帧复制到每个空闲端口的持久缓冲区；先置 busy 再调用异步发送，`ACCEPTED` 保留 busy，`SKIP` 清 busy 并计丢帧，`ERROR` 清 busy 并计错误。忙端口只计丢帧，不覆写帧。完成回调清 busy 并计完成；错误回调清 busy 并计错误。

```c
if ((uint32_t)(now_ms - core->last_ms) < MOTOR_TELEMETRY_PERIOD_MS) return;
core->last_ms = now_ms;
uint8_t encoded_frame[MOTOR_TELEMETRY_FRAME_BYTES];
core->invalid_values += MotorTelemetryCore_Encode(values, encoded_frame);
for (uint32_t i = 0; i < MOTOR_TELEMETRY_PORT_COUNT; ++i) {
    if (!core->port[i].enabled) continue;
    if (core->port[i].busy) { ++core->port[i].dropped; continue; }
    memcpy(core->port[i].frame, encoded_frame, MOTOR_TELEMETRY_FRAME_BYTES);
    core->port[i].busy = true;
    result = sinks[i].send(sinks[i].context, core->port[i].frame,
                           MOTOR_TELEMETRY_FRAME_BYTES);
    if (result != MOTOR_TELEMETRY_SEND_ACCEPTED) {
        core->port[i].busy = false;
        if (result == MOTOR_TELEMETRY_SEND_SKIP) ++core->port[i].dropped;
        else ++core->port[i].errors;
    }
}
```

- [ ] **Step 4: 运行主机测试，检查断言全部通过；对失败输入（空指针、非法端口编号）也有明确安全返回。**
- [ ] **Step 5: 提交核心调度与测试。** `git add -- Core/Inc/motor/motor_telemetry_core.h Core/Src/motor/motor_telemetry_core.c tests/test_motor_telemetry_core.c`，随后 `git commit -m "feat: schedule independent UART and USB telemetry"`。

### Task 3: 接入 H7 外设、Keil 工程与 VOFA+ 文档

**Files:**
- Create: `Core/Inc/motor/motor_telemetry.h`
- Create: `Core/Src/motor/motor_telemetry.c`
- Create: `tests/test_motor_telemetry_integration.ps1`
- Create: `docs/h7-vofa-telemetry.md`
- Modify: `Core/Src/main.c`
- Modify: `Core/Src/usart.c`
- Modify: `Core/Src/stm32h7xx_it.c`
- Modify: `USB_DEVICE/App/usbd_cdc_if.c`
- Modify: `MDK-ARM/emptytest.uvprojx`
- Modify: `emptytest.ioc`

**Interfaces:**
- `MotorTelemetry_Init(UART_HandleTypeDef *uart, bool uart_enabled, bool usb_enabled)`；`MotorTelemetry_Service(void)`；`MotorTelemetry_SetUartEnabled(bool)`；`MotorTelemetry_SetUsbEnabled(bool)`。
- `MotorTelemetry_OnUartComplete(UART_HandleTypeDef *uart)`；`MotorTelemetry_OnUartError(UART_HandleTypeDef *uart)`；`MotorTelemetry_OnUsbComplete(void)`；`MotorTelemetry_OnUsbDisconnected(void)`。
- `g_motor_telemetry_debug` 暴露每端口完成、丢帧、错误和无效字段计数供 Keil Watch 使用；不包含控制命令。

- [ ] **Step 1: 先写失败的接线测试。** `tests/test_motor_telemetry_integration.ps1` 读取源码与工程文件，断言主循环在 `MotorControl_Service()` 后调用 `MotorTelemetry_Service()`；初始化位于 `MotorControl_Start()` 后；`USART1_IRQHandler` 调用 `HAL_UART_IRQHandler(&huart1)`；USART1 NVIC 优先级为 5 且启用；UART 完成/错误回调转发；CDC 完成/DeInit 回调转发；`CDC_Transmit_FS` 在 `pClassData` 为空或设备未配置时安全返回；Keil 工程包含 `motor_telemetry_core.c` 和 `motor_telemetry.c`；`.ioc` 保存 USART1 IRQ。还要断言控制环的 `motor_control.c` 与 `motor_params.h` 未作为此任务的修改目标（用 `git diff --name-only` 手工复核）。

```powershell
$main = Get-Content -Raw -Encoding UTF8 (Join-Path $root 'Core/Src/main.c')
$irq = Get-Content -Raw -Encoding UTF8 (Join-Path $root 'Core/Src/stm32h7xx_it.c')
$project = Get-Content -Raw -Encoding UTF8 (Join-Path $root 'MDK-ARM/emptytest.uvprojx')
if ($main -notmatch 'MotorTelemetry_Service\s*\(') { throw 'Telemetry service not wired.' }
if ($irq -notmatch 'USART1_IRQHandler[\s\S]*?HAL_UART_IRQHandler\s*\(\s*&huart1') {
    throw 'USART1 TX interrupt not wired.'
}
foreach ($name in @('motor_telemetry_core.c', 'motor_telemetry.c')) {
    if ($project -notmatch [regex]::Escape($name)) { throw "Keil missing $name" }
}
```

- [ ] **Step 2: 运行 `powershell -NoProfile -ExecutionPolicy Bypass -File tests/test_motor_telemetry_integration.ps1`，确认因尚未接入而失败。**
- [ ] **Step 3: 实现 H7 适配层。** 在主循环服务到期时，以保存/恢复 PRIMASK 的短临界区复制八个调试字段，按照规格顺序交给核心。UART sink 调用 `HAL_UART_Transmit_IT()`；`HAL_OK` 映射 `ACCEPTED`、`HAL_BUSY` 映射 `SKIP`、其他映射 `ERROR`。USB sink 先检查 `hUsbDeviceFS.dev_state==USBD_STATE_CONFIGURED` 与 `pClassData!=NULL`，再调用 `CDC_Transmit_FS()`；`USBD_OK`、`USBD_BUSY` 和失败分别映射三种结果。USB 完成回调仅释放 USB 端；DeInit 按错误完成并清 busy。适配层的完成/错误回调只处理传入句柄为 USART1 的情况。`MotorTelemetry_Service()` 在任何 ISR 内不得调用。

```c
uint32_t primask = __get_PRIMASK();
__disable_irq();
values[0] = g_motor_control_debug.id_ref_a;
values[1] = g_motor_control_debug.id_a;
values[2] = g_motor_control_debug.iq_ref_a;
values[3] = g_motor_control_debug.iq_a;
values[4] = g_motor_control_debug.speed_active_target_rpm;
values[5] = g_motor_control_debug.speed_filtered_rpm;
values[6] = g_motor_control_debug.position_target_deg;
values[7] = g_motor_control_debug.position_feedback_deg;
if (primask == 0U) __enable_irq();
```

- [ ] **Step 4: 接通生成代码的用户区与项目文件。** USART1 IRQ 设置优先级 5（TIM8/ADC 为 0，SPI4 DMA 为 1，USB 为 4）；`main()` 增加初始化/服务/UART回调；CDC 完成和 DeInit 通知遥测模块；`.ioc` 添加 USART1 NVIC 配置；Keil `Application/User/Motor` 组追加两个源文件。`CDC_Transmit_FS` 先检查设备状态和类数据指针，再解引用 `TxState`。不修改现有 `MotorControl_*` 调用顺序或命令数值。
- [ ] **Step 5: 编写 `docs/h7-vofa-telemetry.md`。** 写明串口端 115200/8N1、USB 端选择 Windows 新枚举 COM 口，两者均选 `JustFloat`，八通道名称和单位，分别使能的方法、断开/忙时丢帧的含义，以及上位机只开一个端口也可工作的验证方法。
- [ ] **Step 6: 运行接线检查、主机测试和现有集成检查。**

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_motor_telemetry_tests.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File tests/test_motor_telemetry_integration.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File tests/test_h7_current_config.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File tests/test_motor_control_integration.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File tests/test_biss_h7_config.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File tests/test_encoder_mode_integration.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File tests/test_speed_mode_integration.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File tests/test_position_mode_integration.ps1
```

- [ ] **Step 7: 运行 Keil 全量构建，并查看日志中的 error/warning 与新源文件名。**

```powershell
& 'D:\Keil_v5\UV4\UV4.exe' -r 'MDK-ARM\emptytest.uvprojx' -o "$env:TEMP\h7_vofa_rebuild.log"
Get-Content -LiteralPath "$env:TEMP\h7_vofa_rebuild.log" -Encoding Default | Select-Object -Last 60
```

- [ ] **Step 8: 对照规格逐条复核，执行 `git diff --check` 与 `git diff --name-only`。** 记录软件验证结果与“仍需用户在实体电机上验证两个 COM 口及实时故障计数”的边界；不把软件构建成功表述为实机验证。
- [ ] **Step 9: 提交集成与文档。** 只暂存本任务文件，随后 `git commit -m "feat: stream motor loops over UART and USB CDC"`。
