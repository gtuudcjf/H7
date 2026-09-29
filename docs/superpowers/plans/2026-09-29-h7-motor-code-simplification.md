# H743 Motor Code Simplification Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 精简 H743 电机控制工程中的调试镜像、历史诊断代码和生成产物，同时保持已通过实机验证的控制行为、必要观测值及 VOFA 协议不变。

**Architecture:** 保持现有硬件适配、算法小模块和 `motor_control.c` 编排层边界。将 `MotorControlDebug` 缩减为现场诊断所需字段，在数据产生处更新；删除独立启动追踪和遥测全局调试镜像，但保留遥测核心的内部端口状态。

**Tech Stack:** C11-compatible embedded C、STM32H7 HAL、Keil MDK ARMCC 5、PowerShell、TinyCC 主机测试、VOFA+ JustFloat。

**Spec:** `docs/superpowers/specs/2026-09-29-h7-motor-code-simplification-design.md`

## Global Constraints

- 不改变五种控制模式、模式枚举值、故障枚举值和状态转换语义。
- 不改变 `motor_params.h` 中的参数、限幅和保护阈值。
- 不改变 TIM8、ADC、SPI4/DMA、DRV8323、USB/UART 配置及中断优先级。
- 保持 10 kHz 电流环、1 kHz 速度/位置环和现有启动顺序。
- VOFA 保持 8 通道、JustFloat、36 字节帧、10 ms 周期和双端口独立发送。
- 保留 `g_motor_control_debug` 名称及必要的 Keil Watch/Ozone 观测能力。
- 不把职责明确的算法模块合并进 `motor_control.c`。
- 所有源码修改使用 UTF-8；关键安全、时序、硬件和物理单位注释必须保留。

## Review Focus

- 调试字段缩减后，模式切换与故障发生时 `mode/requested_mode/run_state/fault` 必须及时更新；Task 2 的集成断言和回归测试覆盖。
- 停机、故障或非位置模式下，VOFA 必须继续发送最近发布的八个有限值而不访问已删除字段；Task 3 的适配测试覆盖。
- UART 忙、USB 未枚举、USB 断开和异步错误不能阻塞另一路端口；Task 3 的核心及适配测试覆盖。
- 删除启动追踪后不得改变 ADC 校准、PWM 使能、编码器等待和安全停机的调用顺序；Task 2 的静态接线断言与 Task 6 的 Keil 构建覆盖。
- 清理 Keil 输出目录后，工程必须能从源码重新全量生成 `.axf/.hex`；Task 5 的项目引用检查和 Task 6 的全量构建覆盖。

---

### Task 1: 固化精简调试契约

**Files:**
- Modify: `tests/test_motor_control_integration.ps1`
- Modify: `tests/test_encoder_mode_integration.ps1`
- Modify: `tests/test_motor_telemetry_integration.ps1`
- Modify: `tests/run_motor_telemetry_suite.ps1`

**Interfaces:**
- Consumes: 当前 `MotorControlDebug` 和 `g_motor_control_debug` 声明。
- Produces: 必要字段白名单、禁止字段黑名单及无历史兼容宏的可执行契约。

- [ ] **Step 1: 在 `test_motor_control_integration.ps1` 增加必要字段断言**

要求结构体继续包含：`mode`、`requested_mode`、`run_state`、`fault`、两相 ADC 原始值与零偏、`id/iq` 给定与反馈、`electrical_angle_pu`、速度目标/反馈/PI 输出、位置目标/反馈/误差、ADC 年龄、过流计数、电压饱和、编码器就绪/校准/位置/年龄/帧状态和五类错误计数。

- [ ] **Step 2: 增加禁止字段与删除模块断言**

用 `Assert-NotContains` 固化以下内容不再出现：PI 比例项和积分器镜像、Clarke/Park 中间量、编码器原始字节及 CRC 明细、启动检查点、前台服务计数和 `motor_startup_trace.h`。在 `test_motor_telemetry_integration.ps1` 中单独断言 `MotorTelemetryDebug` 与 `g_motor_telemetry_debug` 不再导出。

- [ ] **Step 3: 修改编码器模式测试，要求删除旧兼容宏**

删除“别名必须存在”的旧断言，改为断言 `MOTOR_CONTROL_OPEN_LOOP` 和 `MOTOR_CONTROL_ENCODER_CURRENT` 不再定义；正式枚举名必须保留。

- [ ] **Step 4: 将更新后的集成检查加入总套件并验证先失败**

Run: `powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_motor_telemetry_suite.ps1`

Expected: FAIL，原因是旧调试字段、启动追踪、遥测调试全局或兼容宏仍存在。

- [ ] **Step 5: Commit**

```powershell
git add -- tests/test_motor_control_integration.ps1 tests/test_encoder_mode_integration.ps1 tests/test_motor_telemetry_integration.ps1 tests/run_motor_telemetry_suite.ps1
git commit -m "test: define simplified motor diagnostics contract"
```

### Task 2: 精简控制层调试状态并删除启动追踪

**Files:**
- Modify: `Core/Inc/motor/motor_control.h:18-160`
- Modify: `Core/Src/motor/motor_control.c:20-320`
- Modify: `Core/Src/motor/motor_control.c:320-2185`
- Delete: `Core/Inc/motor/motor_startup_trace.h`
- Delete: `Core/Src/motor/motor_startup_trace.c`
- Delete: `tests/test_motor_startup_trace.c`
- Modify: `MDK-ARM/emptytest.uvprojx`
- Test: `tests/test_motor_control_integration.ps1`
- Test: `tests/test_encoder_mode_integration.ps1`

**Interfaces:**
- Consumes: 现有内部状态 `motor_mode`、`requested_mode`、`run_state`、`fault_code`、电流/编码器/速度/位置结果。
- Produces: `typedef struct MotorControlDebug` 的精简字段集合及 `extern volatile MotorControlDebug g_motor_control_debug`。

- [ ] **Step 1: 将 `MotorControlDebug` 缩减到 Task 1 固化的字段**

保留字段名和现有物理单位；不新增新的派生计算。删除两个旧模式名兼容宏以及 `motor_startup_trace.h` include。

- [ ] **Step 2: 删除启动追踪状态和调用**

移除 `startup_trace` 静态变量、初始化、检查点、校准观察和所有调试镜像；不得改变其周围启动、校准、故障和 PWM 调用的相对顺序。

- [ ] **Step 3: 用三个小型发布函数替代全量调试复制**

在 `motor_control.c` 内实现并使用：

```c
static void MotorControl_PublishStateDebug(void);
static void MotorControl_PublishEncoderDebug(void);
static void MotorControl_PublishMotionDebug(void);
```

电流字段继续在 ADC/电流环产生结果的位置直接更新。发布函数只赋值，不执行控制算法、不访问 Flash、不启动外设。

- [ ] **Step 4: 删除所有已移除字段赋值和无效 include**

Run: `rg -n "startup_trace|startup_|foreground_service_count|encoder_raw|received_crc|calculated_crc|speed_pi_proportional|integrator_v|i_alpha|i_beta" Core/Inc/motor/motor_control.h Core/Src/motor/motor_control.c`

Expected: 无匹配；保留字段名称造成的合理匹配除外，并逐项人工确认。

- [ ] **Step 5: 从 Keil 工程删除 `motor_startup_trace.c` 并运行契约测试**

Run: `powershell -NoProfile -ExecutionPolicy Bypass -File tests/test_motor_control_integration.ps1`

Expected: PASS；遥测全局变量的失败由独立的遥测集成测试保留到 Task 3 处理。

- [ ] **Step 6: 运行控制、编码器、速度和位置回归**

Run: `powershell -NoProfile -ExecutionPolicy Bypass -File tests/test_h7_current_config.ps1`

Run: `powershell -NoProfile -ExecutionPolicy Bypass -File tests/test_motor_control_integration.ps1`

Run: `powershell -NoProfile -ExecutionPolicy Bypass -File tests/test_biss_h7_config.ps1`

Run: `powershell -NoProfile -ExecutionPolicy Bypass -File tests/test_encoder_mode_integration.ps1`

Run: `powershell -NoProfile -ExecutionPolicy Bypass -File tests/test_speed_mode_integration.ps1`

Run: `powershell -NoProfile -ExecutionPolicy Bypass -File tests/test_position_mode_integration.ps1`

Expected: 全部 PASS。

- [ ] **Step 7: Commit**

```powershell
git add -- Core/Inc/motor/motor_control.h Core/Src/motor/motor_control.c Core/Inc/motor/motor_startup_trace.h Core/Src/motor/motor_startup_trace.c tests/test_motor_startup_trace.c MDK-ARM/emptytest.uvprojx
git commit -m "refactor: simplify motor control diagnostics"
```

### Task 3: 精简遥测适配层而保持线协议不变

**Files:**
- Modify: `Core/Inc/motor/motor_telemetry.h`
- Modify: `Core/Src/motor/motor_telemetry.c`
- Modify: `tests/test_motor_telemetry_adapter.c`
- Modify: `tests/test_motor_telemetry_integration.ps1`
- Test: `tests/test_motor_telemetry_core.c`

**Interfaces:**
- Consumes: 精简后的 `g_motor_control_debug` 八个 VOFA 字段；`MotorTelemetryCore` 内部端口状态。
- Produces: 原有 `MotorTelemetry_*` 生命周期与回调接口；不再导出 `MotorTelemetryDebug` 或 `g_motor_telemetry_debug`。

- [ ] **Step 1: 先改遥测适配测试以移除全局调试镜像依赖**

测试继续断言：八个 float 的发送顺序、UART/USB 独立提交、busy 时丢帧、不枚举 USB 时不阻塞 UART、UART 错误后可恢复、USB 断开后清 busy。端口计数的详细断言留在 `test_motor_telemetry_core.c`。

- [ ] **Step 2: 运行遥测测试确认先失败**

Run: `powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_motor_telemetry_tests.ps1`

Expected: FAIL，因为生产头文件和适配层仍导出/更新旧遥测调试结构。

- [ ] **Step 3: 删除遥测调试结构和发布函数**

从头文件删除 `MotorTelemetryDebug` 与 extern；从实现删除 `PublishPort()` 和所有 `g_motor_telemetry_debug` 写入。保留 `MotorTelemetryCore` 的内部计数和回调状态转换。

- [ ] **Step 4: 保持八字段短临界区和协议常量**

确认 `MotorTelemetry_Service()` 仍在保存/恢复 PRIMASK 的临界区内只复制八个字段；`MOTOR_TELEMETRY_CHANNEL_COUNT == 8U`、`FRAME_BYTES == 36U`、`PERIOD_MS == 10U`。

- [ ] **Step 5: 运行遥测核心、适配和接线测试**

Run: `powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_motor_telemetry_tests.ps1`

Run: `powershell -NoProfile -ExecutionPolicy Bypass -File tests/test_motor_telemetry_integration.ps1`

Expected: 全部 PASS。

- [ ] **Step 6: Commit**

```powershell
git add -- Core/Inc/motor/motor_telemetry.h Core/Src/motor/motor_telemetry.c tests/test_motor_telemetry_adapter.c tests/test_motor_telemetry_integration.ps1
git commit -m "refactor: keep only essential telemetry state"
```

### Task 4: 全工程冗余审计和可读性整理

**Files:**
- Modify: `Core/Src/main.c`
- Modify: `Core/Src/motor/motor_control.c`
- Modify: `Core/Inc/motor/motor_control.h`
- Modify as justified: `Core/Src/motor/*.c`, `Core/Inc/motor/*.h`
- Modify: `tests/test_motor_control_integration.ps1`

**Interfaces:**
- Consumes: Tasks 2-3 的精简接口。
- Produces: 无未使用生产接口、无重复安全退出片段、关键注释完整的控制工程。

- [ ] **Step 1: 生成生产符号引用清单并逐项分类**

Run: `rg -n "^[A-Za-z_][A-Za-z0-9_ *]*\b[A-Za-z][A-Za-z0-9_]+\(" Core/Inc/motor Core/Src/motor`

将每个疑似未调用符号分类为：HAL 回调入口、公开控制接口、测试使用接口、模块内部使用、可删除。公开在线整定和模式切换接口即使当前 `main.c` 未调用也保留。

- [ ] **Step 2: 为确认可删除的符号增加静态反向断言**

在 `test_motor_control_integration.ps1` 中按确切符号名增加 `Assert-NotContains`，避免使用宽泛模式误伤 HAL 或公开接口。

- [ ] **Step 3: 运行测试确认这些断言先失败**

Run: `powershell -NoProfile -ExecutionPolicy Bypass -File tests/test_motor_control_integration.ps1`

Expected: FAIL，且仅指向审计确认的冗余符号或重复结构。

- [ ] **Step 4: 删除确认冗余并整理控制编排**

删除无调用符号、无效 include 和重复注释；仅合并语义完全一致的复位、限幅或安全退出代码。不得改变不同状态分支的执行顺序，也不得把状态机改写成复杂宏或表驱动代码。

- [ ] **Step 5: 精简 `main.c` 注释**

保留：10 kHz 时间基准、启动/停机安全顺序、五模式含义、24 V 实验参数边界、Flash 只能在前台服务、HAL 回调职责。删除重复说明和历史开发过程描述。

- [ ] **Step 6: 运行总回归**

Run: `powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_motor_telemetry_suite.ps1`

Expected: 8 项总套件全部 PASS，数量按 Task 1 最终登记为准。

- [ ] **Step 7: Commit**

```powershell
git add -- Core tests
git commit -m "refactor: remove redundant motor code"
```

### Task 5: 清理 Keil 生成物并更新现行文档

**Files:**
- Modify: `.gitignore`
- Delete: `MDK-ARM/emptytest/`
- Delete: `MDK-ARM/vofa-build.log`
- Delete: `MDK-ARM/JLinkLog.txt`
- Delete: `MDK-ARM/startup_stm32h743xx.lst`
- Delete: `MDK-ARM/emptytest.uvguix.tx`
- Modify: `docs/h7-motor-control-code-analysis.md`
- Modify: `docs/h7-vofa-telemetry.md`
- Modify as referenced: `docs/h7-*-bring-up.md`

**Interfaces:**
- Consumes: 最终调试字段和模块清单。
- Produces: 可从源码重建、默认不跟踪生成物的 Keil 工程及与当前接口一致的使用文档。

- [ ] **Step 1: 记录并验证待删除路径边界**

Run: `Resolve-Path 'MDK-ARM/emptytest'; git ls-files 'MDK-ARM/emptytest/**' 'MDK-ARM/vofa-build.log' 'MDK-ARM/JLinkLog.txt' 'MDK-ARM/startup_stm32h743xx.lst' 'MDK-ARM/emptytest.uvguix.tx'`

Expected: 所有目标均位于仓库的 `MDK-ARM` 目录；不包含 `.uvprojx`、`.uvoptx`、启动 `.s`、`DebugConfig` 或 `JLinkSettings.ini`。

- [ ] **Step 2: 在 `.gitignore` 增加精确 Keil 生成物规则**

忽略 `MDK-ARM/emptytest/`、`MDK-ARM/*.log`、`MDK-ARM/*.lst`、`MDK-ARM/*.uvguix.*`；不得忽略整个 `MDK-ARM/`。

- [ ] **Step 3: 删除已确认的生成物**

使用 Git 删除 Step 1 列出的精确目标。删除后运行 `git status --short`，确认项目文件和源码未进入删除列表。

- [ ] **Step 4: 更新当前使用文档**

将调试字段表改为精简集合；删除 `g_motor_telemetry_debug` 和启动追踪操作说明；保留实机安全、VOFA 接线、通道、周期和验证边界。历史 `docs/superpowers/specs` 与 `plans` 不回写。

- [ ] **Step 5: 检查旧引用和忽略规则**

Run: `rg -n "motor_startup_trace|g_motor_telemetry_debug|startup_trace_count|foreground_service_count" Core tests docs/h7-*.md MDK-ARM/emptytest.uvprojx`

Expected: 无匹配。

Run: `git check-ignore -v MDK-ARM/emptytest/example.o MDK-ARM/example.log MDK-ARM/example.lst MDK-ARM/example.uvguix.user`

Expected: 四个示例路径均由新规则忽略。

- [ ] **Step 6: Commit**

```powershell
git add -A -- .gitignore MDK-ARM docs/h7-motor-control-code-analysis.md docs/h7-vofa-telemetry.md docs/h7-*-bring-up.md
git commit -m "chore: remove generated Keil artifacts"
```

### Task 6: 全量验证和结果审计

**Files:**
- Verify only: all modified files
- Generated then ignored: `MDK-ARM/emptytest/`

**Interfaces:**
- Consumes: Tasks 1-5 的全部结果。
- Produces: 主机回归、Keil 全量构建、结构搜索和差异审计证据。

- [ ] **Step 1: 运行完整主机回归**

Run: `powershell -NoProfile -ExecutionPolicy Bypass -File tests/run_motor_telemetry_suite.ps1`

Expected: 所有登记检查 PASS，退出码 0。

- [ ] **Step 2: 使用 Keil 命令行全量构建**

Run: `& 'D:\Keil_v5\UV4\UV4.exe' -b 'MDK-ARM\emptytest.uvprojx' -j0 -o 'MDK-ARM\simplification-build.log'`

Expected: 退出码 0；日志包含 `0 Error(s), 0 Warning(s)`；重新生成的输出目录和日志由 `.gitignore` 忽略。

- [ ] **Step 3: 验证关键配置没有变化**

Run: `git diff ca48edd -- Core/Inc/motor/motor_params.h Core/Src/tim.c Core/Src/adc.c Core/Src/spi.c Core/Src/usart.c Core/Src/motor/drv8323.c Core/Inc/motor/motor_runtime_policy.h`

Expected: 除明确的无效 include/注释整理外无差异；任何数值、枚举或寄存器配置差异均必须回退。

- [ ] **Step 4: 验证删除目标和保留接口**

Run: `rg -n "motor_startup_trace|g_motor_telemetry_debug|MOTOR_CONTROL_OPEN_LOOP|MOTOR_CONTROL_ENCODER_CURRENT" Core tests MDK-ARM/emptytest.uvprojx`

Expected: 无匹配。

Run: `rg -n "g_motor_control_debug|MOTOR_TELEMETRY_CHANNEL_COUNT|MOTOR_TELEMETRY_FRAME_BYTES|MOTOR_TELEMETRY_PERIOD_MS" Core tests`

Expected: 精简调试结构和 8/36/10 常量仍存在且被测试覆盖。

- [ ] **Step 5: 审计最终差异**

Run: `git diff --check`

Run: `git status --short`

Run: `git diff --stat ca48edd..HEAD`

Expected: 无空白错误；只包含规格范围内源码、测试、项目、文档和生成物清理。

若验证发现问题，返回拥有该代码的任务补充失败测试、修正并重新执行 Task 6；不得在验证阶段以未记录的临时改动绕过失败，也不得提交 Keil 输出。
