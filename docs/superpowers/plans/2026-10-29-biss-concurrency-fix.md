# BiSS 接收并发修复实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 修复 20 kHz 控制抢占下 DMA 缓冲区过早复用与快照发布不一致，保留通信配置和保护时间。

**Architecture:** 用独立解析状态占有接收缓冲区，DMA 硬件超时只覆盖实际传输。CRC 解析允许被抢占，快照发布与所有权交接放入短临界区；不将整个解析放入关中断区。

**Tech Stack:** STM32H743、HAL SPI4/DMA、ARMCC5/Keil、TinyCC 主机测试。

**Spec:** `docs/h7-20k-encoder-crc-analysis.md` 的修正顺序，以及用户本轮授权结合新录屏实施修复。

## Global Constraints

- 不改变 TIM8 20 kHz、电流环 20 kHz、速度环 2 kHz、编码器请求 10 kHz。
- 不改变 SPI 模式/时钟、CRC 校验、约 300 μs DMA 超时、约 1.1 ms 过期保护。
- 保留用户 main.c 的电流给定和 Keil 设置；不烧录、不提交。
- 新录屏静止条件下 CRC 增长较慢，不能将并发缺陷直接等同于已证实的实机全部根因。

## Review Focus

- 有效帧解析中被抢占，不得启动复用 RX 缓冲区的 DMA。
- 解析阶段控制 tick 继续维护有效帧年龄，但不能误触发硬件 DMA 超时。
- 高优先级读取只能看到完整旧快照或完整新快照。
- CRC 错误、保护区损坏、SPI 错误、中止及初始化后应能重新采集。
- 保持调用前 PRIMASK；不能解析时全程关闭中断。

## Task 1: 接收并发修复与回归

**Files:**
- Create: `tests/test_biss_concurrency.c`
- Modify: `tests/run_control_tests.py`
- Modify: `Core/Src/motor/biss_encoder.c`
- Modify: `Core/Inc/motor/biss_encoder.h`
- Modify: `docs/h7-20k-encoder-crc-analysis.md`

- [x] 为真实解析器和真实接收实现加入抢占、快照读取、解析期间超时及错误恢复测试，确认旧实现失败。
- [x] 增加解析所有权状态；在短临界区交接、发布和释放；同步头文件语义。
- [x] 运行 24/48 V 主机回归、PowerShell 集成检查及编译器 CLI 测试。
- [x] 完成 24/48 V Keil 构建，交付默认保持 24 V。
- [x] 请求独立代码审查并处理发现；更新诊断报告和实机对照步骤。

## 执行记录

- RED：原接收实现 ownership 在解析期间已释放 busy，publication 读到混合快照，两项分别失败。GREEN：修复后通过。
- RED：TX guard 失败后连续有效计数未清零，采一帧就恢复 ready；补清零后 GREEN。
- 验证：39 项主机检查、2 项 CLI 测试通过；24/48 V Keil 均 0 错误/0 警告；git diff --check 通过。
- PowerShell 入口 6/9 通过，另 3 项均为既有零 Iq 启动断言与用户当前 0.7 A 配置冲突。保留用户设置与原断言，报告该限制，不宣称全套通过。
- 独立审查及补充审查未发现本轮新增问题；既有 HAL EOT→用户回调入口的超时/abort 窗口在报告中记录，需实机时序验证。
- Ruling: 在当前 feat/control-20k-rad-s-dual-bus 分支就地修改，因用户共享工程有未提交升级与试验设置，不另建缺少这些状态的工作树。
- Ruling: 只修改接收层及测试/说明，不改变用户 0.7 A 启动或通过放宽集成断言获得绿色结果。
- 未烧录、未提交、未推送。
