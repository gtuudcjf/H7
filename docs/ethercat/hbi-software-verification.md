# HBI 软件交付验证记录

日期：2026-10-24。基线为当前工程 `d973112`；SSC、对象字典和应用来源为 SPI OP 提交 `60f64449`。实现保留在当前工作目录，未提交或推送。

本记录只确认软件验证。没有烧录 MCU、写 LAN9252 EEPROM、操作 TwinCAT 或完成实板 OP 验收。

## 构建结果

工具链：本机 Keil ARMCC 5.06 update 5 build 528。对通信模式和电机模式分别完整 rebuild，均为 **0 errors / 0 warnings**。

| 模式 | Code | RO-data | RW-data | ZI-data | 用途 |
|---|---:|---:|---:|---:|---|
| `ETHERCAT_BRINGUP_ONLY=1` | 37074 | 2066 | 1144 | 8336 | 本次 HBI 通信固件 |
| `ETHERCAT_BRINGUP_ONLY=0` | 64040 | 844 | 476 | 10748 | 保留路径编译回归，不用于本次上板 |

最终通信 HEX SHA256：

```text
B625E92559F5C61BA1B5AA392070EB4E400376D177E240E60F57AF0769BC4E6C
```

工程输出为 `MDK-ARM/emptytest/emptytest.hex`；单独交付副本为 `output/hbi/H743-LAN9252-HBI-LAB.hex`，同目录有匹配 ESI、散列和源文件快照清单。不要使用早先构建的散列或电机编译回归 HEX。

实际 ARM 预处理检查了两种启动路径；通信模式没有电机/ADC/TIM/SPI/USB/USART 初始化调用。链接 map 保留 HBI、SSC、应用和服务，移除了 `MotorControl_Start/Init`。最终反汇编确认 FMC 使用 `LDRH/STRH` 半字指令及 `DSB`，DWORD 恢复间隔使用 DWT 周期差等待。

## 主机测试

GCC 16.2.0，C99，`-Wall -Werror`。对未修改的历史 SSC 源码，仅关闭 GCC 的 misleading-indentation 警告。原 TinyCC 缓存缺失已通过共享编译器发现逻辑处理，没有安装依赖。

| 检查 | 本轮结果 |
|---|---|
| 项目、ESI、CubeMX 配置 | 83 个编译输入存在；身份、SM、PDO、名称长度和配置一致 |
| HBI 驱动 | 481501 项断言通过；包括 1～130 字节 PRAM、所有首字节偏移、CSR 对齐、FIFO、超时与回绕 |
| GPIO/FMC/MPU/板级时基 | 150 项断言通过；另有锁定 DWT 启动及停滞计数器的独立执行检查 |
| 硬件端口 | 146 项断言通过，包括时基错误时不探测 LAN9252 |
| 服务层 | 36 项断言通过 |
| 真实 SSC | polling 99、SDO 75、mailbox 125、object 157、mailbox length 6257、application 372 项断言通过 |
| 原电机回归 | 原有 8 个顶层检查全部通过；电机实现源文件无差异 |

这些数字是测试断言数，不是实板通信周期数。完整日志在 `verification/2026-10-24-hbi/`。

## 复核和修正

- 使用临时源码副本故意改错半字地址、删除 DWORD 恢复间隔、将 PB5 拉高；对应测试均能检测到错误，生产文件未用于这些故障注入。
- 独立代码复核发现 Cortex-M7 DWT 软件锁的冷启动问题。新增测试先复现缺失解锁，再修复为显式写 LAR、固定次数进度检查；计数器不工作时返回 `LAN9252_ERR_CLOCK=12`，不访问 LAN9252。复核者再次检查后确认该问题已修复。
- 最终 ESI 检查补获 `0x1008` 仍沿用 SPI 名称长度的问题；已改为 `STRING(20)`、160 bit 和 HBI 名称默认字节，并加入回归检查。
- Keil 构建脚本保留个人 `.uvoptx` 和共享 RTE 头；最终二者与原始工作区无差异。当前工作区 `git diff --check` 通过。

## 仍需上板完成

按 `h743-lan9252-hbi-op-runbook.md` 核查接线和原 EEPROM 备份，生成匹配的 SII，烧录、回读并彻底断电重启。最终验收包括至少一轮脱离调试器冷启动、10 ms FreeRun OP、PDO 回环、10 分钟稳定性、3 次冷启动和拔线恢复。

CubeMX 配置已维护，但本次没有执行完整重生成。重生成后须重新检查 USER CODE、GPIO 安全状态、SRAM 模块、FMC 提前返回、MPU 和 Keil 文件组，并重新执行验证。
