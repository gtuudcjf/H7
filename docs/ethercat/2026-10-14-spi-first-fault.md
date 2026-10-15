# LAN9252 首次访问故障诊断（2026-10-14）

首次诊断改动只添加故障现场记录，不修改 SPI 模式/频率/IO 保持、超时阈值、
HAL 收发帧、故障锁存/停机策略，不增加重试、恢复或 EEPROM 写入。
随后依据实板快照、经用户批准，仅修正 HAL 毫秒超时换算，见文末。
软件回归通过不等于板上间歇性 INIT/PREOP 问题已经解决，仍须复测。

## 当前现象的解释边界

启动探测及三种 BYTE_TEST 读取成功，不代表持续服务一定成功。
当前服务遇到访问故障后停止调用 SSC MainLoop，但 MCU 的主循环仍然运行。
如果故障发生在 INIT，则可能无法进入 PREOP；若已进入 PREOP，
则可能无法响应后续退回 INIT 的请求。该过程与现象相符，
仍需下面的首次快照确认实际失败位置与分支，不能据此判定芯片损坏。

`ethercat_init_status` 是启动返回值，不是实时服务状态。
故障后 `al_status/al_status_code/al_event` 可能停止更新；
不能拿它们的旧值否定 TwinCAT 经网口读到的当前状态。
程序运行时 Keil 寄存器/黄色执行位置也可能保留上次暂停的显示，
不能仅凭运行中的 `PC=main` 判定复位。

## 复现流程

1. 保持电机不连接、功率级关闭，保持 `ETHERCAT_BRINGUP_ONLY=1`。
2. 在 Keil Rebuild，确认 0 errors，再由操作者下载到当前板。
   不需要重写 EEPROM、改 XML 或重新测 MISO。
3. 进入 Debug，在 Watch 添加下列两个全局结构（不是指针）：
   - `g_lan9252_first_fault`
   - `g_lan9252_hal_fault`
   同时保留 `ethercat_diagnostics`、`g_lan9252_spi_debug`。
4. 删除 SPI/初始化中的测试断点，按 F5，整个主站操作期间保持 MCU 连续运行。
   实时看 Watch 时启用 `View -> Periodic Window Update`。
5. 在 TwinCAT 清除错误，请求 INIT；若成功，再请求 PREOP。
   若 PREOP 成功，再请求 INIT。任一步失败即可保存现场，不必反复请求。
6. 截图保存 TwinCAT 当前状态及新错误，并完整展开两个快照。
   在主站故障发生后可只点 Stop 读取稳定快照，但注明暂停时间；
   不要先 Reset、断电、重新进入 Debug 或重新下载。
   若 Stop 时 `valid` 仍为 0，不要继续单步并把暂停引入的错误当成原始故障。

两个快照均只保存 MCU 启动后第一次相关失败，`valid=1` 最后写入。
读取时必须先确认 `valid=1`；`valid=0` 时其他字段不能解释为故障证据。
后续成功、失败、BYTE_TEST 对照和显式 `EtherCAT_Init()` 均不会清空快照。
保存后若要开始独立的下一轮，再完整重启/下载运行；不要在 Watch 中改字段清错。

## 驱动层：`g_lan9252_first_fault`

| 字段 | 含义 |
|---|---|
| `valid` | 1 表示完整首次快照已发布 |
| `address` | 失败的 LAN9252 物理 SPI 寄存器地址，例如 CSR_CMD `0x0304` |
| `esc_address` | 上层 ESC 请求的起始地址，例如 AL Event `0x0220`；直接探测为 `0xFFFFFFFF` |
| `command` | `0x0B` 快读，`0x02` 写；是物理 SPI 操作方向，不一定等于上层 ESC 读写方向 |
| `phase` | 1 命令/地址头阶段，2 数据阶段 |
| `reason` | 1 调用 transport 前预算耗尽，2 transport 返回失败，3 transport 成功但之后预算检查超时 |
| `status` | 原驱动返回码；3 DIRECT_TIMEOUT，2 IO |
| `elapsed_us` | 从物理访问开始至记录时的耗时，包含失败处理/CS 释放开销，不是单纯线上时钟时间 |
| `budget_us` | 原直接访问预算，目前 1000 μs，未放宽 |

本结构记录失败的物理直接访问，不为参数拒绝、BYTE_TEST 值不匹配、
CSR/PRAM 上层轮询超时虚构一条物理失败；那些情况仍看原诊断错误码。

## HAL 层：`g_lan9252_hal_fault`

| 字段 | 含义 |
|---|---|
| `valid` | 1 表示同次首次失败有板级 adapter 现场；驱动层独立计时超时时可为 0 |
| `reason` | 1 调用 HAL 前预算耗尽；2 HAL_TIMEOUT；3 HAL_ERROR 且 FLAG/EOT 等待失败；4 HAL 返回后的预算检查；5 其他 HAL_ERROR/HAL_BUSY |
| `hal_status` | 0 OK、1 ERROR、2 BUSY、3 TIMEOUT；`0xFFFFFFFF` 表示该分块未调用 HAL |
| `hal_error` | HAL 返回当时 ErrorCode；未调用 HAL 为 `0xFFFFFFFF`，不把旧错误当成新错误 |
| `hal_state` | 记录当时的 HAL 状态；HAL 自行清理后 READY 不代表刚才没有失败 |
| `elapsed_us` | 本次 transport 回调已经消耗的时间，不是整个 ESC 请求耗时 |
| `timeout_us/timeout_ms` | 交给 adapter 的剩余微秒预算及该分块交给 HAL 的毫秒超时参数 |
| `chunk_length` | 本次 HAL 分块字节数；调用前预算拒绝时为 0 |
| `spi_sr/cr1/cr2/cfg1/cfg2` | HAL 返回后的寄存器快照，仅读取，不读 RXDR/TXDR、不清标志 |
| `tx_remaining/rx_remaining` | HAL 返回后的剩余计数，仅对已调用 HAL 的情况有传输解释意义 |

注意：HAL 在返回前可能已经关闭传输、清除硬件标志。因此这里的 `spi_sr`
不是 HAL 内部等待失败瞬间的原始 SR；诊断时应联合 `reason/hal_status/hal_error`，
不能仅凭 SR 中没有 EOT/OVR 就断言它们从未出现。
若 `hal_status=2`（BUSY），`hal_error` 可能是上次调用留下的值，不能据此
认定本次发生了 FLAG/EOT 错误；本次会单独归入 reason=5。
如果驱动首次失败没有 HAL 快照，后续失败也不会补入一份不相关的 HAL 快照。
原有 `g_lan9252_spi_debug` 是独立的一次性 BYTE_TEST 对照，不会填入或覆盖这两个快照。

## 回传与下一步

请回传：两个结构完整截图、`ethercat_diagnostics`、
TwinCAT 失败方向/当前状态，以及本轮是否曾暂停 MCU。
HAL 的 1 ms 计时粒度与微秒预算检查不同，410 μs 就记录 DIRECT_TIMEOUT
不必然自相矛盾；需由本次快照区分 HAL 返回失败和纯微秒预算失败。

只有得到首次失败分支和地址后，再讨论针对性的修复。
不要仅凭“有时 PREOP”放宽时间、不清错重试，也不要继续改 EEPROM 掩盖现场。

## 实板快照后的 Tick 边界修正

实测首次失败：ESC 请求 `0x0004`，物理 CSR_CMD `0x0304`，快读数据阶段；
HAL 返回 TIMEOUT，ErrorCode `0x100`。HAL 回调耗时 11 μs，剩余预算 977 μs，
整次物理访问记录耗时 39 μs；传给 HAL 的超时参数为 1 ms。
此次 `ethercat_init_status=2`，失败发生在 SSC 初始化期间，尚未正常进入服务循环。

HAL 以整数毫秒 Tick 差值 `>= Timeout` 判断超时。若调用靠近 Tick 边界，
参数 1 ms 可在几微秒后到期。修正后的板级适配器使用：

```text
HAL 超时参数 = ceil(剩余微秒预算 / 1000) + 1
```

剩余预算 1～1000 μs 时交给 HAL 的参数为 2 ms。使用商和余数计算，避免
`remaining_us + 999` 在大参数下溢出回绕。驱动直接访问的 1000 μs 检查不变，
超预算完成仍返回失败；不重试、不自动清错，不修改 HAL 库文件或 SPI 配置。

这不是硬 1 ms 返回保证：HAL 故障调用可能接近 2 ms，完整访问还包含命令头、
适配器和中断开销。对通用剩余预算，额外阻塞可接近 2 ms；
这与“超预算结果不能被接受为成功”是不同的约束。

另外，HAL 超时清理会把 TxXferCount/RxXferCount 清零，因此快照中的两个零
不能证明数据已经全部成功发送/接收。

主机测试用真实适配器/协议驱动和受控 HAL 边界模型复现旧问题，覆盖全部
1000 个整数微秒 Tick 起点；也检查晚完成仍失败、分块共享预算和无重试。
该模型不能验证实际引脚时序和板上持续稳定性。

复测仍按上面的下载/连续运行步骤：先检查初始化返回 1、两份故障快照 valid=0，
再连续请求 INIT→PREOP→INIT 数次，并做独立冷启动复测。有效测试期间不要暂停 MCU。
若再次失败，优先保存两份快照及当前错误；不继续扩大超时。
