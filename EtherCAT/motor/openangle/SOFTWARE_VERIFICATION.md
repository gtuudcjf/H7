# 软件验证记录（2026-10-20，最终修正2026-10-21）

范围：新固定模式2、默认功率锁0；只做宿主测试、预处理、离线编译。没有烧录、
EEPROM写入、TwinCAT激活/登录/运行操作，也没有电流/机械或保护延迟实测。

## 可复现命令与结果

原runner需将本机TinyCC目录 `tmp/commit_verification_20261015/tool/tcc` 加入当前进程PATH。
首次未设置PATH的重跑发生工具缺失错误；设置后完整重跑通过，不是忽略失败。

| 命令（PowerShell -File） | 结果 |
| --- | --- |
| tests/run_ethercat_motor_tests.ps1 | 原core114、guard19、safety两组各21、bridge4/22、dictionary47及原电机清理/策略通过 |
| tests/run_ethercat_openangle_tests.ps1 | core32、policy32、guard7、真实bridge/core/guard两种功率宿主配置通过；生产反馈快照15场景通过 |
| tests/run_ethercat_suite.ps1 | 0 failed / 0 pending |
| tests/run_motor_telemetry_suite.ps1 | 原电机/编码器遥测8组通过 |
| tests/test_ethercat_motor_build.ps1 | 原模式ARM预处理与启动隔离通过 |
| tests/test_openangle_build.ps1 | 模式2、零启动、无编码器调度、功率锁ARM预处理通过 |

`python -m unittest discover -s tests -p 'test*artifacts.py'`：8 tests OK。
反馈额外回归先观察到“ADC和TIM8同时停下后旧valid仍有效”的失败，增加仅由成功ADC
更新的时间戳后通过：超过2ms的反馈无效，同时保留原3个PWM tick检查。
宿主假外围寄存器不等于实际ADC/PWM验证；PLC构件测试也不是完整ST动态仿真。

最终只读独立审查发现两项Important，均先失败再修正通过：

- 本地保护撤销后 Receive 先于 Service 时，12/13/14被误报4。新版Receive先取本地原因再解释guard锁存。
- ALIGNING时ADC/TIM8同时停止，旧快照无效但STARTING仍等2s。现在已建立三相控制后
  的无效快照报告故障8，由桥接关断；正常初始零偏校准不被误判。

第二项由生产快照测试与真实桥接/状态机/guard的STARTING测试组合覆盖，尚不是实际ADC冻结波形。
审查留下一项Minor测试覆盖缺口：未补5000ms延迟启动后恢复使能的组合场景，以及PLC
控制字/停止/反向规则的静态自动断言；已有纯guard/100ms组合测试与人工代码审查不能等同于这两项。
原runner在新沙箱下曾遇到TinyCC系统DLL重定位错误；经批准在沙箱外离线运行后完成验证。

## ARMCC5实际链接

`tools/build_ethercat_motor.ps1 -Mode <模式>`：Communication、LocalMotor、DryRun、
PowerPathCheck、OpenAngleDryRun五组均0 Error(s),0 Warning(s)。最新OpenAngleDryRun
含上述ADC时间戳修正。日志在 `MDK-ARM/motor_build_<小写模式>.log`。

新固件：`MDK-ARM/motor_speed/OpenAngleDryRun/motor_build_openangledryrun.axf`及同名hex。
最终修正后HEX SHA256：`C5039E117C4BF4BDF5C715A0677A1C080687171D8216D01E3BB3A8C41EAB4BA1`。
独立手工工程：`MDK-ARM/ethercat_openangle.uvprojx`。这些都是功率锁0的新模式交付；
PowerPathCheck只是旧速度功率分支编译检查，不产生可下载HEX，不代表功率路径已验收。

## TwinCAT离线验证

4024.78隔离临时工程编译并导出，归档再导入第二临时工程编译通过：

- `tmp/plc_verify_e0e994e01cd740ba9e23fdd85ff0d946`
- `tmp/plc_verify_fd44c3bedab54197ac73f9714c19564e`

实际导出任务周期10000us。交付 `plc/EtherCATMotorOpenAngleDemo.tpzip` 的SHA256：
`4D8335F584AF6E439C052184B4C4A1BFC88F7D9A5FB928BC38D7D4DEF548939C`。
用户活动PLC未被替换，现场仍需操作者在工程副本中按COMMISSIONING执行。

## 保留与未完成项

修改前备份：`tmp/openangle_baseline_20261020_150331`，166文件SHA256核对。
复查12个旧motor构件/工程文件、系统ESI及用户活动tsproj与清单一致；8/20字节PDO
和13个GVL链接契约保持。共用电机/桥接源码有隔离条件修改，不是完全不碰旧源码。

尚未实测：本板电流零偏/极性、固定磁场对齐、正反转、5s/3s独立时限和失联
<=110ms波形。ActualSpeed固定0且测量有效位为0，不能据此判断机械停稳。
任何功率解锁及带电测试仍需单独批准。

最终审查及裁决记录：`.superpowers/sdd/2026-10-20-ethercat-open-angle-current/progress.md`。

## 2026-10-21：BenchCheck最小构建扩展

用户批准仅准备受限实验构建及B清单，并要求不重复繁琐备份。此次没有新增备份目录、
没有改控制C源码/PLC/模板工程/ESI/PDO。仅修改build runner、添加一个配置测试，更新现有两份说明。

- 入口：`tools/build_ethercat_motor.ps1 -Mode OpenAngleBenchCheck`；默认模式不变。
- 新生成项目使用原模式2模板；独立目录、POWER=1，不生成HEX，禁用调试入口自动下载。
  AXF仍然可手工烧录并允许功率，编译不是实机实验批准。
- `tests/test_openangle_bench_build.ps1`：先RED（缺失新Mode），后GREEN；实际执行
  GenerateOnly并核对新/旧宏、输出隔离、默认旧速度DryRun仍锁功率、无HEX/自动下载/构建钩子，
  模板/main/电机控制/候选参数/PLC文件前后hash一致。测试不连接硬件。
- ARMCC5：`MDK-ARM/motor_build_openanglebenchcheck.log`，0 Error(s),0 Warning(s)。
- AXF：`MDK-ARM/motor_speed/OpenAngleBenchCheck/motor_build_openanglebenchcheck.axf`。
  SHA256：`BE6C1F516EDAA569597084C5225CAF5BD50B75FE7B3E5717B3CC7E0DAD441E3C`。
- 新模式宿主core32/policy32/guard7/bridge两功率配置/生产快照15场景通过；旧电机回归通过；
  原DryRun ARM预处理通过；EtherCAT 0 failed/0 pending；telemetry8组及Python构件8测试通过。
  TinyCC沙箱内DLL重定位错误经批准在沙箱外重跑通过；未用修改测试来绕过。
- 本次只编译，没有下载、启动调试器、在线PLC写入或动力操作。B阶段真实时序/ADC/电流
  验收仍待确认接线、测量条件和实机批准；5s独立保护注入方案未在此增加，不能以零命令
  自动停机或宿主结果替代其波形验收。

## 2026-10-21：单次启动电流诊断（仅RAM记录）

用户批准在现有代码上做最小诊断扩展，不新增备份或工程版本。本次没有修改PLC、PDO、
ESI、电机参数、保护阈值或构建模板。仅在OPEN_ANGLE路径接入216字节RAM记录，
包含零偏、首个/首次超限/峰值/最后有效样本、超限计数与首次终止分类。
外部Stop API分类不细分通信/PLC原因；复位保留诊断，下一次Start覆盖，断电丢失。

- 策略测试从32项增至51项，覆盖最大值、非连续与连续超限、严格0.6A边界、
  Stop后的冻结、复位清理不覆盖、下一次启动清空、时间戳回绕。
  首轮因新增诊断类型/API不存在而编译失败，补实现后51项通过。
- 生产测量/故障函数集成测试使用真实CurrentSense和Clarke/Park，确认非连续两次超限
  不触发、连续第三次触发原fault7、先执行原安全撤销再复制触发样本、终止记录不被覆盖。
  假外围仅替代关断及无关控制器清理；宿主CRT浮点数学接口通过真实double数学函数适配。
- 新模式core32/policy51/guard7、两种功率bridge、反馈快照15场景通过；旧模式
  Stop/ClearFault、runtime policy、core114、guard19、safety两组21、bridge4/22、dictionary47通过。
- EtherCAT suite：0 failed / 0 pending；telemetry suite：8组通过；Python构件8 tests OK。
- BenchCheck及原速度DryRun ARMCC离线编译均0 Error(s),0 Warning(s)，
  日志仍使用原同名log，BenchCheck输出仍在原OpenAngleBenchCheck目录。
  本次BenchCheck AXF SHA256：`C3B58CA75AEA5AD853BD2590B1483054187226CDBFC8B2743A5E2357698774DB`。
- 测试工具问题单独处理：TinyCC沙箱DLL重定位失败，经批准在沙箱外离线运行；
  tmp下宿主math.h的x87内联兼容问题改用CRT fabs，不改固件数学实现。
  Keil沙箱内无法启动ARMCC/链接器，经批准在沙箱外完成编译。

本次没有烧录、进入Debug、写PLC或上电实验。实际采样异常原因尚未确定；新增记录的
中断开销与硬件保护延迟未实测，不能把软件测试或无故障画面当作硬件验收。

## 2026-10-21：ADC 双通道序列完成中断修正

用户确认后，仅将公共 `Core/Src/adc.c` 的 `EOCSelection` 从
`ADC_EOC_SINGLE_CONV` 改为 `ADC_EOC_SEQ_CONV`，使当前 HAL 的注入中断选择 JEOS，
与回调一起读取 JDR1/JDR2 的契约一致。采样通道、触发点、PWM、控制器、保护阈值、
诊断记录和 PLC/PDO 未修改；未增加备份、工程版本或寄存器读回功能。

- 在现有 `tests/test_h7_current_config.ps1` 中接入 `tests/test_h7_adc_init.c`：
  执行提取的真实 `MX_ADC1_Init`，仅替代硬件 HAL 边界，检查实际提交的完整序列配置。
  先在旧配置观察到 EOCSelection 断言失败，再修改配置并确认通过。
  宿主编译沿用本机 `tmp/commit_verification_20261015/tool/tcc/tcc.exe`；干净环境需先准备该工具。
- 新/旧模式、EtherCAT、遥测、ARM预处理及Python构件回归通过。
- BenchCheck和旧速度DryRun离线编译均0 Error(s),0 Warning(s)，仍使用原目录的同名输出。
  只读审查未发现关键或重要问题；上述宿主编译器路径属于已知可移植性限制。
  SHA256：`5C69349BA690F795A747FEC99200CD8EF62DCA7CA4949F8B139143E9FB148FA1`。
- CubeMX重新生成公共ADC初始化时，应重跑此测试，防止恢复为单通道完成中断。

本次没有下载、调试器控制或上电实验。测试验证配置契约，不模拟或证明实际ADC时序；
旧录屏中的13500次记录不能据此追认为完整同周期样本。电流尖峰是否改善仍需无电机对照实测，
不能宣称异常已解决，也不提高0.6A阈值或绕过保护。

## 2026-10-21：DRV8323 启动寄存器 RAM 读回

用户批准后，仅在板级配置包装层的 OPEN_ANGLE 路径增加68字节
`g_drv8323_startup_diagnostics`，读取0x00..0x06并保存原始值、逐项HAL状态及掩码。
位置在原配置/清故障之后、CAL/ADC/PWM启动之前；首个读取失败停止、不重试、不修复配置。
原配置结果保留，新增读回不作为控制判据；原独立安全许可在每次读取前及全部读取后复核，
许可撤销保持驱动关闭并返回错误。Stop不读SPI、不覆盖记录，下一次配置调用覆盖并增加session。
旧模式不包含该全局量和新增读取；JEOS修正、增益/阈值、PWM、PLC/PDO保持不变。

- 测试先因新全局量未实现而编译失败；实现后真实板级包装层与真实DRV SPI帧代码测试通过。
  仅替代HAL硬件边界及许可源，覆盖7项原始读回、上位无效位屏蔽、CLR_FLT自清读回、
  配置失败、读取超时停止/无重试、值不符不重写、Stop冻结、下一次清空、读中/末次读许可撤销。
  OPEN_ANGLE=0用同一生产源码确认仍为原三次写入、无新增读操作。
- 新/旧模式回归、EtherCAT 0 failed/0 pending、遥测8组、ARM预处理及Python构件8项通过。
- BenchCheck与旧速度DryRun离线编译均0 Error(s),0 Warning(s)；只读审查无阻塞问题。
  BenchCheck map确认新记录68字节；旧DryRun map不包含该符号。
  当前BenchCheck AXF SHA256：`B3DD581C707DEC681524E240460D7BD2FCB880A8CEEC2C6D210CB2AE012B078F`。

读取来自启动前且已清故障的时刻，不代表之后运行中没有故障；HAL_OK/valid_mask只证明
传输返回成功，不证明实际芯片接受了配置。解释和Watch步骤见COMMISSIONING的B5。
没有新增备份/工程版本，没有下载、操作调试器、写PLC或进行带电实验；新增前台SPI耗时、
实际读回与电流采样异常仍待实测，不能宣称故障已修复。

## 2026-10-22：一次启动的100ms无相PWM采样对照

用户批准最小诊断改动并明确最终目标为电机正反转、不做过度实机测试。
沿用同一BenchCheck工程/AXF，不建备份、版本、分支或工作树。只在OPEN_ANGLE路径中，
成功CAL校准拉低CAL后保持READY约100ms，TIM8 CH4原注入触发继续，六路相PWM尚不使能；
随后进入原对齐/零需求自动停止。新增216字节RAM记录
`g_motor_openangle_baseline_diagnostics`，复用原结构/换算/保护并与后续样本分开。
保护连续计数跨段保留，不放宽阈值、时限或故障；不改ADC采样时刻、增益、PWM配置、
PLC/PDO、编码器或原非OPEN_ANGLE流程。操作与解释见COMMISSIONING B6。

- TDD首先对真实FinishCalibration观察到断言失败：旧流程立即调用相PWM模式入口。
  新生产函数测试覆盖100ms前不进入、100ms时只进入一次、1000样本段落隔离、
  原0.6A第三个连续样本故障（包括跨段第三个）、范围/数学/采样超时、许可撤销、
  相输出位异常、Stop/ClearFault冻结和毫秒回绕；真实CurrentSense与Clarke/Park参与。
- 只读审查和父进程均识别并修复直接Stop与ADC基线完成之间的竞态：两基线helper必须
  同时满足READY、STOPPED模式且sampling_started；故Stop先改变状态后，即使ADC中断
  插入HAL清理也不能重开相PWM。测试先模拟该情形并失败，再加入检查并通过。
  采样新鲜度在基线激活前初始化，等待段保留快速中断/前台两条过期检查。
- 新模式回归、旧电机模式回归、EtherCAT套件0 failed/0 pending、遥测8组、
  ARM预处理/构建隔离、Python构件8项通过；只读审查无剩余关键/重要问题。
- 最终BenchCheck ARMCC离线编译0 Error(s),0 Warning(s)，Code=71560、RO=2304、
  RW=1344、ZI=9976；map确认新增记录216字节。当前AXF SHA256：
  `276A672DC113668DAA1A776B17D05B1CE39BA2AFA39E94947A7BE2315BD45EA4`。
  第一次沙箱内编译进程滞留，已仅清理核实属于本次任务的隐藏Keil进程，沙箱外最终编译完成；
  用户原Keil窗口保留，没有下载、Debug控制、PLC写入或带电测试。

该修改提供一次无电机对照，不证明模拟采样、实际相PWM关闭电平、电流精度或实时执行裕量
已经验收，更不直接批准接电机。对照结果用于选择必要修正，避免重复同样实验；
短窗未发现异常也不能排除更长时间或不同PWM状态下的异常。

## 2026-10-23：单变量ADC分频对照准备（不是已确认修复）

用户要求按最小改动尽快推进下一次测试。已利用旧模式2代码和最新录屏缩小范围，
仅在现有BenchCheck（OPEN_ANGLE=1且POWER_ENABLED=1）的MX_ADC1_Init中把DIV1改为DIV2；
其他模式仍为DIV1。未修改TIM8触发频率/时刻、通道、采样周期枚举、JEOS、控制器、
DRV寄存器、增益/阈值、诊断结构、PLC/PDO或时限，没有新增工程、备份或诊断版本。
此为ADC时序敏感性假设实验，不能称为根因修复或推断原ADC超频。

- 扩展原真实ADC初始化测试：先观察Bench分频断言失败；修改生产分支后，Bench、
  OpenAngle DryRun、旧编码器DryRun、PowerPath及Local五种配置全部通过。
  同时检查原12bit、32.5采样周期、两rank、触发源/边沿与JEOS契约不变。
- 新模式全部回归（含真实基线/保护/停止竞态）、旧电机模式回归通过；EtherCAT套件
  0 failed/0 pending、遥测8组、ARM模式预处理及构建隔离、Python构件8项全部通过。
  最初直接执行PS脚本被系统执行策略拒绝，改用既有-NoProfile/-ExecutionPolicy Bypass
  子进程后完整重跑通过，没有忽略失败；未修改系统执行策略。
- 同名BenchCheck ARMCC离线编译完成：0 Error(s),0 Warning(s)，Code=71564、
  RO=2304、RW=1344、ZI=9976，AXF SHA256：
  `40534D98864F9B1207F80E07343F0D6B14C76594E340F1341C6F7C1C557A5A94`。
  当前HAL的DIV2符号值为0x00040000，可在重新下载/连续Run后只读检查。

编译及测试未操作调试器、下载板卡、写PLC或进行供电/机械实验。没有新增RAM记录，
正常控制仍由原10kHzTIM8触发，但ADC转换完成和控制更新的实际延迟会改变；软件测试
不证明模拟采样改善或PWM有效采样窗口，结论依赖一次无电机零需求硬件对照，步骤见B7。

## 2026-10-23：用户选择暂缓采样排查，恢复旧过流阈值

用户明确要求不再进行采样专项硬件测试，记录问题待后续讨论，优先实现手动正反转。
最小生产改动为OAE_PHASE_TRIP_A复用原MOTOR_CURRENT_TRIP_A（10.0A），保留原
MotorControl_UpdateCurrentMeasurement的严格>10A连续2样本关断，未禁用过流检查。
OAE原3样本检查仍存在但门槛相同，正常被原两样本保护先截获。诊断观察门槛单独命名为
OAE_DIAGNOSTIC_PHASE_A=0.6A，保持旧统计可比较，不参与停机判定。

未增加电流给定（仍0.1A）、电压/电流参考限幅或会话/运动/停止时限；未修改PLC/PDO、
心跳/反馈新鲜度、ADC范围/数学保护、停止/换向逻辑。保留上次Bench DIV2设置，
未为本次操作重新调整采样参数；无新增版本、备份、工程或RAM结构。

- TDD先在恢复后的期望下观察到失败：约2.2A读数第三次仍被0.6A旧门槛截获。
  更改门槛后OAE策略71项通过；真实换算/故障测试确认中等读数连续出现不关断而旧
  0.6A计数继续记录，>10A第二次关断、先撤销再记录、冻结历史保持。
  基线测试覆盖>10A跨基线/开启PWM的两样本计数连续性，以及基线内两次即关断。
- 新模式全部回归、旧电机模式、EtherCAT套件0 failed/0 pending、遥测8组、
  ARM模式预处理/工程隔离及Python构件8项通过；只读审查未发现本次差异的关键/重要问题。
- 同名BenchCheck ARMCC构建日志完成且0 Error(s),0 Warning(s)，Code=71564、
  RO=2304、RW=1344、ZI=9976。AXF SHA256：
  `86323AC4754B7BE79B9BDDDFC708839F15664EBFAD89EAF19411719E9563E256`。
  链接完成后隐藏Keil未退出，批准后核对完整命令行，仅清理本次构建PID26412；
  用户原Keil PID19644保留。构建通过依据为完成日志/当前AXF和宿主回归，未将清理退出码
  当作正常编译器退出证据。未下载、调试器运行/暂停、PLC写入或进行带电实验。

10A只是用户选择恢复的旧软件配置，并非本次核定的电机/功率级安全额定值，
也不保证不可信采样一定能检测真实过流。DIV2数据与遗留问题已记录COMMISSIONING待办；
提高阈值不是采样修复，正反转尚未实机验证。当前短时手动正转/停止/反转操作见C，
需固定空载电机、正确释放外部抱闸，不在运动中反向、不自行提高供电限流或命令电流。

## 2026-10-23：用户批准对齐/运行给定0.1A改为0.3A

用户确认抱闸已释放，提出起动力不足假设并明确批准0.3A方案。唯一生产改动为
`Core/Inc/motor/open_angle_experiment.h:5` 的 `OAE_CURRENT_A` 从0.1f改为0.3f，
同时供真实对齐Id目标和OAE_Step运行Iq目标使用。0.3A参考矢量限幅、0.15pu电压限幅、
>10A连续2样本关断、3s运动/5s会话上限、停止/反向/通信保护和ADC设置均未改。

- 策略测试先因真实OAE_Step的对齐输出仍为0.1A而失败；修改后73项通过，覆盖0.3A对齐、
  正反向运行和减速阶段保持给定/最终清零，检查对齐目标不超过现有参考矢量限幅。
- 新模式完整回归、旧电机模式、EtherCAT套件0 failed/0 pending、遥测8组、ARM预处理与
  构建隔离、Python构件8项及git diff --check通过。
- 原同名BenchCheck ARMCC离线构建正常退出，0 Error(s),0 Warning(s)，
  Code=71564、RO=2304、RW=1344、ZI=9976。AXF SHA256：
  `369A0F4D4A8A02EF4879942F00FE47687709BC77D3CB90B9CF34E99AC03F6983`。

未新增备份/工程版本，未下载、操作调试器、写PLC或上电试转。此修改只验证起动力不足
假设，不证明实际电流跟踪、起转成功或采样异常修复；新AXF需操作者重新下载才会生效。

## 2026-10-23：用户批准拆分旧模式2对齐/运行设定点

修改前用户已将OAE_CURRENT_A设为0.8A、OAE_CURRENT_LIMIT_A设为1.0A。本次保留这两项，
仅新增OAE_ALIGN_CURRENT_A=0.5A，并将OAE_Step对齐输出和RunCurrentLoop实际对齐目标
改用新宏；运行Iq仍为0.8A。未修改PLC、ADC、PI、频率斜坡、电压限幅、过流保护和时限。
1Hz不硬编码：操作者在启动前写rDemandRpm=+6.0；按10对极换算为+1Hz。

- TDD先更新行为期望，观察真实OAE_Step仍输出0.8A对齐而非0.5A的失败；修改后策略78项通过。
  覆盖独立对齐幅值、转入0.8A运行、±6rpm到±1Hz换算、停止、反向和时限保护。
- 新模式完整回归、旧电机模式、EtherCAT套件0 failed/0 pending、遥测8组、ARM预处理与
  工程隔离、Python构件8项通过；git diff --check无空白错误（现有文件有Git换行提示）。
- 同名BenchCheck ARMCC离线构建正常退出：0 Error(s),0 Warning(s)，
  Code=71548、RO=2304、RW=1344、ZI=9976。AXF SHA256：
  `5FAA8BD5FD7F04BB0B910D7F2BBBE4E7AF5CC0796516854A752BC9D972D278BA`。

未创建备份或新工程版本，未下载固件、操作调试器或写入TwinCAT。此结果仅为离线验证，
不能证明实际转子已跟随、正反转成功或当前电流配置安全；最新操作见COMMISSIONING的C节。

## 2026-10-23：6/12rpm可转后适度延长观察窗

用户报告6和12rpm均能转动，但时间短、平滑度和可闻噪声仍有问题；截图MotorFault=0、
BridgeFault=12。本轮仅将OpenAngle运动上限从3秒延长到6秒、总会话从5秒延长到10秒。
Core/Inc/motor/open_angle_experiment.h集中定义时限；motor_command_guard.c在OPEN_ANGLE
分支共用这组宏，避免策略延长而独立保护仍提前关断，非OpenAngle分支保留原5秒/3秒。
运动计时含加减速，不承诺恒速保持6秒，也不表示持续运行安全性已验证。

- TDD确认原策略在3010ms提前关断、原独立guard在新会话边界前提前关断。更新后策略80项、
  独立guard双profile各7场景通过；覆盖新边界、原边界继续许可、回绕、首次计时不续期。
  桥回归验证3040ms输出仍许可，6040ms独立撤销输出并保留故障12。
- 新模式及旧电机/EtherCAT/遥测完整回归、ARM预处理/构建隔离、Python8项、diff检查通过。
  只读窄审查未发现本轮critical/important/minor问题。测试断言已改为控制台失败，避免Windows
  CRT断言弹窗阻塞；首次RED进程以失败退出，未清理任何用户调试器进程。
- 同名BenchCheck ARMCC构建正常退出，0 Error(s),0 Warning(s)，Code=71548、RO=2304、
  RW=1344、ZI=9976。AXF SHA256：
  `4E0D78E77D2F78F62237C8EB8C6D86AF372EBE1E24B914D2BD6C28221AC4417C`。

未改电流给定/限幅、PI、ADC、PWM频率、100ms心跳、2秒停止保护或PLC；未下载、上电或试转。
尚未验证噪声来源或匀速性；虚拟角度控制不能证明转子同步，10kHz PWM也可能产生可闻激励。
下一轮按COMMISSIONING C节先观察约4秒并主动停止；持续抖动、异响或异常发热立即停机。
