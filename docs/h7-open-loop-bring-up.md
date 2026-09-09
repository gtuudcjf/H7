# H7 开环控制上板步骤

## 编译前检查

Keil 工程 `MDK-ARM/emptytest.uvprojx` 已增加 `Application/User/Motor` 组和 `../Core/Inc/motor` 包含路径。若 CubeMX 重新生成工程，请确认这两个改动仍保留。

驱动绑定已按 H7 当前工程确认：`SPI2`（PI1/PC2/PC3）、`PC1=DRV_CS`、`PC4=DRV_ENA`、`PC5=DRV_CAL`。`DRV_CAL` 在开环阶段保持低电平；电流零偏校准移至后续电流环阶段。

若本机安装了 C99 编译器，可先在工程根目录运行：

```powershell
gcc -std=c99 -ICore/Inc/motor tests/test_open_loop.c Core/Src/motor/svpwm.c Core/Src/motor/open_loop.c -lm -o test_open_loop
.\test_open_loop.exe
```

## 未接母线时的波形确认

1. 先断开直流母线，仅给 H7 数字电源供电。
2. 下载程序后，检查 TIM8 主输出 PI5、PI6、PI7 与互补输出 PH13、PH14、PH15。
3. 应看到约 10 kHz 的中心对齐 PWM。初始 8% `Uq` 会让三相占空比随 1 Hz 电角频率缓慢变化。
4. 调用 `MotorControl_Stop()` 后，三相主/互补 PWM 应停止，`PC4` 应拉低以禁能 DRV8323；当前没有 BKIN，因此该软件停止接口是唯一 MCU 侧关断路径。

## 首次带电

1. 仅在限流电源、空载或机械固定且已确认相序的条件下进行。
2. 保持 `MotorControl_SetOpenLoopCommand(0.0f, 0.08f, 1.0f)` 的低命令开始；不要直接复制旧 F407 的固定比例或调参值。
3. 首先观察电流和温升，再逐步提高 `Uq` 与频率。此阶段没有电流闭环，堵转或过载会持续施加电压，必须由外部电源限流和驱动器保护兜底。
4. 反向只需把电角频率改为负值；若方向与期望相反，优先检查 U/V/W 相序，再确认软件命令方向。

## 后续闭环插入点

- 电流环：在 `MotorControl_FastTick()` 中，以 ADC 采样、Clarke/Park 和 Id/Iq PI 输出替换 `OpenLoop_Step()` 输出的 `MotorVoltageDq`。
- 速度环：在低频任务或快速任务分频中计算速度 PI，并更新 `Iq_ref`；它不直接写 TIM8 CCR。
- 编码器角度：用于 Park 变换和速度估算；不能在 TIM8 快速中断内使用阻塞 SPI 读取。
