$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
$compiler='D:/Keil_v5/ARM/ARMCLANG/bin/armclang.exe'
[xml]$project=Get-Content -LiteralPath "$root/MDK-ARM/ethercat_openangle.uvprojx" -Raw
$target=$project.Project.Targets.Target
if($target.TargetName -ne 'ethercat_openangle_dryrun'){throw 'wrong isolated target'}
$controls=$target.TargetOption.TargetArmAds.Cads.VariousControls
$argsBase=@('--target=arm-arm-none-eabi','-mcpu=cortex-m7','-E','-P')
foreach($d in ([string]$controls.Define -split ',')){if($d){$argsBase+="-D$d"}}
foreach($i in ([string]$controls.IncludePath -split ';')){
    $path=if([IO.Path]::IsPathRooted($i)){$i}else{Join-Path "$root/MDK-ARM" $i}
    $argsBase+=@('-I',[IO.Path]::GetFullPath($path))
}
$text=(& $compiler @argsBase "$root/Core/Src/main.c") -join "`n"
if($LASTEXITCODE -ne 0){throw 'openangle main preprocessing failed'}
$entry=[regex]::Match($text,'(?s)int main\(void\)\s*\{(.*?)void SystemClock_Config\(void\)').Groups[1].Value
if($entry -notmatch 'MotorControl_SetOpenAngleSpeedCommand\(0.0f\)'){throw 'missing explicit zero openangle target'}
if($entry -match 'MotorControl_Start\s*\(|BissEncoder_Init\s*\(|MX_SPI4_Init\s*\(|MX_USB_DEVICE_Init\s*\('){throw 'unexpected startup/polling peripheral'}
if($text -notmatch 'mode = MOTOR_CONTROL_OPEN_ANGLE_CURRENT'){throw 'wrong fixed motor mode'}
$motor=(& $compiler @argsBase "$root/Core/Src/motor/motor_control.c") -join "`n"
if($LASTEXITCODE -ne 0){throw 'openangle motor preprocessing failed'}
$fast=[regex]::Match($motor,'(?s)void MotorControl_FastTick\(float dt_s\)\s*\{(.*?)void MotorControl_CurrentSampleComplete').Groups[1].Value
if($fast -match 'BissEncoder_(StartRead|ControlTick)\s*\('){throw 'encoder DMA scheduling remains in fast path'}
if($controls.Define -notmatch 'ETHERCAT_MOTOR_POWER_ENABLED=0'){throw 'default power is not locked'}
Write-Output 'openangle ARM preprocessing: mode2/zero boot/no encoder scheduling/power lock passed'
