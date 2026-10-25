$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
$compiler='D:/Keil_v5/ARM/ARMCLANG/bin/armclang.exe'
[xml]$project=Get-Content -LiteralPath (Join-Path $root 'MDK-ARM/emptytest.uvprojx') -Raw
$controls=$project.Project.Targets.Target.TargetOption.TargetArmAds.Cads.VariousControls
$argsBase=@('--target=arm-arm-none-eabi','-mcpu=cortex-m7','-E','-P','-DETHERCAT_BRINGUP_ONLY=0','-DETHERCAT_MOTOR_CONTROL=1')
foreach($d in ([string]$controls.Define -split ',')){if($d -and $d -notmatch '^ETHERCAT_'){$argsBase+="-D$d"}}
foreach($i in ([string]$controls.IncludePath -split ';')){
    $path=if([IO.Path]::IsPathRooted($i)){$i}else{Join-Path "$root/MDK-ARM" $i}
    $argsBase+=@('-I',[IO.Path]::GetFullPath($path))
}
$text=(& $compiler @argsBase "$root/Core/Src/main.c") -join "`n"
if($LASTEXITCODE -ne 0){throw 'motor-mode preprocessing failed'}
$entry=[regex]::Match($text,'(?s)int main\(void\)\s*\{(.*?)void SystemClock_Config\(void\)').Groups[1].Value
foreach($call in 'EtherCAT_Init','EtherCAT_Service','MotorControl_Init','EtherCATMotorBridge_Init','EtherCATMotorBridge_Service','MotorSafety_Init','MotorSafety_Tick','HAL_ADCEx_InjectedConvCpltCallback') {
    if($text -notmatch ("\b"+$call+'\s*\(')){throw "motor-mode missing $call"}
}
if($entry -match 'MotorControl_Start\s*\(|MotorTelemetry_Init\s*\(|MX_USB_DEVICE_Init\s*\(|SetSpeedCommand\s*\(50') {throw 'motor mode auto-starts or enables unnecessary peripherals'}
# Both mode flags must be rejected by the real preprocessor.
$ErrorActionPreference='Continue'
$ErrorActionPreference='Continue'
$invalid=& $compiler @($argsBase|Where-Object{$_ -ne '-DETHERCAT_BRINGUP_ONLY=0'}) -DETHERCAT_BRINGUP_ONLY=1 "$root/Core/Src/main.c" 2>&1
$ErrorActionPreference='Stop'
$ErrorActionPreference='Stop'
if($LASTEXITCODE -eq 0){throw 'conflicting modes were accepted'}
Write-Output 'motor-mode ARM preprocessing and startup isolation passed'
