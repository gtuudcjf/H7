$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
$protected=@('MDK-ARM/ethercat_openangle.uvprojx','Core/Src/main.c','Core/Src/motor/motor_control.c','Core/Inc/motor/open_angle_experiment.h','EtherCAT/motor/openangle/plc/MAIN.TcPOU')
$before=@{};foreach($path in $protected){$before[$path]=(Get-FileHash "$root/$path").Hash}
& powershell -NoProfile -ExecutionPolicy Bypass -File "$root/tools/build_ethercat_motor.ps1" -GenerateOnly
if($LASTEXITCODE -ne 0){throw 'Default build generation failed'}
[xml]$default=Get-Content -Raw -LiteralPath "$root/MDK-ARM/motor_build_dryrun.uvprojx"
$defaultDefs=[string]$default.Project.Targets.Target.TargetOption.TargetArmAds.Cads.VariousControls.Define
if(($defaultDefs -split ',') -notcontains 'ETHERCAT_MOTOR_POWER_ENABLED=0' -or $defaultDefs -match 'ETHERCAT_MOTOR_OPEN_ANGLE=1'){throw 'Default changed from locked encoder DryRun'}
foreach($variant in @('OpenAngleBenchCheck','OpenAngleDryRun')){
    & powershell -NoProfile -ExecutionPolicy Bypass -File "$root/tools/build_ethercat_motor.ps1" -Mode $variant -GenerateOnly
    if($LASTEXITCODE -ne 0){throw "Cannot generate isolated $variant project"}
    [xml]$project=Get-Content -Raw -LiteralPath "$root/MDK-ARM/motor_build_$($variant.ToLowerInvariant()).uvprojx"
    $target=$project.Project.Targets.Target
    $defs=[string]$target.TargetOption.TargetArmAds.Cads.VariousControls.Define
    $power=if($variant -eq 'OpenAngleBenchCheck'){'1'}else{'0'}
    if(($defs -split ',') -notcontains "ETHERCAT_MOTOR_POWER_ENABLED=$power"){throw 'Wrong power selection'}
    if(($defs -split ',') -notcontains 'ETHERCAT_MOTOR_OPEN_ANGLE=1'){throw 'Wrong application profile'}
    if([string]$target.TargetOption.TargetCommonOption.OutputDirectory -ne "motor_speed\$variant\"){throw 'Output overlaps another variant'}
    if([string]$target.TargetOption.TargetCommonOption.OutputName -ne "motor_build_$($variant.ToLowerInvariant())"){throw 'Wrong output name'}
    if($variant -eq 'OpenAngleBenchCheck'){
        if([string]$target.TargetOption.TargetCommonOption.CreateHexFile -ne '0'){throw 'Bench must not deliver HEX'}
        if($project.SelectNodes('//UpdateFlashBeforeDebugging[text()="1"]').Count){throw 'Bench must not auto-download on debug entry'}
        if($project.SelectNodes('//RunUserProg1[text()="1"] | //RunUserProg2[text()="1"]').Count){throw 'Unexpected build hooks'}
    }
}
foreach($path in $protected){if((Get-FileHash "$root/$path").Hash -ne $before[$path]){throw "Source unexpectedly changed: $path"}}
Write-Output 'Bench/DryRun generation: profile/power/output separation/no HEX/no auto-download/source preservation passed'
