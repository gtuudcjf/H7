param([ValidateSet('DryRun','Communication','LocalMotor','PowerPathCheck','OpenAngleDryRun','OpenAngleBenchCheck')][string]$Mode='DryRun',[switch]$GenerateOnly)
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
$keil='D:/Keil_v5/UV4/UV4.exe'
if(-not(Test-Path -LiteralPath $keil)){throw 'Keil UV4.exe unavailable'}
$template=if($Mode -in 'OpenAngleDryRun','OpenAngleBenchCheck'){"$root/MDK-ARM/ethercat_openangle.uvprojx"}else{"$root/MDK-ARM/ethercat_motor.uvprojx"}
[xml]$p=Get-Content -LiteralPath $template -Raw
$t=$p.Project.Targets.Target
$templateName=[string]$t.TargetName
$name='motor_build_'+$Mode.ToLowerInvariant()
$defines='USE_PWR_LDO_SUPPLY,USE_HAL_DRIVER,STM32H743xx,'
switch($Mode){
 'DryRun' {$defines+='ETHERCAT_BRINGUP_ONLY=0,ETHERCAT_MOTOR_CONTROL=1,ETHERCAT_MOTOR_POWER_ENABLED=0'}
 'Communication' {$defines+='ETHERCAT_BRINGUP_ONLY=1,ETHERCAT_MOTOR_CONTROL=0,ETHERCAT_MOTOR_POWER_ENABLED=0'}
 'LocalMotor' {$defines+='ETHERCAT_BRINGUP_ONLY=0,ETHERCAT_MOTOR_CONTROL=0,ETHERCAT_MOTOR_POWER_ENABLED=0'}
 'PowerPathCheck' {$defines+='ETHERCAT_BRINGUP_ONLY=0,ETHERCAT_MOTOR_CONTROL=1,ETHERCAT_MOTOR_POWER_ENABLED=1'}
 'OpenAngleDryRun' {$defines+='ETHERCAT_BRINGUP_ONLY=0,ETHERCAT_MOTOR_CONTROL=1,ETHERCAT_MOTOR_OPEN_ANGLE=1,ETHERCAT_MOTOR_POWER_ENABLED=0'}
 'OpenAngleBenchCheck' {$defines+='ETHERCAT_BRINGUP_ONLY=0,ETHERCAT_MOTOR_CONTROL=1,ETHERCAT_MOTOR_OPEN_ANGLE=1,ETHERCAT_MOTOR_POWER_ENABLED=1'}
}
$t.TargetName=$name
foreach($targetInfo in $p.SelectNodes('//targetInfo')){$targetInfo.SetAttribute('name',$name)}
$t.TargetOption.TargetArmAds.Cads.VariousControls.Define=$defines
$t.TargetOption.TargetCommonOption.OutputDirectory="motor_speed/$Mode/"
$t.TargetOption.TargetCommonOption.OutputName=$name
# Non-dry-run variants are compile/link checks, not downloadable HEX delivery.
if($Mode -notin 'DryRun','OpenAngleDryRun'){$t.TargetOption.TargetCommonOption.CreateHexFile='0'}
$generated="$root/MDK-ARM/$name.uvprojx"
$log="$root/MDK-ARM/$name.log"
$raw=Get-Content -LiteralPath $template -Raw -Encoding UTF8
$raw=$raw.Replace($templateName,$name)
$raw=[regex]::Replace($raw,'<Define>[^<]*</Define>',"<Define>$defines</Define>")
$raw=[regex]::Replace($raw,'<OutputDirectory>[^<]*</OutputDirectory>',"<OutputDirectory>motor_speed\$Mode\</OutputDirectory>")
if($Mode -notin 'DryRun','OpenAngleDryRun'){$raw=$raw.Replace('<CreateHexFile>1</CreateHexFile>','<CreateHexFile>0</CreateHexFile>')}
if($Mode -eq 'OpenAngleBenchCheck'){
    $raw=$raw.Replace('<UpdateFlashBeforeDebugging>1</UpdateFlashBeforeDebugging>','<UpdateFlashBeforeDebugging>0</UpdateFlashBeforeDebugging>')
    Write-Warning 'BENCH POWER PERMISSION=1: offline build only. AXF is flashable; no download or energized test is authorized by this script.'
}
[IO.File]::WriteAllText($generated,$raw,(New-Object System.Text.UTF8Encoding($false))) # build artifact
if($GenerateOnly){Write-Output "Generated $generated (no compiler/debugger/download launched).";return}
$process=Start-Process -FilePath $keil -ArgumentList @('-r',"`"$generated`"",'-t',$name,'-j0','-o',"`"$log`"") -WindowStyle Hidden -PassThru -Wait
if(-not(Test-Path -LiteralPath $log)){throw "Keil exited $($process.ExitCode) before producing build log: $log"}
Get-Content -LiteralPath $log -Tail 12
if($process.ExitCode -gt 1){throw "Keil build exit $($process.ExitCode)"}
$text=Get-Content -LiteralPath $log -Raw
if($text -notmatch '0 Error\(s\), 0 Warning\(s\)'){throw 'Build must have zero errors and warnings'}
Write-Output "ARM build $Mode passed (build only; no flash/download)."
