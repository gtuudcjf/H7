$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
$out=Join-Path $root 'tmp/openangle_tests'
New-Item -ItemType Directory -Path $out -Force | Out-Null
$source=Get-Content -Encoding UTF8 -LiteralPath "$root/Core/Src/motor/motor_control.c" -Raw
$start=$source.IndexOf('void MotorControl_GetOpenAngleFeedback(')
if($start -lt 0){throw 'Missing production mode-2 coherent feedback adapter'}
$end=$source.IndexOf("`n#endif",$start)
if($end -lt 0){throw 'Missing mode-2 adapter conditional boundary'}
[IO.File]::WriteAllText((Join-Path $out 'openangle_feedback_under_test.inc'),$source.Substring($start,$end-$start))
$cc=Join-Path $root 'tmp/commit_verification_20261015/tool/tcc/tcc.exe'
& $cc -Wall -Werror -I $out "$root/tests/test_openangle_motor_adapter.c" -o (Join-Path $out 'adapter.exe')
if($LASTEXITCODE -ne 0){throw 'production feedback compile failed'}
& (Join-Path $out 'adapter.exe');if($LASTEXITCODE -ne 0){throw 'production feedback behavior failed'}

# Exercise the real current conversion/protection/diagnostic ordering, not a
# source-text assertion. Hardware shutdown alone is replaced in the C harness.
$recordStart=$source.IndexOf('static void MotorControl_RecordCurrentDiagnostics(')
$recordEnd=$source.IndexOf("`n#endif",$recordStart)
$faultStart=$source.IndexOf('static void MotorControl_EnterFault(')
$faultEnd=$source.IndexOf('static HAL_StatusTypeDef MotorControl_EnablePwm(',$faultStart)
$currentStart=$source.IndexOf('static bool MotorControl_UpdateCurrentMeasurement(')
$currentEnd=$source.IndexOf('static bool MotorControl_UpdateEncoderAngle(',$currentStart)
$baselineStart=$source.IndexOf('static bool MotorControl_RunPwmOffBaseline(')
$baselineBlockStart=if($baselineStart -ge 0){$source.LastIndexOf('#if ETHERCAT_MOTOR_OPEN_ANGLE',$baselineStart)}else{-1}
if($baselineBlockStart -ge 0){$currentEnd=$baselineBlockStart}
if($recordStart -lt 0 -or $recordEnd -le $recordStart -or $faultStart -lt 0 -or $faultEnd -le $faultStart -or $currentStart -lt 0 -or $currentEnd -le $currentStart){throw 'Missing production current/fault functions'}
$currentSource=$source.Substring($recordStart,$recordEnd-$recordStart)+"`n"+$source.Substring($faultStart,$faultEnd-$faultStart)+"`n"+$source.Substring($currentStart,$currentEnd-$currentStart)
[IO.File]::WriteAllText((Join-Path $out 'openangle_current_under_test.inc'),$currentSource)
& $cc -Wall -Werror -I $out -I "$root/Core/Inc/motor" "$root/tests/test_openangle_current_diagnostics.c" "$root/Core/Src/motor/open_angle_experiment.c" "$root/Core/Src/motor/current_sense.c" "$root/Core/Src/motor/foc_transform.c" -o (Join-Path $out 'current_diagnostics.exe')
if($LASTEXITCODE -ne 0){throw 'production current diagnostic compile failed'}
& (Join-Path $out 'current_diagnostics.exe');if($LASTEXITCODE -ne 0){throw 'production current diagnostic behavior failed'}

# Run the real calibration/baseline entry and its protection/cleanup consumers.
$finishStart=$source.IndexOf('static void MotorControl_FinishCalibration(')
$baselineEnd=$source.IndexOf('static bool MotorControl_UpdateEncoderAngle(',$currentStart)
$stopStart=$source.IndexOf('HAL_StatusTypeDef MotorControl_Stop(void)')
$stopEnd=$source.IndexOf('void MotorControl_GetFeedback(',$stopStart)
$hasBaseline=if($baselineStart -ge 0){1}else{0}
if(-not $hasBaseline){throw 'Missing production PWM-off baseline entry'}
$baselineSource="#define HAVE_BASELINE_HELPER $hasBaseline`n"+$currentSource+"`n"+$source.Substring($finishStart,$currentStart-$finishStart)
if($hasBaseline){$baselineSource+="`n"+$source.Substring($baselineBlockStart,$baselineEnd-$baselineBlockStart)}
$baselineSource+="`n"+$source.Substring($stopStart,$stopEnd-$stopStart)
[IO.File]::WriteAllText((Join-Path $out 'openangle_baseline_under_test.inc'),$baselineSource)
& $cc -Wall -Werror -I $out -I "$root/Core/Inc/motor" "$root/tests/test_openangle_baseline.c" "$root/Core/Src/motor/open_angle_experiment.c" "$root/Core/Src/motor/current_sense.c" "$root/Core/Src/motor/foc_transform.c" "$root/Core/Src/motor/motor_runtime_policy.c" -o (Join-Path $out 'baseline.exe')
if($LASTEXITCODE -ne 0){throw 'production PWM-off baseline compile failed'}
& (Join-Path $out 'baseline.exe');if($LASTEXITCODE -ne 0){throw 'production PWM-off baseline behavior failed'}
