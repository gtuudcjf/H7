param([string]$ArchiveToVerify='', [string]$PlcSourceDir='', [ValidatePattern('^[A-Za-z][A-Za-z0-9_]*$')][string]$ProjectName='MotorSpeedDemo')
# Isolated, offline XAE compilation only. Never ActivateConfiguration/Restart/Login/Start.
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
$out=Join-Path $root ('tmp/plc_verify_'+[guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $out -Force | Out-Null
if(-not $PlcSourceDir){$PlcSourceDir="$root/EtherCAT/motor/plc"}
Copy-Item -LiteralPath (Resolve-Path -LiteralPath $PlcSourceDir).Path -Destination "$out/plc" -Recurse
$archiveName=if($ProjectName -eq 'MotorSpeedDemo'){'EtherCATMotorSpeedDemo'}else{$ProjectName}
$dte=$null
function Invoke-ComRetry([scriptblock]$Operation) {
    for($attempt=0; $attempt -lt 80; $attempt++) {
        try { return (& $Operation) }
        catch {
            if(($_.Exception.ToString() -notmatch '80010001|8001010A|rejected by callee|被呼叫方拒绝|XAE_NOT_READY') -or $attempt -eq 79){throw}
            Start-Sleep -Milliseconds 250
        }
    }
}
try {
    $dte=New-Object -ComObject 'TcXaeShell.DTE.15.0'
    Write-Output "Created isolated XAE; evidence: $out"
    Invoke-ComRetry { $dte.SuppressUI=$true; $dte.MainWindow.Visible=$false; $dte.UserControl=$false }
    # Keep COM handles out of the PowerShell output pipeline: some TwinCAT
    # objects expose IEnumVARIANT and get silently unrolled into their children.
    $script:solution=$dte.Solution
    Invoke-ComRetry { $script:project=$script:solution.AddFromTemplate('C:\TwinCAT\3.1\Components\Base\PrjTemplate\TwinCAT Project.tsproj',"$out/System",'MotorPlcCheck') }
    Invoke-ComRetry { $script:manager=$script:project.Object; if(-not $script:manager){throw 'XAE_NOT_READY manager'} }
    Invoke-ComRetry { $script:plc=$script:manager.LookupTreeItem('TIPC'); if(-not $script:plc){throw 'XAE_NOT_READY PLC tree'} }
    if($ArchiveToVerify) {
        $sourceArchive=(Resolve-Path -LiteralPath $ArchiveToVerify).Path
        Invoke-ComRetry { [void]$plc.CreateChild($ProjectName,0,'',$sourceArchive) }
    } else {
    # Generate a standard container with the local XAE version, then import
    # the delivered POUs. This avoids version-specific project import quirks.
    Invoke-ComRetry { [void]$plc.CreateChild($ProjectName,0,'','Standard PLC Template') }
    Invoke-ComRetry { $script:plcProject=$manager.LookupTreeItem("TIPC^$ProjectName^$ProjectName Project") }
    Invoke-ComRetry { $script:mainPou=$manager.LookupTreeItem("TIPC^$ProjectName^$ProjectName Project^POUs^MAIN") }
    [xml]$source=Get-Content -LiteralPath "$out/plc/MAIN.TcPOU" -Raw -Encoding UTF8
    Invoke-ComRetry { $mainPou.DeclarationText=$source.TcPlcObject.POU.Declaration.InnerText; $mainPou.ImplementationText=$source.TcPlcObject.POU.Implementation.ST.InnerText }
    Invoke-ComRetry { [void]$plcProject.CreateChild('GVL_MotorIO',58,'',"$out/plc/GVL_MotorIO.TcGVL") }
    }
    Invoke-ComRetry { $dte.Solution.SaveAs("$out/MotorPlcCheck.sln") }
    Invoke-ComRetry { $dte.Solution.SolutionBuild.Build($true) }
    $errors=$dte.ToolWindows.ErrorList.ErrorItems
    $messages=@()
    for($i=1;$i -le $errors.Count;$i++) {
        $e=$errors.Item($i)
        $messages+="category=$($e.ErrorLevel) $($e.Description) $($e.FileName):$($e.Line)"
    }
    $messages | Set-Content -LiteralPath "$out/errors.txt" -Encoding UTF8
    $messages | Select-Object -First 25
    if($dte.Solution.SolutionBuild.LastBuildInfo -ne 0){throw 'Offline PLC build failed; see errors.txt'}
    Invoke-ComRetry { $plc.ExportChild($ProjectName,"$out/$archiveName.tpzip") }
    if(-not(Test-Path -LiteralPath "$out/$archiveName.tpzip")){throw 'PLC archive export failed'}
    # XAE includes a relative ../../../.../_Boot archive. Do not distribute
    # an automatically generated boot payload or entries escaping the archive.
    Add-Type -AssemblyName System.IO.Compression
    Add-Type -AssemblyName System.IO.Compression.FileSystem
    $zip=[IO.Compression.ZipFile]::Open("$out/$archiveName.tpzip",[IO.Compression.ZipArchiveMode]::Update)
    try {
        @($zip.Entries) | Where-Object { $_.FullName -match '(^|[/\\])\.\.([/\\]|$)|^[A-Za-z]:|^[/\\]' } | ForEach-Object { $_.Delete() }
    } finally { $zip.Dispose() }
    Write-Output "Offline PLC build passed. Evidence: $out (no activation/download)."
} finally {
    if($dte){try {Invoke-ComRetry { $dte.Quit() }} catch {Write-Warning "Could not close owned offline XAE: $_"}}
}
