$ErrorActionPreference = 'Stop'
$path = Join-Path (Split-Path -Parent $PSScriptRoot) 'EtherCAT/H743-LAN9252-LAB.xml'
if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw 'Missing required ESI: EtherCAT/H743-LAN9252-LAB.xml' }
[xml]$xml = Get-Content -LiteralPath $path -Raw

function Equal($actual, $expected, [string]$message) {
    if ("$actual" -ne "$expected") { throw "$message (expected $expected, got $actual)" }
}
function HexValue($value) {
    $s = "$value" -replace '^(#x|0x)', ''
    if ($s -notmatch '^[0-9a-fA-F]+$') { throw "Invalid hexadecimal value: $value" }
    return [Convert]::ToUInt32($s, 16)
}

$device = $xml.SelectSingleNode('/EtherCATInfo/Descriptions/Devices/Device')
if (-not $device) { throw 'ESI Device element is missing.' }
Equal (HexValue $xml.EtherCATInfo.Vendor.Id) 0x00F0C743 'Vendor ID mismatch'
Equal (HexValue $device.Type.ProductCode) 0x00009252 'Product code mismatch'
Equal (HexValue $device.Type.RevisionNo) 1 'Revision mismatch'
Equal ([string]$device.Type.InnerText) 'H743-LAN9252-LAB' 'Device type mismatch'
if (@($device.SelectNodes('./Name') | Where-Object { $_.InnerText -eq 'H743-LAN9252-LAB' }).Count -eq 0) { throw 'Device name mismatch.' }

$identity = $device.SelectSingleNode('./Profile/Dictionary/Objects/Object[Index="#x1018"]')
if (-not $identity) { throw 'Identity object 0x1018 is missing.' }
$identityDefaults = @('04','43C7F000','52920000','01000000','01000000')
$subitems = @($identity.SelectNodes('./Info/SubItem'))
Equal $subitems.Count $identityDefaults.Count 'Identity object 0x1018 subindex count mismatch'
for ($i=0; $i -lt $identityDefaults.Count; $i++) {
    Equal $subitems[$i].Info.DefaultData $identityDefaults[$i] "Identity object 0x1018 subindex $i mismatch"
}

$sm = @($device.SelectNodes('./Sm'))
Equal $sm.Count 4 'SyncManager count mismatch'
$expected = @(@(0x1000,128,'MBoxOut'),@(0x1080,128,'MBoxIn'),@(0x1100,2,'Outputs'),@(0x1400,6,'Inputs'))
for ($i=0; $i -lt 4; $i++) {
    Equal (HexValue $sm[$i].StartAddress) $expected[$i][0] "SM$i start mismatch"
    Equal $sm[$i].DefaultSize $expected[$i][1] "SM$i size mismatch"
    Equal $sm[$i].InnerText $expected[$i][2] "SM$i direction mismatch"
}
function Check-Pdo([string]$kind, [uint32]$index, [string]$smNumber, [array]$entries) {
    $pdo = @($device.SelectNodes("./$kind") | Where-Object { (HexValue $_.Index) -eq $index })
    Equal $pdo.Count 1 "$kind index mismatch"
    Equal $pdo[0].Sm $smNumber "$kind SM assignment mismatch"
    $actual = @($pdo[0].Entry)
    Equal $actual.Count $entries.Count "$kind entry count mismatch"
    for ($i=0; $i -lt $entries.Count; $i++) {
        Equal (HexValue $actual[$i].Index) $entries[$i][0] "$kind entry $i index mismatch"
        Equal (HexValue $actual[$i].SubIndex) $entries[$i][1] "$kind entry $i subindex mismatch"
        Equal $actual[$i].BitLen $entries[$i][2] "$kind entry $i bit length mismatch"
    }
}
Check-Pdo 'RxPdo' 0x1600 '2' @(@(0x7000,1,16))
Check-Pdo 'TxPdo' 0x1A00 '3' @(@(0x6000,1,16),@(0x6000,2,32))
foreach ($name in 'Dc','FoE','EoE','SoE','AoE') {
    if ($device.SelectSingleNode(".//$name")) { throw "$name must not be declared." }
}
if ($device.OuterXml -match '(?i)CiA\s*402|CiA402') { throw 'CiA402 must not be declared.' }
Write-Output 'EtherCAT ESI identity contract passed'
