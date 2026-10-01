$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$headerPath = Join-Path $projectRoot 'Core/Inc/ethercat/ethercat_identity.h'
$path = Join-Path $projectRoot 'EtherCAT/H743-LAN9252-LAB.xml'
$missing = @()
if (-not (Test-Path -LiteralPath $headerPath -PathType Leaf)) { $missing += 'Core/Inc/ethercat/ethercat_identity.h' }
if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { $missing += 'EtherCAT/H743-LAN9252-LAB.xml' }
if ($missing.Count) { throw "Missing required identity files: $($missing -join ', ')" }
$header = Get-Content -LiteralPath $headerPath -Raw
[xml]$xml = Get-Content -LiteralPath $path -Raw

function Equal($actual, $expected, [string]$message) {
    if ("$actual" -ne "$expected") { throw "$message (expected $expected, got $actual)" }
}
function HexValue($value) {
    $s = "$value" -replace '^(#x|0x)', ''
    if ($s -notmatch '^[0-9a-fA-F]+$') { throw "Invalid hexadecimal value: $value" }
    return [Convert]::ToUInt32($s, 16)
}
function DefineValue([string]$name, [string]$type) {
    $pattern = '(?m)^#define\s+' + [regex]::Escape($name) + '\s+' + $type + '_C\((0x[0-9A-Fa-f]+|[0-9]+)\)\s*$'
    $matches = [regex]::Matches($header, $pattern)
    Equal $matches.Count 1 "$name definition count mismatch"
    $value = $matches[0].Groups[1].Value
    if ($value -match '^0x') { return [Convert]::ToUInt32($value.Substring(2), 16) }
    return [Convert]::ToUInt32($value, 10)
}
function LittleEndianValue([string]$hex) {
    if ($hex -notmatch '^[0-9a-fA-F]{8}$') { throw "Expected four little-endian bytes, got $hex" }
    $bytes = [byte[]](0, 2, 4, 6 | ForEach-Object { [Convert]::ToByte($hex.Substring($_, 2), 16) })
    return [BitConverter]::ToUInt32($bytes, 0)
}

$nameMatch = [regex]::Matches($header, '(?m)^#define\s+ETHERCAT_DEVICE_NAME\s+"([^"]+)"\s*$')
Equal $nameMatch.Count 1 'Device name definition count mismatch'
$deviceName = $nameMatch[0].Groups[1].Value
Equal $deviceName 'H743-LAN9252-LAB' 'Lab device name mismatch'
$vendorId = DefineValue 'ETHERCAT_VENDOR_ID' 'UINT32'
$productCode = DefineValue 'ETHERCAT_PRODUCT_CODE' 'UINT32'
$revision = DefineValue 'ETHERCAT_REVISION' 'UINT32'
$serial = DefineValue 'ETHERCAT_SERIAL_NUMBER' 'UINT32'
$siiSize = DefineValue 'ETHERCAT_SII_SIZE_BYTES' 'UINT16'
Equal $vendorId 0x00F0C743 'Lab Vendor ID mismatch'
Equal $productCode 0x00009252 'Lab product code mismatch'
Equal $revision 1 'Lab revision mismatch'
Equal $serial 1 'Lab serial mismatch'
Equal $siiSize 2048 'Lab SII size mismatch'
if ($header -notmatch '(?i)lab.only') { throw 'Vendor ID must be marked lab-only.' }

$device = $xml.SelectSingleNode('/EtherCATInfo/Descriptions/Devices/Device')
if (-not $device) { throw 'ESI Device element is missing.' }
Equal (HexValue $xml.EtherCATInfo.Vendor.Id) $vendorId 'Vendor ID mismatch'
Equal (HexValue $device.Type.ProductCode) $productCode 'Product code mismatch'
Equal (HexValue $device.Type.RevisionNo) $revision 'Revision mismatch'
Equal ([string]$device.Type.InnerText) $deviceName 'Device type mismatch'
if (@($device.SelectNodes('./Name') | Where-Object { $_.InnerText -eq $deviceName }).Count -eq 0) { throw 'Device name mismatch.' }
Equal ([string]$device.GroupType) $deviceName 'Device group mismatch'
if (-not $xml.SelectSingleNode('/EtherCATInfo/Descriptions/Groups/Group[Type="H743-LAN9252-LAB"]')) { throw 'Lab group is missing.' }

$identity = $device.SelectSingleNode('./Profile/Dictionary/Objects/Object[Index="#x1018"]')
if (-not $identity) { throw 'Identity object 0x1018 is missing.' }
$identityDefaults = @('04','43C7F000','52920000','01000000','01000000')
$subitems = @($identity.SelectNodes('./Info/SubItem'))
Equal $subitems.Count $identityDefaults.Count 'Identity object 0x1018 subindex count mismatch'
for ($i=0; $i -lt $identityDefaults.Count; $i++) {
    Equal $subitems[$i].Info.DefaultData $identityDefaults[$i] "Identity object 0x1018 subindex $i mismatch"
}
Equal (LittleEndianValue $subitems[1].Info.DefaultData) $vendorId 'Identity Vendor ID differs from header'
Equal (LittleEndianValue $subitems[2].Info.DefaultData) $productCode 'Identity product code differs from header'
Equal (LittleEndianValue $subitems[3].Info.DefaultData) $revision 'Identity revision differs from header'
Equal (LittleEndianValue $subitems[4].Info.DefaultData) $serial 'Identity serial differs from header'

$eeprom = $device.SelectSingleNode('./Eeprom')
if (-not $eeprom) { throw 'EEPROM description is missing.' }
Equal $eeprom.ByteSize $siiSize 'EEPROM size differs from header'
Equal $eeprom.ConfigData '800E00CC8813f000000000800000' 'LAN9252 SPI ConfigData mismatch'
Equal $eeprom.ConfigData.Substring(0,2) '80' 'PDI Control must select SPI'

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
Check-Pdo 'RxPdo' 0x1600 '2' (,@(0x7000,1,16))
Check-Pdo 'TxPdo' 0x1A00 '3' @(@(0x6000,1,16),@(0x6000,2,32))
function Check-ObjectDefaults([string]$index, [array]$defaults) {
    $object = $device.SelectSingleNode("./Profile/Dictionary/Objects/Object[Index='#x$index']")
    if (-not $object) { throw "Object 0x$index is missing." }
    $items = @($object.SelectNodes('./Info/SubItem'))
    Equal $items.Count $defaults.Count "Object 0x$index subindex count mismatch"
    for ($i=0; $i -lt $defaults.Count; $i++) {
        Equal $items[$i].Info.DefaultData $defaults[$i] "Object 0x$index subindex $i default mismatch"
    }
}
Check-ObjectDefaults '1600' @('01','10010070')
Check-ObjectDefaults '1a00' @('02','10010060','20020060')
Check-ObjectDefaults '1c12' @('01','0016')
Check-ObjectDefaults '1c13' @('01','001a')
Check-ObjectDefaults '7000' @('01','0000')
Check-ObjectDefaults '6000' @('02','0000','00000000')

$mailbox = $device.SelectSingleNode('./Mailbox')
if (-not $mailbox -or @($mailbox.ChildNodes).Count -ne 1 -or $mailbox.FirstChild.LocalName -ne 'CoE') {
    throw 'Only CoE mailbox may be declared.'
}
foreach ($name in 'Dc','FoE','EoE','SoE','AoE') {
    if ($device.SelectSingleNode(".//$name")) { throw "$name must not be declared." }
}
if ($device.OuterXml -match '(?i)CiA\s*402|CiA402') { throw 'CiA402 must not be declared.' }
Write-Output 'EtherCAT ESI identity contract passed'
