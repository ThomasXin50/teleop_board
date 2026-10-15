<#
.SYNOPSIS
Flash the RP2040 from Windows (the flash.sh equivalent).

.DESCRIPTION
Reboots the board into BOOTSEL over USB (1200-baud touch), copies the UF2 to
the RPI-RP2 drive and waits for the firmware to come back. Rebooting releases
both clutches.

.EXAMPLE
.\flash.ps1
.\flash.ps1 -Uf2 ..\teleop_board_firmware\build\andre_rp2040_controller_bench.uf2
#>
param(
    # Default: the normal firmware in the firmware build folder next to this one.
    [string]$Uf2,
    # Default: the RP2040's COM port, found by its USB vendor ID.
    [string]$Port
)

$ErrorActionPreference = "Stop"
# $PSScriptRoot is empty in param() defaults on Windows PowerShell 5.1.
if (-not $Uf2) {
    $here = Split-Path -Parent $MyInvocation.MyCommand.Path
    $Uf2 = Join-Path $here "..\teleop_board_firmware\build\andre_rp2040_controller.uf2"
}

function Find-Rp2040Port {
    $device = Get-CimInstance Win32_PnPEntity |
        Where-Object { $_.PNPDeviceID -like 'USB\VID_2E8A*' -and $_.Name -match '\((COM\d+)\)' } |
        Select-Object -First 1
    if ($device -and $device.Name -match '\((COM\d+)\)') { return $Matches[1] }
    return $null
}

function Find-BootselDrive {
    $disk = Get-CimInstance Win32_LogicalDisk -Filter "VolumeName='RPI-RP2'" | Select-Object -First 1
    if ($disk) { return $disk.DeviceID }
    return $null
}

function Wait-For([scriptblock]$probe, [int]$seconds) {
    $deadline = (Get-Date).AddSeconds($seconds)
    do {
        $result = & $probe
        if ($result) { return $result }
        Start-Sleep -Milliseconds 500
    } while ((Get-Date) -lt $deadline)
    return $null
}

if (-not (Test-Path $Uf2)) { throw "firmware not found: $Uf2" }
$Uf2 = (Resolve-Path $Uf2).Path

$drive = Find-BootselDrive
if (-not $drive) {
    if (-not $Port) { $Port = Find-Rp2040Port }
    if (-not $Port) { throw "no RP2040 COM port and none in BOOTSEL (hold BOOTSEL while plugging it in)" }
    Write-Host "rebooting RP2040 on $Port into BOOTSEL"
    $serial = New-Object System.IO.Ports.SerialPort $Port, 1200
    try { $serial.Open(); $serial.Close() } catch { }  # the board drops off USB right away
    $drive = Wait-For { Find-BootselDrive } 15
    if (-not $drive) { throw "RP2040 didn't enter BOOTSEL; hold BOOTSEL while replugging it" }
}

Write-Host "copying $(Split-Path $Uf2 -Leaf) to $drive"
Copy-Item $Uf2 "$drive\"
$Port = Wait-For { Find-Rp2040Port } 15
if (-not $Port) { throw "firmware didn't come back as a COM port" }
Write-Host "flashed; RP2040 running again on $Port (clutches released)"
