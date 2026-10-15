<#
.SYNOPSIS
BENCH ONLY: drive the throttle output and the steering motor with the safety
interlocks off, then put the normal firmware back (the bench_drive.sh equivalent).

.DESCRIPTION
Flashes andre_rp2040_controller_bench.uf2, whose throttle ignores calibration,
sensor and fault interlocks (brake > 5% and the comms timeout still cut it),
and whose steering runs open loop: stick deflection sets the motor's speed (up
to BENCH_STEER_MAX_STEPS_PER_S) and direction, centered stops it, no travel
limits. Wheels off the ground and the steering motor uncoupled. Quitting
teleop (Q or Ctrl+C) restores andre_rp2040_controller.uf2. Closing the window
instead skips that: run .\flash.ps1 afterwards.

.EXAMPLE
.\bench_drive.ps1
.\bench_drive.ps1 --input keyboard     # extra arguments go to teleop.py
#>
param(
    # Defaults: the firmware build folder next to this one.
    [string]$BenchUf2,
    [string]$NormalUf2,
    [Parameter(ValueFromRemainingArguments = $true)][string[]]$TeleopArgs
)

$ErrorActionPreference = "Stop"
# $PSScriptRoot is empty in param() defaults on Windows PowerShell 5.1.
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$build = Join-Path $here "..\teleop_board_firmware\build"
if (-not $BenchUf2) { $BenchUf2 = Join-Path $build "andre_rp2040_controller_bench.uf2" }
if (-not $NormalUf2) { $NormalUf2 = Join-Path $build "andre_rp2040_controller.uf2" }
foreach ($file in $BenchUf2, $NormalUf2) {
    if (-not (Test-Path $file)) { throw "missing $file (build the firmware first)" }
}

Write-Host @"
BENCH MODE: throttle interlocks OFF, steering motor open loop with NO limits.
  - drive wheels off the ground, nobody near them
  - steering motor NOT coupled to the column (it would drive into the stops)
  - e-stop within reach
"@
if ((Read-Host "Type BENCH to continue") -cne "BENCH") { Write-Host "cancelled"; exit 1 }

$python = Join-Path $here ".venv\Scripts\python.exe"
if (-not (Test-Path $python)) { $python = "python" }

& (Join-Path $here "flash.ps1") -Uf2 $BenchUf2
try {
    & $python (Join-Path $here "teleop.py") --max-speed 10 --speed-step 1 @TeleopArgs
} finally {
    Write-Host "restoring normal firmware"
    & (Join-Path $here "flash.ps1") -Uf2 $NormalUf2
}
