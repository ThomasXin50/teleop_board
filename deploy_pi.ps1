<#
.SYNOPSIS
Copies teleop_pi_scripts (and optionally the firmware UF2s) from this PC to the Pi.

.DESCRIPTION
Uses the tar, scp and ssh that ship with Windows 10/11. Text files are converted
to LF line endings on the way, so the shell scripts and #! lines work on the Pi.
The Pi's .venv is left alone. Without an SSH key you are asked for the Pi's
password once per scp/ssh step.

.EXAMPLE
.\deploy_pi.ps1 -PiHost raspberrypi.local
.\deploy_pi.ps1 -PiHost 192.168.1.50 -Firmware -Flash
.\deploy_pi.ps1 -PiHost 192.168.1.50 -Setup        # first time: venv, groups, tests
#>
param(
    [Parameter(Mandatory = $true)][string]$PiHost,
    [string]$User = "cev",
    # Relative to the Pi user's home directory.
    [string]$RemoteDir = "teleop-pi",
    # Also copy both firmware UF2s to the Pi's home directory (where flash.sh looks).
    [switch]$Firmware,
    # Default: teleop_board_firmware\build next to this script.
    [string]$FirmwareDir,
    # Run setup.sh on the Pi after copying.
    [switch]$Setup,
    # Flash the normal firmware after copying (implies -Firmware).
    [switch]$Flash
)

$ErrorActionPreference = "Stop"
# $PSScriptRoot is empty in param() defaults on Windows PowerShell 5.1.
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
if (-not $FirmwareDir) { $FirmwareDir = Join-Path $here "teleop_board_firmware\build" }
$target = "$User@$PiHost"
$source = Join-Path $here "teleop_pi_scripts"
if ($Flash) { $Firmware = $true }

function Invoke-Native([string]$what, [scriptblock]$command) {
    & $command
    if ($LASTEXITCODE -ne 0) { throw "$what failed (exit code $LASTEXITCODE)" }
}

$firmwareFiles = @()
if ($Firmware) {
    foreach ($name in "andre_rp2040_controller.uf2", "andre_rp2040_controller_bench.uf2") {
        $path = Join-Path $FirmwareDir $name
        if (-not (Test-Path $path)) { throw "missing $path (build the firmware, or pass -FirmwareDir)" }
        $firmwareFiles += $path
    }
}

# Stage a copy with LF line endings, without caches or a local venv.
$stage = Join-Path ([IO.Path]::GetTempPath()) ("teleop-pi-" + [guid]::NewGuid().ToString("N").Substring(0, 8))
$archive = "$stage.tar"
New-Item -ItemType Directory -Path $stage | Out-Null
try {
    $utf8 = New-Object Text.UTF8Encoding($false)
    Get-ChildItem $source -File | Where-Object { $_.Name -notlike "*.pyc" } | ForEach-Object {
        $dest = Join-Path $stage $_.Name
        if ($_.Extension -in ".py", ".sh", ".txt", ".md") {
            $text = [IO.File]::ReadAllText($_.FullName).Replace("`r`n", "`n")
            [IO.File]::WriteAllText($dest, $text, $utf8)
        } else {
            Copy-Item $_.FullName $dest
        }
    }
    Invoke-Native "tar" { tar -C $stage -cf $archive . }

    Write-Host "copying scripts to ${target}:~/$RemoteDir"
    Invoke-Native "scp" { scp $archive "${target}:teleop-pi-upload.tar" }
    $remote = "mkdir -p '$RemoteDir' && tar -xf teleop-pi-upload.tar -C '$RemoteDir' && rm teleop-pi-upload.tar"
    Invoke-Native "ssh" { ssh $target $remote }

    if ($Firmware) {
        Write-Host "copying firmware to ${target}:~/"
        Invoke-Native "scp" { scp @firmwareFiles "${target}:" }
    }
} finally {
    Remove-Item -Recurse -Force $stage -ErrorAction SilentlyContinue
    Remove-Item -Force $archive -ErrorAction SilentlyContinue
}

# -t gives these an interactive terminal (sudo password prompts, live output).
if ($Setup) { Invoke-Native "setup.sh" { ssh -t $target "cd '$RemoteDir' && sh setup.sh" } }
if ($Flash) { Invoke-Native "flash.sh" { ssh -t $target "cd '$RemoteDir' && sh flash.sh" } }
Write-Host "done"
