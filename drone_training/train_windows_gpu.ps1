# Use an existing CUDA/YOLO environment; never require a specific Python version.
$ErrorActionPreference = 'Continue'
Set-Location -LiteralPath $PSScriptRoot
$env:PYTHONUTF8 = '1'
$env:PYTHONUNBUFFERED = '1'
$diagnostics = Join-Path $PSScriptRoot 'environment_check.log'
Set-Content -LiteralPath $diagnostics -Value ('Environment scan: ' + (Get-Date)) -Encoding UTF8
function Report([string]$message) {
    Write-Host $message
    Add-Content -LiteralPath $diagnostics -Value $message -Encoding UTF8
}
$candidates = New-Object 'System.Collections.Generic.List[string]'
function Add-Python([string]$path) {
    if ($path -and (Test-Path -LiteralPath $path -PathType Leaf)) {
        $full = (Resolve-Path -LiteralPath $path).Path
        # Skip the Microsoft Store placeholder, which can open a Store window.
        if ($full -notmatch '\\Microsoft\\WindowsApps\\' -and -not $candidates.Contains($full)) {
            $candidates.Add($full)
        }
    }
}
# An explicit path or an activated environment takes priority.
Add-Python $env:TRAIN_PYTHON
if (Test-Path -LiteralPath 'python_path.txt') {
    Add-Python ((Get-Content -LiteralPath 'python_path.txt' -Raw).Trim().Trim('"'))
}
if ($env:CONDA_PREFIX) { Add-Python (Join-Path $env:CONDA_PREFIX 'python.exe') }
if ($env:VIRTUAL_ENV) { Add-Python (Join-Path $env:VIRTUAL_ENV 'Scripts\python.exe') }
foreach ($name in @('python.exe', 'python3.exe')) {
    foreach ($command in @(Get-Command $name -All -ErrorAction SilentlyContinue)) {
        Add-Python $command.Source
    }
}
# A yolo.exe entry point usually lives beside its environment's Python.
foreach ($command in @(Get-Command yolo.exe -All -ErrorAction SilentlyContinue)) {
    $scripts = Split-Path -Parent $command.Source
    Add-Python (Join-Path $scripts 'python.exe')
    Add-Python (Join-Path (Split-Path -Parent $scripts) 'python.exe')
}
if (Get-Command py.exe -ErrorAction SilentlyContinue) {
    $listed = & py.exe -0p 2>&1
    foreach ($line in $listed) {
        if ([string]$line -match '([A-Za-z]:\\.*?python(?:\d+(?:\.\d+)?)?\.exe)\s*$') {
            Add-Python $Matches[1]
        }
    }
}
# Conda records envs even when conda is absent from a double-clicked CMD PATH.
$envFile = Join-Path $env:USERPROFILE '.conda\environments.txt'
if (Test-Path -LiteralPath $envFile) {
    foreach ($prefix in Get-Content -LiteralPath $envFile) {
        if ($prefix.Trim()) { Add-Python (Join-Path ($prefix.Trim()) 'python.exe') }
    }
}
if (Get-Command conda -ErrorAction SilentlyContinue) {
    try {
        $info = (& conda env list --json 2>$null | Out-String | ConvertFrom-Json)
        foreach ($prefix in $info.envs) { Add-Python (Join-Path $prefix 'python.exe') }
    } catch { Report ('Could not list conda envs: ' + $_.Exception.Message) }
}
foreach ($base in @($env:USERPROFILE, $env:LOCALAPPDATA, $env:ProgramData)) {
    if (-not $base) { continue }
    foreach ($name in @('anaconda3', 'miniconda3', 'miniforge3', 'mambaforge')) {
        $prefix = Join-Path $base $name
        Add-Python (Join-Path $prefix 'python.exe')
        foreach ($envDir in @(Get-ChildItem -Path (Join-Path $prefix 'envs') -Directory -ErrorAction SilentlyContinue)) {
            Add-Python (Join-Path $envDir.FullName 'python.exe')
        }
    }
}
foreach ($directory in @(Get-ChildItem -Path (Join-Path $env:LOCALAPPDATA 'Programs\Python') -Directory -ErrorAction SilentlyContinue)) {
    Add-Python (Join-Path $directory.FullName 'python.exe')
}
Add-Python (Join-Path $PSScriptRoot '.venv\Scripts\python.exe')
$selected = $null
$probe = Join-Path $PSScriptRoot 'environment_probe.json'
foreach ($python in $candidates) {
    Report ('Checking: ' + $python)
    Remove-Item -LiteralPath $probe -ErrorAction SilentlyContinue
    $savedPath = $env:PATH
    $prefix = Split-Path -Parent $python
    $env:PATH = "$prefix;$prefix\Scripts;$prefix\Library\bin;" + $savedPath
    & $python (Join-Path $PSScriptRoot 'probe_training_python.py') $probe 2>&1 | ForEach-Object { Report ([string]$_) }
    $env:PATH = $savedPath
    if (Test-Path -LiteralPath $probe) {
        try {
            $result = Get-Content -LiteralPath $probe -Raw -Encoding UTF8 | ConvertFrom-Json
            if ($result.ready) { $selected = $python; break }
        } catch { Report ('Invalid probe result: ' + $_.Exception.Message) }
    }
}
if (-not $selected) {
    Report 'No complete CUDA + YOLO environment found. Preparing a dedicated training environment.'
    try {
        & (Join-Path $PSScriptRoot 'setup_windows_training.ps1') -PythonCandidates ($candidates.ToArray())
        $result = Get-Content -LiteralPath $probe -Raw -Encoding UTF8 | ConvertFrom-Json
        if (-not $result.ready) { throw 'CUDA probe did not pass after setup.' }
        $selected = $result.python
    } catch {
        Report ('Environment setup failed: ' + $_.Exception.Message)
        Report 'See setup.log. Re-run this launcher after resolving the reported failure.'
        exit 1
    }
}
$prefix = Split-Path -Parent $selected
$env:PATH = "$prefix;$prefix\Scripts;$prefix\Library\bin;" + $env:PATH
Report ('Selected: ' + $selected)
Report ('GPU: ' + $result.gpu)
Report 'Starting training. Progress is also saved to training.log.'
# PowerShell 5 handles native stderr as records; keep Continue and preserve exit code.
& $selected -u (Join-Path $PSScriptRoot 'train_drone.py') --data dataset --weights yolov8n.pt 2>&1 |
    ForEach-Object { [string]$_ } | Tee-Object -FilePath (Join-Path $PSScriptRoot 'training.log')
$trainExit = $LASTEXITCODE
if ($trainExit -ne 0) {
    Report ('Training exited with code ' + $trainExit + '. See training.log.')
    exit $trainExit
}
Report 'Training finished. Models: trained_model; metrics and plots: runs.'
exit 0
