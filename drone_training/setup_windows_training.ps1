# Called only when no usable existing CUDA + YOLO environment was found.
param([string[]]$PythonCandidates)
$ErrorActionPreference = 'Stop'
$env:PYTHONUTF8 = '1'
$setupLog = Join-Path $PSScriptRoot 'setup.log'
Set-Content -LiteralPath $setupLog -Value ('Training environment setup: ' + (Get-Date)) -Encoding UTF8
function Setup-Message([string]$message) {
    Write-Host $message
    Add-Content -LiteralPath $setupLog -Value $message -Encoding UTF8
}
function Run-Step([string]$program, [string[]]$arguments) {
    Setup-Message ('Running: ' + $program + ' ' + ($arguments -join ' '))
    # Windows PowerShell represents native stderr as error records, including
    # normal pip progress. Check the process exit code, not those records.
    $ErrorActionPreference = 'Continue'
    & $program @arguments 2>&1 | ForEach-Object { Setup-Message ([string]$_) }
    $code = $LASTEXITCODE
    if ($code -ne 0) { throw ('Command failed with exit code ' + $code + '. See setup.log.') }
}
try {
    $condaExe = $null
    foreach ($python in $PythonCandidates) {
        $directory = Split-Path -Parent $python
        for ($level = 0; $level -lt 4 -and $directory; $level++) {
            $possible = Join-Path $directory 'Scripts\conda.exe'
            if (Test-Path -LiteralPath $possible -PathType Leaf) { $condaExe = $possible; break }
            $directory = Split-Path -Parent $directory
        }
        if ($condaExe) { break }
    }
    if (-not $condaExe -and $env:CONDA_EXE -and (Test-Path -LiteralPath $env:CONDA_EXE)) {
        $condaExe = $env:CONDA_EXE
    }
    if (-not $condaExe) {
        $command = Get-Command conda.exe -ErrorAction SilentlyContinue
        if ($command) { $condaExe = $command.Source }
    }
    if (-not $condaExe) { throw 'Miniconda was not found. See environment_check.log for discovered Python paths.' }
    # Keep the environment on the host disk, outside the VMware shared folder.
    $prefix = Join-Path $env:LOCALAPPDATA 'PX4-FLY\drone-training-env'
    $python = Join-Path $prefix 'python.exe'
    $marker = Join-Path $prefix 'px4-drone-setup.json'
    $condaBase = Split-Path -Parent (Split-Path -Parent $condaExe)
    # Conda activation is not needed when using absolute paths, but Windows
    # extension modules still need the environment's DLL directories on PATH.
    $env:PATH = "$prefix;$prefix\Scripts;$prefix\Library\bin;$condaBase\Library\bin;" + $env:PATH
    Setup-Message ('Dedicated training environment: ' + $prefix)
    Setup-Message 'Initial download includes CUDA PyTorch (several GB). Keep this window open.'
    if (-not (Test-Path -LiteralPath $python)) {
        Run-Step $condaExe @('create', '--yes', '--prefix', $prefix, '--override-channels',
                             '--channel', 'conda-forge', 'python=3.11', 'pip')
    }
    if (-not (Test-Path -LiteralPath $marker)) {
        Run-Step $python @('-m', 'pip', 'install', '--disable-pip-version-check',
                           '--timeout', '120', '--retries', '5',
                           'torch==2.7.1', 'torchvision==0.22.1',
                           '--index-url', 'https://download.pytorch.org/whl/cu126')
        Run-Step $python @('-m', 'pip', 'install', '--disable-pip-version-check',
                           '--timeout', '120', '--retries', '5',
                           'ultralytics==8.3.203', 'numpy<2', 'opencv-python<4.12', 'onnx==1.17.0')
        Set-Content -LiteralPath $marker -Value '{"torch":"2.7.1+cu126","ultralytics":"8.3.203"}' -Encoding UTF8
    }
    $probe = Join-Path $PSScriptRoot 'environment_probe.json'
    Run-Step $python @((Join-Path $PSScriptRoot 'probe_training_python.py'), $probe)
    $result = Get-Content -LiteralPath $probe -Raw -Encoding UTF8 | ConvertFrom-Json
    if (-not $result.ready) { throw 'Installed environment cannot run CUDA. Check setup.log and NVIDIA driver.' }
    Set-Content -LiteralPath (Join-Path $PSScriptRoot 'python_path.txt') -Value $python -Encoding UTF8
    Setup-Message ('GPU verified: ' + $result.gpu)
    Setup-Message 'Environment is ready; continuing to training.'
} catch {
    Setup-Message ('Setup failed: ' + $_.Exception.Message)
    throw
}
