@echo off
setlocal
cd /d "%~dp0"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0train_windows_gpu.ps1"
set "TRAIN_EXIT=%ERRORLEVEL%"
if not "%TRAIN_EXIT%"=="0" echo See setup.log, environment_check.log and training.log for details.
pause
exit /b %TRAIN_EXIT%
