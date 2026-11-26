@echo off
setlocal
python "%~dp0tools\start_mqtt_dashboard.py"
set "launcher_exit=%errorlevel%"
if not "%launcher_exit%"=="0" pause
exit /b %launcher_exit%
