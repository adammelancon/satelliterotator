@echo off
setlocal EnableDelayedExpansion

REM === start_proxy.bat ===
set SERIAL_PORT=COM7
set BAUD=115200
set LISTEN_HOST=127.0.0.1
set LISTEN_PORT=7777

REM Check if uv is installed
where uv >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo uv is not installed. It is recommended for running this script without dependency issues.
    set /p "CHOICE=Do you want to install uv now? (Y/N): "
    if /i "!CHOICE!"=="Y" (
        echo Installing uv...
        powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
        echo.
        echo uv has been installed.
        echo Please restart this script or your terminal to ensure 'uv' is in your PATH.
        pause
        exit /b
    ) else (
        echo uv is required for the automatic dependency management in this script.
        echo You can try running it manually with python if dependencies are installed.
        echo.
        pause
        exit /b
    )
)

:CHECK_PORT
echo Checking for %SERIAL_PORT%...
powershell -NoProfile -Command "if ([System.IO.Ports.SerialPort]::GetPortNames() -contains '%SERIAL_PORT%') { exit 0 } else { exit 1 }"
if %ERRORLEVEL% EQU 0 (
    goto RUN_PROXY
)

echo Port %SERIAL_PORT% not found!
echo.
echo Available ports:
powershell -NoProfile -Command "[System.IO.Ports.SerialPort]::GetPortNames()"
echo.
set /p "SERIAL_PORT=Please enter a valid COM port (e.g. COM3) or 'exit' to quit: "
if /i "%SERIAL_PORT%"=="exit" exit /b
goto CHECK_PORT

:RUN_PROXY
echo.
echo Starting Rotator Proxy with uv on %SERIAL_PORT%...
uv run "%~dp0rotator_proxy.py" ^
  --serial-port %SERIAL_PORT% ^
  --baud %BAUD% ^
  --listen-host %LISTEN_HOST% ^
  --listen-port %LISTEN_PORT%

pause
