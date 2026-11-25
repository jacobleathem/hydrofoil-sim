@echo off
setlocal
pushd "%~dp0"

set "PY="
set "VENV_PY=%~dp0.venv\Scripts\python.exe"

if exist "%VENV_PY%" call :try_exe "%VENV_PY%"
if not defined PY call :try_exe python
if not defined PY call :try_exe python3
if not defined PY call :try_py3

if not defined PY (
    echo Python 3.10+ was not found or could not run.
    echo Install Python 3.10+ ^(with PATH enabled^) and re-run.
    pause
    popd
    exit /b 1
)

rem Reuse current Python directly if dependencies are already installed; otherwise isolate in .venv.
"%PY%" -c "import mujoco, numpy, scipy, matplotlib, plotly, PIL" >nul 2>&1
if not errorlevel 1 goto :menu

if /I "%PY%"=="%VENV_PY%" goto :install_reqs
echo Creating virtual environment in .venv...
"%PY%" -m venv "%~dp0.venv"
if errorlevel 1 (
    echo Failed to create .venv.
    pause
    popd
    exit /b 1
)
set "PY=%VENV_PY%"

:install_reqs
echo Installing requirements into %PY%...
"%PY%" -m pip install -r requirements.txt
if errorlevel 1 (
    echo Failed to install requirements.
    pause
    popd
    exit /b 1
)

:menu
echo.
echo Hydrofoil Sim Launcher ^(Python: %PY%^)
echo   [1] Web Dashboard + Boat Editor + CAD Boot-Up ^(default^)
echo   [2] CAD Boat 3D Viewer - Auto Combined + Supervisor + Waves + Noisy Sensors
echo   [3] Manned 275 kg 3D Viewer - Auto Combined + Supervisor + Calm
echo   [4] Model 25 kg 3D Viewer   - Raw LQR/PID + Calm
echo   [5] Model Consistency Check
echo   [6] Run Pytest Suite
echo.
set "CHOICE=1"
set /p "CHOICE=Select option [1-6] (Enter for 1): "
for /f "tokens=1" %%C in ("%CHOICE%") do set "CHOICE=%%C"

if "%CHOICE%"=="1" (
    echo Starting web dashboard on http://127.0.0.1:8765 ^(Ctrl+C to stop^)...
    "%PY%" -m hydrofoil.sim_server
) else if "%CHOICE%"=="2" (
    "%PY%" -m hydrofoil.viewer --preset cad --controller auto --supervisor --sensors noisy --waves
) else if "%CHOICE%"=="3" (
    "%PY%" -m hydrofoil.viewer --preset manned --controller auto --supervisor
) else if "%CHOICE%"=="4" (
    "%PY%" -m hydrofoil.viewer --preset model
) else if "%CHOICE%"=="5" (
    "%PY%" -m hydrofoil.consistency
    pause
) else if "%CHOICE%"=="6" (
    "%PY%" -m pytest -v
    pause
) else (
    echo Invalid choice: %CHOICE%
    pause
    popd
    exit /b 1
)

if errorlevel 1 (
    echo.
    echo Command exited with an error.
    pause
)
popd
exit /b 0

:try_exe
"%~1" -c "import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)" >nul 2>&1
if not errorlevel 1 set "PY=%~1"
exit /b 0

:try_py3
py -3 -c "import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)" >nul 2>&1
if not errorlevel 1 (
    for /f "delims=" %%I in ('py -3 -c "import sys; print(sys.executable)"') do set "PY=%%I"
)
exit /b 0
