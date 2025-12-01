@echo off
setlocal
set "REPO_DIR=%~dp0"
set "VENV_DIR=%REPO_DIR%.venv"

if not exist "%VENV_DIR%\Scripts\python.exe" (
    echo [LiteSim] Creating virtual environment at %VENV_DIR%...
    python -m venv "%VENV_DIR%" 2>nul || py -3 -m venv "%VENV_DIR%" 2>nul

    if not exist "%VENV_DIR%\Scripts\python.exe" (
        echo [LiteSim] Failed to create virtual environment. Install Python 3 and retry.
        exit /b 1
    )
)

set "PYTHON=%VENV_DIR%\Scripts\python.exe"
set "PIP=%VENV_DIR%\Scripts\pip.exe"

if exist "%REPO_DIR%requirements.txt" (
    echo [LiteSim] Ensuring dependencies are installed in the virtual environment...
    "%PYTHON%" -m pip install --upgrade pip >nul
    "%PIP%" install -r "%REPO_DIR%requirements.txt"
)

echo [LiteSim] Launching GUI...
"%PYTHON%" "%REPO_DIR%main.py"
endlocal
