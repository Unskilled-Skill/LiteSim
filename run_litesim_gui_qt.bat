@echo off
setlocal
set "ROOT=%~dp0"

if exist "%ROOT%\\.venv\\Scripts\\python.exe" (
    start "" "%ROOT%\\.venv\\Scripts\\pythonw.exe" "%ROOT%\\launch_gui_qt.py"
) else (
    start "" pythonw "%ROOT%\\launch_gui_qt.py"
)
exit /b 0
