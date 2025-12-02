@echo off
setlocal
set "ROOT=%~dp0"

if exist "%ROOT%\\.venv\\Scripts\\python.exe" (
    "%ROOT%\\.venv\\Scripts\\python.exe" "%ROOT%\\gui_qt.py"
) else (
    python "%ROOT%\\gui_qt.py"
)
