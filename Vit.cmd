@echo off
setlocal
set PYTHONUTF8=1
set OPENBLAS_NUM_THREADS=1
set "PY=%~dp0.venv\Scripts\python.exe"
if not exist "%PY%" set "PY=%USERPROFILE%\ROBOT\.venv\Scripts\python.exe"
if not exist "%PY%" set "PY=python"
"%PY%" -B -X utf8 "%~dp0canard.py" %*
set "rc=%errorlevel%"
if not "%rc%"=="0" if "%~1"=="" pause
exit /b %rc%

