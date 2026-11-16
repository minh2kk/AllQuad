@echo off
cd /d "%~dp0"
set PYTHONUTF8=1
set PYTHONIOENCODING=utf-8
set PY=%USERPROFILE%\.platformio\penv\Scripts\python.exe
if not exist "%PY%" set PY=python
"%PY%" aqflash.py
echo.
echo Xong. Ket qua trong aqflash_log.txt
timeout /t 8
