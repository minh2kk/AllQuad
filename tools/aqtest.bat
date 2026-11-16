@echo off
cd /d "%~dp0"
set PY=%USERPROFILE%\.platformio\penv\Scripts\python.exe
if not exist "%PY%" set PY=python
"%PY%" aqtest.py
echo.
echo Xong. Ket qua trong aqtest_log.txt
timeout /t 8
