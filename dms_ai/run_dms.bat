@echo off
title AI Driver Monitoring System (DMS) & Safety Rover Interlock
echo ==============================================================
echo  Launching AI Driver Monitoring & Safety Rover Interlock
echo ==============================================================
echo.

cd /d "%~dp0"

echo [1/2] Verifying Python dependencies...
python -m pip install -r requirements.txt --quiet

echo.
echo [2/2] Starting AI Vision Pipeline & FastAPI Server...
python drowsiness_detector.py
pause
