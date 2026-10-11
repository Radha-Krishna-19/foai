@echo off
cd /d "%~dp0"
echo Starting the demo ... a browser window will open.
python -m pip install -q -r requirements.txt
python run_demo.py
pause
