cd /d "%~dp0"
python -m pip install streamlit
python ./installer/streamlit/main.py || python ./streamlit/main.py
set /p dato=if you see me, please send this logs to the developer
