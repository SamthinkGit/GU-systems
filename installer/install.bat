cd /d "%~dp0"
python -m pip install streamlit
streamlit run ./installer/streamlit/app.py || streamlit run ./streamlit/app.py || streamlit run ./app.py
set /p dato=if you see me, please send this logs to the developer
