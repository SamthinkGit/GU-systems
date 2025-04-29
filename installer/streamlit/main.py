from pathlib import Path

from streamlit.web.bootstrap import run

if __name__ == "__main__":
    args = []
    app = Path(__file__).parent / "app.py"
    run(str(app.resolve().absolute()), False, args, {})
