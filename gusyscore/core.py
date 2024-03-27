from pathlib import Path

def get_core_path() -> Path:    

    path = Path(__file__).parent.resolve()
    assert str(path).endswith("gusyscore"), f"Path resolved with get_core_path is not safe: {path}"
    return path

