from pathlib import Path


def add_to_dotenv(key: str, value: str, dotenv_path: Path):
    """Add a key-value pair to the .env file. If the key already exists, update its value."""
    if not dotenv_path.exists():
        dotenv_path.touch()

    lines = dotenv_path.read_text(encoding="utf-8").splitlines()
    key_found = False
    new_lines = []

    for line in lines:
        if line.startswith(f"{key}="):
            new_lines.append(f"{key}={value}")
            key_found = True
        else:
            new_lines.append(line)

    if not key_found:
        new_lines.append(f"{key}={value}")

    dotenv_path.write_text("\n".join(new_lines) + "\n", encoding="utf-8")
