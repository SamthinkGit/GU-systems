import requests

BASE_URL = "http://127.0.0.1:8000"

BUTTONS = {
    "1": "<nova_button>",
    "2": "<resume_button>",
    "3": "<exit_button>",
}


def send_config():
    config = {}
    print("\nEnter configuration values (empty key to stop):")
    while True:
        key = input("Key: ")
        if not key:
            break
        value = input("Value: ")
        config[key] = value

    response = requests.post(f"{BASE_URL}/config", json={"config": config})
    print("[RESPONSE]", response.json())


def send_extra():
    extra = {}
    print("\nEnter extra values (empty key to stop):")
    while True:
        key = input("Key: ")
        if not key:
            break
        value = input("Value: ")
        extra[key] = value

    response = requests.post(f"{BASE_URL}/extra", json={"extra": extra})
    print("[RESPONSE]", response.json())


def press_button():
    print("\nAvailable buttons:")
    for k, v in BUTTONS.items():
        print(f"  {k}. {v}")
    choice = input("Select a button: ").strip()
    button = BUTTONS.get(choice)
    if not button:
        print("Invalid choice.")
        return
    response = requests.post(f"{BASE_URL}/button", json={"button": button})
    print("[RESPONSE]", response.json())


def get_next_state(current_state):
    payload = {"state": current_state}
    print("[INFO] Waiting for next state...")
    response = requests.post(f"{BASE_URL}/next_state", json=payload)
    print("[RESPONSE]", response.json())
    return response.json().get("next_state")


def main():
    current_state = "<home>"
    while True:
        print("\n=== API Simulation Menu ===")
        print("1. Send config")
        print("2. Press button")
        print("3. Get next state")
        print("4. Send extra data")
        choice = input("Choose an option: ").strip()

        match choice:
            case "1":
                send_config()
            case "2":
                press_button()
            case "3":
                current_state = get_next_state(current_state)
            case "4":
                send_extra()
            case _:
                print("Invalid choice.")


if __name__ == "__main__":
    main()
