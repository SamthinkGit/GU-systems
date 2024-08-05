import subprocess


def get_os_release_info():
    return subprocess.run(
        ["hostnamectl"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    ).stdout


def check_distro():
    os_info = get_os_release_info()

    if "raspberry" in os_info.lower():
        return "Raspbian"

    elif "ubuntu" in os_info.lower():
        return "Ubuntu"
    else:
        return "Unknown"


if __name__ == "__main__":
    print("You are using:", check_distro())
