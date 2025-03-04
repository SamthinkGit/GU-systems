from typing import Callable, Literal, Any
import platform
import subprocess


def get_os_release_info():
    return subprocess.run(
        ["hostnamectl"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
    ).stdout


def check_distro():

    os_info = platform.system()
    if os_info == "Windows":
        return "Windows"

    os_info = get_os_release_info()
    if "raspberry" in os_info.lower():
        return "Raspbian"

    elif "ubuntu" in os_info.lower():
        return "Ubuntu"
    else:
        return "Unknown"


OsWrappedFunction = Callable[
    [Literal["Windows", "Raspbian", "Ubuntu", "Uknown"], Any], Any
]


def osmethod(func: Callable) -> OsWrappedFunction:

    def wrapper(*args, **kwargs):
        distro = check_distro()
        result = func(distro, *args, **kwargs)
        return result

    wrapper: OsWrappedFunction
    return wrapper


if __name__ == "__main__":
    print("You are using:", check_distro())
