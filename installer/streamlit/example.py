import locale
import os
import subprocess


def run_command_live(cmd):
    # Elegimos shell solo si es necesario (Windows normalmente sí necesita)
    use_shell = os.name == "nt"  # 'nt' = Windows, 'posix' = Linux/Mac
    system_encoding = locale.getpreferredencoding()

    process = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        shell=use_shell,
        encoding=system_encoding,  # aseguramos codificación
    )

    result = []
    for line in process.stdout:
        if line:
            print(line, end="")
            result.append(line)

    process.stdout.close()
    return_code = process.wait()
    return return_code, "".join(result)


if __name__ == "__main__":
    cmd = ["ping", "miaw"]  # Windows usa "-n"

    # Correr el comando
    exit_code, result = run_command_live(cmd)
    print(exit_code)
    print(result)
