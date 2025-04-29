import locale
import os
import re
import subprocess


class PersistentShell:
    def __init__(self):
        use_shell = os.name == "nt"
        self.system_encoding = locale.getpreferredencoding()
        self.is_windows = os.name == "nt"

        if self.is_windows:
            shell_cmd = ["powershell"]
        else:
            shell_cmd = ["bash"]

        self.process = subprocess.Popen(
            shell_cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            encoding=self.system_encoding,
            shell=use_shell,
        )

    def send_command(self, command: str):
        if not self.process.stdin:
            raise RuntimeError("No stdin available to send commands.")

        self.process.stdin.write(command + "\n")
        print(command)
        self.process.stdin.flush()

        if self.is_windows:
            cmd = "echo RETURN_CODE:$LASTEXITCODE\n"
        else:
            cmd = "echo RETURN_CODE:$?\n"

        print(cmd)
        self.process.stdin.write(cmd)
        self.process.stdin.flush()

    def read_output(self):
        output = []
        return_code = None

        if not self.process.stdout:
            raise RuntimeError("No stdout available to read.")

        for line in self.process.stdout:
            output.append(line)
            print(line, end="")

            if "$" in line:
                continue

            match = re.search(r"RETURN_CODE:(.*)", line)
            if match:
                return_code_raw = match.group(1).strip()  # Quitamos espacios
                if return_code_raw == "":
                    return_code = 0
                else:
                    try:
                        return_code = int(return_code_raw)
                    except ValueError:
                        raise RuntimeError(
                            f"Invalid RETURN_CODE value: '{return_code_raw}'"
                        )
                break

        return return_code, "".join(output)

    def close(self):
        self.send_command("exit")
        self.process.wait()
