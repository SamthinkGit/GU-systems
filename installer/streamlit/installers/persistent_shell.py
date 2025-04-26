import locale
import os
import re
import subprocess

import streamlit as st


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
        self.process.stdin.flush()

        if self.is_windows:
            self.process.stdin.write("echo RETURN_CODE:$LASTEXITCODE\n")
        else:
            self.process.stdin.write("echo RETURN_CODE:$?\n")
        self.process.stdin.flush()

    def read_output(self, write_output_with_st: bool = True):
        output = []
        return_code = None

        if not self.process.stdout:
            raise RuntimeError("No stdout available to read.")

        for line in self.process.stdout:
            if write_output_with_st:
                st.write(line)

            output.append(line)

            match = re.search(r"RETURN_CODE:(\d+)", line)
            if match:
                return_code = int(match.group(1))
                break

        return return_code, "".join(output)

    def close(self):
        self.send_command("exit")
        self.process.wait()
