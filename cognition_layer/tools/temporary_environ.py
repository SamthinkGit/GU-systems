import os
from contextlib import contextmanager


@contextmanager
def temporary_environ(new_env: dict[str, str]):
    old_env = {}

    for key, value in new_env.items():
        if key in os.environ:
            old_env[key] = os.environ[key]
        os.environ[key] = value

    try:
        yield
    finally:
        for key in new_env:
            if key in old_env:
                os.environ[key] = old_env[key]
            else:
                del os.environ[key]
