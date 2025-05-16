import time
from functools import wraps
from typing import Callable


def listable(func: Callable) -> Callable:
    def inner(*args, **kwargs):
        if len(args) == 1 and isinstance(args[0], list):
            return func(*args[0], **kwargs)
        return func(*args, **kwargs)

    return inner


def throttle(min_interval_seconds=15):
    def decorator(func):
        last_called = [0]

        @wraps(func)
        def wrapper(*args, **kwargs):
            now = time.time()
            elapsed = now - last_called[0]
            if elapsed < min_interval_seconds:
                raise RuntimeError(
                    "This action is being called too frequently."
                    "\nPlease select other action.",
                )
            last_called[0] = now
            return func(*args, **kwargs)

        return wrapper

    return decorator
