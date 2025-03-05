from typing import Callable


def listable(func: Callable) -> Callable:
    def inner(*args, **kwargs):
        if len(args) == 1 and isinstance(args[0], list):
            return func(*args[0], **kwargs)
        return func(*args, **kwargs)

    return inner
