import time

from ecm.tools.registry import ItemRegistry


@ItemRegistry.register_function
def print_two_keywords(keyword_1, keyword_2):
    """MOCK"""
    print("My keyword_1 is", keyword_1)
    print("My keyword_2 is", keyword_2)


@ItemRegistry.register_function
def hello_world():
    """MOCK"""
    print("Hello World!")


@ItemRegistry.register_function
def append_to_list(list: list, value):
    """MOCK"""
    list.append(value)
    print(f"Appended {value} to the list")


@ItemRegistry.register_function
def sleep_and_print(text: str):
    """MOCK"""
    time.sleep(1)
    print(text)


@ItemRegistry.register_function
def test_function(num: int, sleep: int = 0):
    """MOCK"""
    time.sleep(1)
    print("Succesfully executed function with argument: ", num)
    time.sleep(sleep)


@ItemRegistry.register_function
def empty_function():
    """MOCK"""
    return
