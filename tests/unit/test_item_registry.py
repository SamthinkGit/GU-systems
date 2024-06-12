import pytest

from ecm.registry import ItemRegistry


def test_item_registry(capsys: pytest.CaptureFixture):
    registry = ItemRegistry()

    print("----- Testing ItemRegistry -----")

    @ItemRegistry.register_function
    def myfunc(message):
        print("Hello Item Registry!")
        print(f"I will print a message: {message}")

    id = ItemRegistry.get_id(myfunc)
    message = "Hi! I'm a message."
    registry.call(id, message=message)

    capture = capsys.readouterr()
    assert message in capture.out


def test_obtain_function():
    registry = ItemRegistry()

    @ItemRegistry.register_function
    def expected_func(): ...

    assert registry.get_from_name("expected_func") == expected_func
