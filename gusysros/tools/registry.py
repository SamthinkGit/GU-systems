import hashlib
import json
import uuid
from typing import Any
from typing import Callable
from typing import Dict
from typing import Hashable

from gusyscore.constants import ITEM_ENCODED_PREFIX


class ItemRegistry:

    _instance = None
    _functions: Dict[Hashable, Callable] = {}
    _items: Dict[Hashable, Any] = {}

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    @classmethod
    def get_function(cls, key: Hashable) -> Callable:
        """Retrieve a registered function by its unique key."""
        return cls._functions[key]

    @classmethod
    def get_item(cls, key: Hashable) -> Any:
        """Retrieve a registered item by its unique key."""
        return cls._items[key]

    @classmethod
    def register_function(cls, func: Callable):
        """Decorator to register a function with a unique action ID."""
        id = cls.get_id(func)
        cls._functions[id] = func
        return func

    @classmethod
    def add_function(cls, func: Callable) -> None:
        """Add an item to the registry with a unique ID."""
        id = cls.get_id(func)
        if id not in cls._functions:
            cls._functions[id] = func

    @classmethod
    def add_item(cls, item: Any) -> None:
        """Add an item to the registry with a unique ID."""
        cls._items[cls.get_id(item)] = item

    @classmethod
    def _force_add(cls, key: Hashable, item: Any) -> None:
        cls._items[key] = item

    @staticmethod
    def get_id(obj: Any) -> Hashable:
        """Generate a unique ID for the given object."""
        if callable(obj):
            obj_repr = f"{obj.__qualname__}.{obj.__name__}"
        else:
            obj_repr = str(obj)

        obj_bytes = obj_repr.encode('utf-8')
        hash_object = hashlib.sha256(obj_bytes)
        return int(hash_object.hexdigest(), 16)

    @classmethod
    def call(cls, id: Hashable, *args, **kwargs) -> Any:
        """Call a registered function by its unique ID."""
        if id in cls._functions:
            return cls._functions[id](*args, **kwargs)
        else:
            raise KeyError(f"No function registered under ID {id}")


class ItemEncoder():

    @staticmethod
    def encode(item: Any) -> Hashable:
        id = uuid.uuid4()
        ItemRegistry._force_add(id, item)
        return id

    @staticmethod
    def decode(id: Hashable) -> Any:
        item = ItemRegistry.get_item(id)
        del ItemRegistry._items[id]
        return item

    @classmethod
    def autoencode(cls, item: Any):
        try:
            json.dumps(item)
            return item

        except TypeError:
            id = cls.encode(item)
            return ITEM_ENCODED_PREFIX + str(id)

    @classmethod
    def autodecode(cls, code: str):

        if not isinstance(code, str):
            return code

        if code.startswith(ITEM_ENCODED_PREFIX):
            id = uuid.UUID(code[len(ITEM_ENCODED_PREFIX):])
            return cls.decode(id)

        return code


if __name__ == '__main__':

    print("----- Testing ItemRegistry -----")
    registry = ItemRegistry()

    @ItemRegistry.register_function
    def myfunc(message):
        print("Hello Item Registry!")
        print(f"I will print a message: {message}")

    id = ItemRegistry.get_id(myfunc)
    print(f"The new function myfunc has id: {id}")
    print("Executing action...")

    registry.call(id, message="Hi! It looks like it is working ^^")

    print("\n\n----- Testing Encoder -----")
    num = 4
    item = object()

    num_code = ItemEncoder.autoencode(num)
    item_code = ItemEncoder.autoencode(item)

    print(f"An integer autoencoded is: {num_code}")
    print(f"The object autoencoded is: {item_code}")

    num = ItemEncoder.autodecode(num_code)
    item = ItemEncoder.autodecode(item_code)

    print(f"Decodification of num: {num}")
    print(f"Decodification of object: {item}")
