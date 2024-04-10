"""
Item and Function Registry Module
=================================

This module defines a singleton class `ItemRegistry` for registering and retrieving
functions and items by a unique identifier. It also includes `ItemEncoder` for encoding
and decoding items, supporting automatic encoding for non-JSON serializable objects.

Note: ItemRegistry saves all variables in a dictionary so BOTH sender and client must
be in the same memory space. Thus, the ItemEncoder must be shared in the same process.

If the encoding does only use functions (not object-variables) the ItemRegistry can be
used between distinct processes but the full compatibility is not ensured, and both
client and sender must have imported the function target.
"""
import hashlib
import json
import uuid
from typing import Any
from typing import Callable
from typing import Dict
from typing import Hashable

from gusyscore.constants import ITEM_ENCODED_PREFIX


class ItemRegistry:
    """
    A singleton registry for storing functions and items with unique identifiers.

    This class provides methods to add, retrieve, and invoke functions and items stored
    in the registry. It ensures that each function and item can be accessed through a
    unique hashable key generated based on the object itself.
    """

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
    """
    Provides static methods for encoding and decoding items to and from unique identifiers.

    This encoder is used to handle objects that are not natively serializable by JSON, such
    as complex Python objects. Encoded items are stored in the `ItemRegistry` until decoded,
    then they are freed from Registry.
    """

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
        """
        Encodes an item into a unique identifier and stores it in the registry only if
        the item is not JSON serializable

        :param item: The item to be encoded.
        :return: A unique identifier associated with the encoded item.
        """
        try:
            json.dumps(item)
            return item

        except TypeError:
            id = cls.encode(item)
            return ITEM_ENCODED_PREFIX + str(id)

    @classmethod
    def autodecode(cls, code: str):
        """
        Automatically encodes an item, using direct JSON serialization if possible,
        or encoding through `ItemEncoder` otherwise.

        :param item: The item to be decoded.
        :return: The original item if it's JSON serializable, or an encoded identifier string otherwise.
        """

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
