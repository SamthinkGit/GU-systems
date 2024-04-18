"""
Registry Module
=================================

This module defines a singleton class `ItemRegistry` for registering and retrieving
functions and items by a unique identifier. It also includes `ItemEncoder` for encoding
and decoding items, supporting automatic encoding for non-JSON serializable objects.

Note: ItemRegistry saves all variables in a dictionary so BOTH sender and client must
be in the same memory space. Thus, the ItemEncoder must be shared in the same process.

If the encoding does only use functions (not object-variables) the ItemRegistry can be
used between distinct processes but the full compatibility is not ensured, and both
client and sender must have imported the function target.

[Update] It also contains TaskRegistry, focused on threading and task execution, managing individual
threads associated with specific tasks. It uses a singleton pattern to ensure a consistent
global registry of threads throughout the application lifecycle.  Key functionalities include
watching tasks, running tasks with specific identifiers, and waiting for task completion.
"""
import hashlib
import json
import threading
import uuid
from typing import Any
from typing import Callable
from typing import Dict
from typing import Hashable

from gusyscore.constants import ITEM_ENCODED_PREFIX
from gusyscore.core import get_logger


class ItemRegistry:
    """
    [SINGLETON] A registry for storing functions and items with unique identifiers.

    This class provides methods to add, retrieve, and invoke functions and items stored
    in the registry. It ensures that each function and item can be accessed through a
    unique hashable key generated based on the object itself.
    """

    _logger = get_logger("ItemRegistry")
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
        cls._logger.debug(f"Function {func} registered")
        return func

    @classmethod
    def add_function(cls, func: Callable) -> None:
        """Add an item to the registry with a unique ID."""
        cls._logger.debug(f"Function {func} has been manually added")
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

        obj_bytes = obj_repr.encode("utf-8")
        hash_object = hashlib.sha256(obj_bytes)
        return int(hash_object.hexdigest(), 16)

    @classmethod
    def call(cls, id: Hashable, *args, **kwargs) -> Any:
        """Call a registered function by its unique ID."""
        if id in cls._functions:
            return cls._functions[id](*args, **kwargs)
        else:
            raise KeyError(f"No function registered under ID {id}")


class ItemEncoder:
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


class ThreadRegistry:
    """
    [SINGLETON] Manages the registry of threads for different tasks, providing a centralized point of control for task
    execution threads.

    :param None: This class is implemented as a singleton and does not require parameters for instantiation.
    """

    _logger = get_logger("ThreadRegistry")
    _threads: Dict[Hashable, threading.Thread] = {}
    _threading_local_data = threading.local()
    _instance = None

    def __new__(cls):
        """
        Ensures that only one instance of ThreadRegistry is created (singleton pattern).
        :return: Returns the singleton instance of the ThreadRegistry.
        """
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def watch(self, task_id: str, target: Callable, **kwargs):
        """
        Starts a new thread associated with a task ID and executes the given target callable.

        :param task_id: Identifier for the task this thread is associated with.
        :param target: The callable object to be executed in the thread.
        :param kwargs: Additional keyword arguments to be passed to the target callable.
        """
        thread = threading.Thread(
            target=ThreadRegistry._run_with_id, args=(task_id, target, kwargs)
        )
        self._logger.debug(f"New thread built with task_id '{task_id}' for executing '{target.__name__}'")
        self._threads[task_id] = thread
        thread.start()

    def _run_with_id(task_id: str, func: Callable, kwargs):
        """
        Sets the task ID to the local thread data and executes the function with provided arguments.
        Used to wrap kwargs into the function

        :param task_id: Identifier for the task.
        :param func: Function to execute in the thread.
        :param kwargs: Arguments to pass to the function.
        """
        ThreadRegistry._threading_local_data.task_id = task_id
        func(**kwargs)

    def wait(self, task_id):
        """
        Waits for the specified task's thread to complete and removes it from the registry.
        :param task_id: Identifier for the task to wait for.
        """
        self._threads[task_id].join()
        self._logger.debug(f"Thread with task_id {task_id} has succesfully exited")
        del self._threads[task_id]

    @classmethod
    def get_task_id(cls):
        """
        Retrieves the task ID associated with the current thread from the thread-local data.
        :return: The task ID if available, otherwise None.
        """
        return getattr(cls._threading_local_data, "task_id", None)
