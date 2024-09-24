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

[HELP] This module is where all the actions and items are saved serving as communicator between
different layers.
"""
import functools
import hashlib
import json
import threading
import types
import uuid
from typing import Any
from typing import Callable
from typing import Dict
from typing import Hashable

from ecm.constants import ITEM_ENCODED_PREFIX
from ecm.remote.server import EcmServer
from ecm.shared import _MOCKS_ENABLED
from ecm.shared import get_logger


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
    _utils: Dict[str, Callable] = {}
    _items: Dict[Hashable, Any] = {}
    _names: Dict[str, Callable] = {}

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    @classmethod
    def get_function(cls, key: Hashable) -> Callable:
        """Retrieve a registered function by its unique key."""
        func = cls._functions.get(key, None)
        if func is None:
            raise KeyError(
                f"The key received `{key}` does not correspond to a registered function."
                "Please ensure you have used the decorator @register_function first"
            )
        return func

    @classmethod
    def get_item(cls, key: Hashable) -> Any:
        """Retrieve a registered item by its unique key."""
        item = cls._items.get(key, None)
        if item is None:
            raise KeyError(
                f"The key received `{key}` does not correspond to any saved item."
                "Please ensure you have used add_item() befor using this function."
                "If you are using ItemEncoder ensure you have used encoder.autoencode(item)"
            )
        return item

    @classmethod
    def register_function(cls, func: Callable, **kwargs):
        """Decorator to register a function with a unique action ID."""
        # We maintain kwargs for esaing compatibility with ItemRegistryV2
        if not _MOCKS_ENABLED.status and func.__doc__ == "MOCK":
            return func
        id = cls.get_id(func)
        cls._functions[id] = func
        cls._names[func.__name__] = func
        cls._logger.debug(f"Function {func} registered")
        return func

    @classmethod
    def register_util(cls, func: Callable, **kwargs):
        """Decorator to register a util with a unique action ID. This utils will not be
        notified to the AI agents as valid actions."""
        # We maintain kwargs for esaing compatibility with ItemRegistryV2
        if not _MOCKS_ENABLED.status and func.__doc__ == "MOCK":
            return func
        name = func.__name__
        cls._utils[name] = func
        cls._logger.debug(f"Util {func} registered")
        return func

    @classmethod
    def alias(cls, names: list[str]):
        """Decorator to register a function with different names."""

        def wrapper(func: Callable):

            new_funcs = [cls._clone_function(func) for _ in names]
            for idx, name in enumerate(names):

                new_funcs[idx].__name__ = name
                cls.register_function(new_funcs[idx])

            return func

        return wrapper

    @classmethod
    def _clone_function(cls, func: Callable):
        new_func = types.FunctionType(
            func.__code__,
            func.__globals__,
            name=func.__name__,
            argdefs=func.__defaults__,
            closure=func.__closure__,
        )
        new_func.__dict__.update(func.__dict__)
        new_func.__annotations__ = func.__annotations__
        new_func.__qualname__ = func.__qualname__
        return new_func

    @classmethod
    def add_function(cls, func: Callable) -> None:
        """Add an item to the registry with a unique ID."""
        cls._logger.debug(f"Function {func} has been manually added")
        id = cls.get_id(func)
        if id not in cls._functions:
            cls._functions[id] = func
            cls._names[func.__name__] = func

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

    @classmethod
    def get_from_name(cls, name: str) -> Callable | None:
        """Returns a registered function by its name"""
        result = cls._names.get(name)
        if result is None:
            raise KeyError(
                f"The name received `{name}` does not correspond to any saved item."
                "Please ensure you have used add_item() or @ItemRegistry.register_function "
                "before using this function."
            )
        return result

    @classmethod
    def invalidate_all_functions(cls) -> None:
        """Changes all registered functions into fake ones. Used for safe playing of actions."""
        cls._logger.warning(
            "All functions have been invalidated. Fake functions will be run instead (Safe Use)"
        )
        for name, func in cls._names.items():
            key = cls.get_id(func)
            fake = functools.wraps(func)(
                lambda *args, name=name, **kwargs: print(
                    f"{name}(args: {args}, kwargs: {kwargs})"
                )
            )
            cls._utils[name] = fake
            cls._functions[key] = fake

    @classmethod
    def transfer_execution_to_client(cls):
        """Changes all registered functions into the same functions executed on the clients connected
        to the ECM server. Note that those functions must be already imported in the ItemRegirsty of
        the client."""
        cls._logger.warning(
            "All functions will be executed on the clients connected to the ECM Server."
        )

        for name, func in cls._names.items():
            key = cls.get_id(func)
            remote_function = functools.wraps(func)(
                lambda *args, name=name, **kwargs: EcmServer.send_task(
                    name, *args, **kwargs
                )
            )
            cls._names[name] = remote_function
            cls._functions[key] = remote_function

        for name, func in cls._utils.items():
            remote_function = functools.wraps(func)(
                lambda *args, name=name, **kwargs: EcmServer.send_task(
                    name, *args, **kwargs
                )
            )
            cls._utils[name] = remote_function


class ItemEncoder:
    """
    Provides static methods for encoding and decoding items to and from unique identifiers.

    This encoder is used to handle objects that are not natively serializable by JSON, such
    as complex Python objects. Encoded items are stored in the `ItemRegistry` until decoded,
    then they are freed from Registry.
    """

    @staticmethod
    def encode(item: Any) -> Hashable:
        """
        Encodes an item into a unique identifier and stores it in the registry.
        :warning: The Item can collide with SequencePackages if it is not serializable
        :param item: The item to be encoded.
        :return: A unique identifier associated with the encoded item.
        """
        id = uuid.uuid4()
        ItemRegistry._force_add(id, item)
        return id

    @staticmethod
    def decode(id: Hashable) -> Any:
        """
        Decodes an item, given a previous ItemEncoder hash of the item

        :param item: The item to be decoded.
        :return: The original item
        """
        item = ItemRegistry.get_item(id)
        del ItemRegistry._items[id]
        return item

    # ----------------------- RECOMMENDED METHODS -----------------------------
    @classmethod
    def autoencode(cls, item: Any):
        """
        Encodes an item into a unique identifier and stores it in the registry only if
        the item is not JSON serializable

        :param item: The item to be encoded.
        :return: A unique identifier associated with the encoded item.
        """
        try:
            if isinstance(item, list) or isinstance(item, dict):
                raise TypeError

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

    # ---------------------------------------------------------------------------


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

    def watch(self, task_id: str, target: Callable, **kwargs) -> None:
        """
        Executes the target function with a thread whose task_id will be the given in the
        arguments. When returning,

        :param task_id: Identifier for the task this thread is associated with.
        :param target: The callable object to be executed in the thread.
        :param kwargs: Additional keyword arguments to be passed to the target callable.
        """
        thread = threading.Thread(
            target=ThreadRegistry._run_with_id, args=(task_id, target, kwargs)
        )

        # Save some data to be accesible outwards if necessary
        thread._target_func = target  # type: ignore
        thread._task_id = task_id  # type: ignore

        if "seq_type" in kwargs:
            thread._seq_type = kwargs["seq_type"]  # type: ignore

        self._logger.debug(f"New thread built with task_id '{task_id}'")

        thread.start()
        self._threads[task_id] = thread

    @staticmethod
    def _run_with_id(task_id: str, func: Callable, kwargs) -> None:
        """
        Sets the task ID to the local thread data and executes the function with provided arguments.
        Used to wrap kwargs into the function

        :param task_id: Identifier for the task.
        :param func: Function to execute in the thread.
        :param kwargs: Arguments to pass to the function.
        """
        ThreadRegistry._threading_local_data.task_id = task_id
        ThreadRegistry._threading_local_data.target_func = func
        func(**kwargs)

    def get_thread(self, task_id: str) -> threading.Thread | None:
        """Returns the thread with the task_id requested"""
        return self._threads.get(task_id, None)

    def pop_thread(self, task_id: str) -> threading.Thread:
        """Pops (removes from the registry) the thread with the task_id requested"""
        return self._threads.pop(task_id)

    def wait(self, task_id):
        """
        Waits for the specified task's thread to complete and removes it from the registry.
        If there is no thread, it returns inmediatly
        :param task_id: Identifier for the task to wait for.
        """
        thread = self._threads.get(task_id, None)
        if thread is not None:
            thread.join()
            self._logger.debug(f"Thread with task_id {task_id} has succesfully exited")
            del self._threads[task_id]

    @classmethod
    def get_task_id(cls):
        """
        Retrieves the task ID associated with the current thread from the thread-local data.
        :return: The task ID if available, otherwise None.
        """
        return getattr(cls._threading_local_data, "task_id", None)
