"""
Item Registry Module
==============================

This module is designed to manage a registry of items, actions, and tools
within a structured environment. It provides the functionality to create,
load, and manage various components that can be called or executed.
The `ItemRegistry` class serves as the core of this module, allowing users
to register and utilize items effectively while keeping track of their state
and properties.

The module leverages the `dataclass` decorator to define item types,
such as `Item`, `Action`, and `Tool`, providing a clear and concise way
to manage the attributes associated with each type. Additionally, it
provides mechanisms for pretty-printing item details and handling
package loading.
"""
import functools
import types
from collections import defaultdict
from collections import UserDict
from copy import deepcopy
from dataclasses import dataclass
from dataclasses import field
from itertools import chain
from typing import Any
from typing import Callable
from typing import Literal
from typing import Optional

from colorama import Fore
from colorama import Style

from action_space.tools.loader import discover_packages
from action_space.tools.loader import load_package as import_pkg
from ecm.shared import _MOCKS_ENABLED
from ecm.shared import get_logger
from ecm.tools.prettify import pretty_head


@dataclass
class Item:

    # Core properties
    name: str = ""
    active: bool = False
    content: Optional[Any] = None

    # Additional properties
    is_callable: bool = False
    description: str = ""
    labels: list[str] = field(default_factory=list)

    def pretty_print(self, full: bool = True):
        label = self if full else f"{self.labels} {self.name} -> {self.content}"
        print(f"* {Fore.LIGHTMAGENTA_EX}(ITEM): {Fore.RESET}{label}")


@dataclass
class Action(Item):
    name: str
    content: Callable
    description: str
    is_callable: bool = True

    def pretty_print(self, full: bool = True):
        label = self if full else f"{self.labels} {self.name} -> {self.content}"
        print(f"* {Fore.BLUE}(ACTION): {Fore.RESET}{label}")


class Tool(Item):
    name: str
    content: Callable
    is_callable: bool = True

    def pretty_print(self, full: bool = True):
        label = self if full else f"{self.labels} {self.name} -> {self.content}"
        print(f"* {Fore.LIGHTGREEN_EX}(TOOL): {Fore.RESET}{label}")


class ItemRegistry:

    _logger = get_logger("ItemRegistryV2")
    _instances: dict[str, "ItemRegistry"] = {}
    _packages: dict[str, list[Item]] = defaultdict(list)
    _global_items: dict[str, Item] = {}
    _instance_initialization: bool = True

    def __new__(cls, name: str = "default"):

        if len(cls._instances) <= 0:
            cls._instances["default"] = super().__new__(cls)

        if name in ItemRegistry._instances.keys():
            return ItemRegistry._instances[name]

        registry = super().__new__(cls)
        cls._instances[name] = registry
        return registry

    def __init__(self, name: str = "default"):
        if name != "default":
            self._logger = get_logger(name)

        if self._instance_initialization:
            self.name: str = name
            self.actions: dict[str, Action] = {}
            self.tools: dict[str, Tool] = {}
            self.items: dict[str, Item] = {}
            self._instance_initialization = False

    def pretty_print(self, full: bool = True) -> None:
        reg_color = Fore.YELLOW if self.name == "default" else ""
        print(f"{reg_color}{Style.BRIGHT}[{self.name}]{Style.BRIGHT}{Style.RESET_ALL}")

        for item in (
            list(self.actions.values())
            + list(self.tools.values())
            + list(self.items.values())
        ):
            item.pretty_print(full)

    def _load_to_correspondent(
        self, obj: Action | Tool | Item, silent: bool = False
    ) -> None:
        if not issubclass(obj.__class__, Item):
            raise ValueError("Invalid object class passed to load.")

        if isinstance(obj, Action):
            self.actions[obj.name] = obj
            if not silent:
                self._logger.debug(f"Action {obj.name} registered.")
        elif isinstance(obj, Tool):
            self.tools[obj.name] = obj
            if not silent:
                self._logger.debug(f"Tool {obj.name} registered.")
        else:
            self._logger.debug(f"Item {obj.name} registered.")
            if not silent:
                self.items[obj.name] = obj
        obj.active = True

    def load_package(self, package_name: str) -> None:
        """
        Loads all items from a specified package into the registry. It ensures
        that the package exists in the registry and calls the appropriate
        loading methods for each item.

        :param package_name: The name of the package to load items from.
        """

        if package_name not in ItemRegistry._packages.keys():
            raise ValueError(f"{package_name} can't be found in ItemRegistry packages.")

        for item in ItemRegistry._packages[package_name]:
            self._load_to_correspondent(item)

    def load(self, item_name: str):
        """
        Loads a specific item by its name from the global items into the
        registry. This allows for easy access and management of registered
        items.

        :param item_name: The name of the item to load.
        """
        self._load_to_correspondent(ItemRegistry._global_items[item_name])

    def autoload(
        self,
        identifier: Optional[str] = None,
    ):
        """
        Imports and loads a package or item based on the provided identifier.
        If the identifier contains a "/", it is treated as a package name.
        Otherwise, it is treated as an action notation. The method
        dynamically imports the package and loads its items into the
        registry.
        :param identifier: The identifier for the package or item to load.
        :return: None
        """
        if "/" not in identifier:
            # This is a package
            import_pkg(identifier)
            self.load_package(identifier)

        else:
            # Using action notation
            packages = discover_packages()
            import_pkg(identifier)
            for pkg in packages:
                if pkg["notation"] == identifier:
                    pkg_name = pkg["pkg_name"]
                    break
            self.load_package(pkg_name)

    def load_all(self):
        """
        Loads all registered packages and global items into the registry
        It iterates through the registered packages, loading their items
        sequentially, ensuring that all items are available for use.
        """

        for pkg_name in ItemRegistry._packages.keys():
            self.load_package(pkg_name)
        for item in ItemRegistry._global_items.values():
            self._load_to_correspondent(item)

    def invalidate(self, actions: bool = True, tools: bool = True):
        """
        Converts all registered functions into fake ones for safe testing
        purposes. This method allows users to simulate actions and tools
        without executing their real implementations.

        :param actions: If True, all actions will be invalidated.
        :param tools: If True, all tools will be invalidated.
        :return: None
        """
        self._logger.warning(
            f"Multiple functions at registry `{self.name}` have been invalidated. "
            "Fake functions will be run instead (Safe Use)"
        )
        items_to_invalidate: list[Item] = []
        if actions:
            items_to_invalidate.extend(list(self.actions.values()))
        if tools:
            items_to_invalidate.extend(list(self.tools.values()))

        new_items = []
        for item in items_to_invalidate:
            fake_func = functools.wraps(item.content)(
                lambda *args, name=item.name, **kwargs: print(
                    f"{name}(args: {args}, kwargs: {kwargs})"
                )
            )
            cp = deepcopy(item)
            cp.content = fake_func
            cp.labels.append("Invalidated")
            new_items.append(cp)

        if actions:
            self.actions = {}
        if tools:
            self.tools = {}

        for item in new_items:
            self._load_to_correspondent(item, silent=True)

    def get(
        self,
        name: str,
        type: Literal["tool", "action", "item", "any"] = "any",
        return_multiple: bool = False,
    ) -> Tool | Item | Action:
        """
        Used for simplifiying the retrieval of entities from the registry.

        Retrieves an item from the registry based on its name and type.
        It searches through the registered actions, tools, and items,
        returning the first match found. If multiple matches are found,
        it can return a list of all matching items.

        :param name: The name of the item to retrieve.
        :param type: The type of item to retrieve ("tool", "action", "item", "all").
        :param return_multiple: If True, returns a list of all matching items.
        :return: The matching item or a list of matching items.
        """
        result_list = []

        for item in chain(
            self.actions.values(), self.tools.values(), self.items.values()
        ):
            result = None

            if name.lower() in item.name.lower():
                if (
                    type == "any"
                    or (type == "item" and isinstance(item, Item))
                    or (type == "tool" and isinstance(item, Tool))
                    or (type == "action" and isinstance(item, Action))
                ):
                    result = item

            if result is not None:
                result_list.append(result)

        if not return_multiple and len(result_list) > 1:
            options = ", ".join([f"'{item.name}'" for item in result_list])
            self._logger.warning(
                f"Multiple items found for name '{name}':{options} Returning the first match."
            )

        if len(result_list) == 0:
            raise ValueError(
                f"Item '{name}' not found in registry '{self.name}'. "
                f"Available items: {', '.join(self.items.keys())}"
            )

        if not return_multiple:
            return result_list[0]

        return result_list

    @classmethod
    def summary(cls, full: bool = False) -> None:
        """
        Prints a summary of the current state of the item registry, including
        detected packages, unlocalized items, and instances. If the full
        flag is set to True, it provides a more detailed overview.

        :param full: A flag to specify whether to display full details.
        :return: None
        """
        print(pretty_head("Item Registry"))

        if full:
            print(Style.BRIGHT + "\nDetected Packages: " + Style.RESET_ALL)
            for pkg, items in cls._packages.items():
                print(f"[{pkg}]: " + "{")
                for item in items:
                    print("  ", end="")
                    item.pretty_print()
                print("}")

            print(Style.BRIGHT + "\nUnlocalized Items: " + Style.RESET_ALL)
            for item in cls._global_items.values():
                item.pretty_print()

            print(Style.BRIGHT + "\nInstances:" + Style.RESET_ALL)

        for reg in cls._instances.values():
            reg.pretty_print(full)

    @classmethod
    def flush(cls):
        cls._instances = {}
        cls._packages = defaultdict(list)
        cls._global_items = {}

    @staticmethod
    def require_dependencies(*packages: str, storage_key: str = "default") -> Callable:
        def decorator(func: Callable) -> Callable:
            def wrapper(*args, **kwargs):
                registry = ItemRegistry(storage_key)
                missing_packages = [
                    pkg for pkg in packages if pkg not in registry._packages.keys()
                ]
                if len(missing_packages) > 0:
                    raise ImportError(
                        f"Missing required packages: {', '.join(missing_packages)}"
                    )

                return func(*args, **kwargs)

            return wrapper

        return decorator

    @classmethod
    def register(
        cls,
        type: Literal["action", "tool"] = "action",
        package: Optional[str] = None,
        labels: Optional[list[str]] = None,
        mock: bool = False,
    ):
        """
        This method registers a function as either an action or a tool within
        the item registry. It wraps the function and stores it along with its
        metadata. If mocks are enabled, it will handle them accordingly.

        :param type: Specifies the type of registration ("action" or "tool").
        :param package: The optional package name to associate with the item.
        :param mock: A flag to determine if the function should be treated
                    as a mock.
        :return: The original function (idempotent).
        """

        valid_types = ["action", "tool"]
        if type not in valid_types:
            raise ValueError(
                f"Type {type} is not valid for this function. Please use one of the following: {valid_types}"
            )

        def wrapper(func: Callable) -> Callable:
            if not _MOCKS_ENABLED.status and (func.__doc__ == "MOCK" or mock):
                return func
            name = (
                f"{package}.{func.__name__}" if package is not None else func.__name__
            )

            if type == "action":
                item = Action(
                    name=name,
                    content=func,
                    description=func.__doc__,
                    is_callable=True,
                )
            if type == "tool":
                item = Tool(
                    name=name,
                    content=func,
                    is_callable=True,
                )

            if labels is not None:
                item.labels = labels

            if package is not None:
                cls._packages[package].append(item)
            else:
                cls._global_items[name] = item
            cls._update_v1_compatibility()
            return func

        return wrapper

    # Compatibility attributes for ItemRegistryV1
    @classmethod
    def _update_v1_compatibility(cls):
        """
        Updates the compatibility layer for ItemRegistryV1 by creating
        mappings of tools and actions from the global items. This method
        ensures that the registry maintains backward compatibility.
        """

        # TODO: Speed up this function

        cls._utils = {
            name: tool.content
            for name, tool in ItemRegistry._global_items.items()
            if isinstance(tool, Tool)
        }
        cls._names = {
            name: action.content
            for name, action in ItemRegistry._global_items.items()
            if isinstance(action, Action)
        }
        cls._functions = cls._names

    @classmethod
    def transfer_execution_to_client(cls):
        raise SystemError(
            "This function has been deprecated, use EcmServer.wrap_item_registry() "
            "instead. It can be imported from ecm.remote.server"
        )

    @classmethod
    def register_util(cls, func):
        return ItemRegistry.register(type="tool")(func)

    @classmethod
    def register_function(cls, func):
        return ItemRegistry.register(type="action")(func)

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
        new_func.__doc__ = func.__doc__
        new_func.__annotations__ = func.__annotations__
        new_func.__qualname__ = func.__qualname__

        doc = getattr(func, "__doc__", None)
        if doc is not None:
            new_func.__doc__ = doc

        return new_func

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


class Storage(UserDict):

    _instance_initialization: bool = True
    _instances: dict[str, "Storage"] = {}

    def __new__(cls, name: str = "default"):

        if len(cls._instances) <= 0:
            cls._instances["default"] = super().__new__(cls)

        if name in Storage._instances.keys():
            return Storage._instances[name]

        storage = super().__new__(cls)
        cls._instances[name] = storage
        return storage

    def __init__(self, name: str = "default"):
        if self._instance_initialization:
            super().__init__()
            self.name: str = name
            self._instance_initialization = False


ItemRegistry()
