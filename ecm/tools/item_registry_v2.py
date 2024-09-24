from collections import defaultdict
from dataclasses import dataclass
from dataclasses import field
from typing import Any
from typing import Callable
from typing import Literal
from typing import Optional

from colorama import Fore
from colorama import Style

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

    def pretty_print(self):
        print(f"* {Fore.LIGHTMAGENTA_EX}(ITEM): {Fore.RESET}{self}")


@dataclass
class Action(Item):
    name: str
    content: Callable
    description: str
    is_callable: bool = True

    def pretty_print(self):
        print(f"* {Fore.BLUE}(ACTION): {Fore.RESET}{self}")


class Tool(Item):
    name: str
    content: Callable
    is_callable: bool = True

    def pretty_print(self):
        print(f"* {Fore.LIGHTGREEN_EX}(TOOL): {Fore.RESET}{self}")


class ItemRegistry:

    _logger = get_logger("ItemRegistryV2")
    _instances: dict[str, "ItemRegistry"] = {}
    _packages: dict[str, list[Item]] = defaultdict(list)
    _global_items: dict[str, Item] = {}

    def __new__(cls, name: str = "default"):

        if len(cls._instances) <= 0:
            cls._instances["default"] = super().__new__(cls)

        if name == "default":
            return ItemRegistry._instances["default"]

        registry = super().__new__(cls)
        cls._instances[name] = registry
        return registry

    def __init__(self, name: str = "default"):

        self.name: str = name
        self.actions: dict[str, Action] = {}
        self.tools: dict[str, Tool] = {}
        self.items: dict[str, Item] = {}

    def pretty_print(self) -> None:
        reg_color = Fore.YELLOW if self.name == "default" else ""
        print(f"{reg_color}{Style.BRIGHT}[{self.name}]{Style.BRIGHT}{Style.RESET_ALL}")

        for item in (
            list(self.actions.values())
            + list(self.tools.values())
            + list(self.items.values())
        ):
            item.pretty_print()

    def _load_to_correspondent(self, obj: Action | Tool | Item) -> None:
        if not issubclass(obj.__class__, Item):
            raise ValueError("Invalid object class passed to load.")

        if isinstance(obj, Action):
            self.actions[obj.name] = obj
            self._logger.debug(f"Action {obj.name} registered.")
        elif isinstance(obj, Tool):
            self.tools[obj.name] = obj
            self._logger.debug(f"Tool {obj.name} registered.")
        else:
            self._logger.debug(f"Item {obj.name} registered.")
            self.items[obj.name] = obj

    def load_package(self, package_name: str) -> None:
        if package_name not in ItemRegistry._packages.keys():
            raise ValueError(f"{package_name} can't be found in ItemRegistry packages.")

        for item in ItemRegistry._packages[package_name]:
            self._load_to_correspondent(item)

    def load(self, item_name: str):
        self._load_to_correspondent(ItemRegistry._global_items[item_name])

    def load_all(self):
        for pkg_name in ItemRegistry._packages.keys():
            self.load_package(pkg_name)
        for item in ItemRegistry._global_items.values():
            self._load_to_correspondent(item)

    @classmethod
    def summary(cls, full: bool = False) -> None:
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
            reg.pretty_print()

    @classmethod
    def register(
        cls,
        type: Literal["action", "tool"] = "action",
        package: Optional[str] = None,
        mock: bool = False,
    ):

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

            if package is not None:
                cls._packages[package].append(item)
            else:
                cls._global_items[name] = item
            cls._update_v1_compatibility()
            return func

        return wrapper

    # Compatibility attributes for ItemRegistryV1
    @property
    def _functions(self):
        raise SystemError(
            "ItemRegistryV2 does not implement a hash to function method."
        )

    @property
    def _items(self):
        raise SystemError(
            "ItemRegistryV2 does not implement a hash to function method."
        )

    @classmethod
    def _update_v1_compatibility(cls):

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

    @classmethod
    def register_util(cls, func):
        return ItemRegistry.register(type="tool")(func)

    @classmethod
    def register_function(cls, func):
        return ItemRegistry.register(type="action")(func)
