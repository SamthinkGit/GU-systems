from dataclasses import dataclass
from dataclasses import field
from typing import Any
from typing import Optional

from colorama import Fore
from colorama import Style

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


class ItemRegistry:

    _instances: dict[str, "ItemRegistry"] = {}

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
        self.actions: dict[str, Item] = {}
        self.tools: dict[str, Item] = {}
        self.objects: dict[str, Item] = {}

    def pretty_print(self) -> None:
        reg_color = Fore.YELLOW if self.name == "default" else ""
        print(f"{reg_color}{Style.BRIGHT}[{self.name}]{Style.BRIGHT}{Style.RESET_ALL}")

        for action in self.actions:
            print(f"-> {Fore.BLUE}(ACTION): {Fore.RESET}{action}")
        for tool in self.tools:
            print(f"-> {Fore.LIGHTGREEN_EX}(TOOL): {Fore.RESET}{tool}")
        for obj in self.objects:
            print(f"-> {Fore.LIGHTMAGENTA_EX}(OBJECT): {Fore.RESET}{obj}")

    @classmethod
    def summary(cls) -> None:
        print(pretty_head("Item Registry"))
        for reg in cls._instances.values():
            reg.pretty_print()


# registry = ItemRegistry(default=True)
# actions = registry.actions
# tools = registry.tools
# objects = registry.objects


if __name__ == "__main__":

    # test1
    registry = ItemRegistry()
    registry2 = ItemRegistry()
    assert id(registry) == id(registry2)

    super_registry = ItemRegistry(name="super-registry")
    assert id(super_registry) != id(registry2)

    registry.pretty_print()
    ItemRegistry.summary()
    print("Assertions completed done")
