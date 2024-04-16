from typing import Dict
from typing import Type

from gusysros.tools.packages import ActionPackage
from gusysros.tools.packages import SequencePackage
from gusysros.tools.packages import SequencePriority
from gusysros.tools.registry import ItemRegistry


class SequenceType:
    type_code = None
    _registry: Dict[int, Type["SequenceType"]] = {}

    def __init__(self, pkg: SequencePackage) -> None:
        if self.__class__ == SequenceType:
            raise TypeError(
                "SequenceType cannot be instantiated directly, please use a subclass"
            )

        assert (
            pkg.type == self.type_code
        ), "Package type does not match the expected type"

        self.pkg = pkg

    @classmethod
    def register_type(
        cls, type_code: int, type_class: Type["SequenceType"], force: bool = False
    ):

        if type_code in cls._registry:

            if not force and cls._registry[type_code] == type_class:
                raise TypeError(
                    f"SequenceType with type_code {type_code} has already been registered"
                )

        cls._registry[type_code] = type_class

    @classmethod
    def auto_register_type(cls, type: Type["SequenceType"]):
        cls.register_type(
            type_code=type.get_type(),
            type_class=type
        )

    @classmethod
    def from_pkg(cls, pkg: SequencePackage) -> "SequenceType":

        type_class = cls._registry.get(pkg.type)
        if type_class is None:
            raise ValueError(f"No registered class for package type: {pkg.type}")
        return type_class(pkg)

    @classmethod
    def get_type(cls):
        return cls.type_code

    def run(self):
        raise NotImplementedError("SequenceType cannot be instantiated directly, please use a subclass")


class SimpleSequence(SequenceType):

    type_code = 5

    def __init__(self, pkg: SequencePackage) -> None:
        super().__init__(pkg)

    def run(self):
        for action in self.pkg.actions:
            func = ItemRegistry.get_function(action.action_id)
            args = action.args
            kwargs = action.kwargs
            func(*args, **kwargs)


if __name__ == '__main__':

    @ItemRegistry.register_function
    def my_function(keyword_1, keyword_2):
        print("My keyword_1 is", keyword_1)
        print("My keyword_2 is", keyword_2)

    # Every Type has to be registered in order to be accesible
    SequenceType.auto_register_type(SimpleSequence)

    action = ActionPackage(
        ItemRegistry.get_id(my_function),
        **{
            'keyword_1': '1',
            'keyword_2': '2'
        }
    )
    seq_pkg = SequencePackage(
        task_id="my_task",
        type=SimpleSequence.get_type(),
        priority=SequencePriority.NORMAL,
        actions=[action]*3
    )

    stype = SequenceType.from_pkg(seq_pkg)
    stype.run()
