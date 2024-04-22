from gusyscore.core import get_logger
from gusyscore.gateway.mocks.debug import append_to_list
from gusysros.tools.packages import ActionPackage
from gusysros.tools.packages import SequencePackage
from gusysros.tools.packages import SequencePriority
from gusysros.tools.registry import ItemRegistry
from gusysros.types.basic import SimpleSequence


class PackageMock:

    def __init__(self) -> None:
        self._logger = get_logger("MOCK")
        self.target_list = []
        self.priority = SequencePriority.NORMAL
        self.type = SimpleSequence.get_type()
        self.task_id = "debug"

        action_1 = ActionPackage(
            action_id=ItemRegistry.get_id(append_to_list),
            list=self.target_list,
            value="A",
        )
        action_2 = ActionPackage(
            action_id=ItemRegistry.get_id(append_to_list),
            list=self.target_list,
            value="B",
        )

        self.pkg = SequencePackage(
            task_id=self.task_id,
            type=self.type,
            priority=self.priority,
            actions=[action_1, action_2],
        )

    def get_package(self):
        return self.pkg

    def get_target(self):
        return self.target_list

    def list_properly_modified(self):
        if self.target_list != ["A", "B"]:
            self._logger.error(f"Expected {['A', 'B']} and {self.target_list} was obtained")
            return False
        return True
