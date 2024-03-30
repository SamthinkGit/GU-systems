import json
from enum import Enum

import gusyscore.gateway.mocks.gateway as gateway
from gusyscore.ros2.tools.registry import ItemEncoder
from gusyscore.ros2.tools.registry import ItemRegistry


class ActionType(Enum):
    SIMPLE_SEQUENCE = 1
    PARALLEL_SEQUENCE = 2


class ActionPackage():
    def __init__(self, type: ActionType, action_id: int, *args, **kwargs) -> None:
        self.type = type
        self.action_id = action_id
        self.args = args
        self.kwargs = kwargs

    def to_dict(self):
        return {
            'type': self.type.value,
            'action_id': self.action_id,
            'args': [ItemEncoder.autoencode(arg) for arg in self.args],
            'kwargs': {key: ItemEncoder.autoencode(val) for key, val in self.kwargs.items()}
        }


class SequencePackage():
    def __init__(self, id: str, priority: int, actions: list[ActionPackage]) -> None:

        self.id = id
        self.priority = priority
        self.actions = actions

    def to_dict(self):
        return {
            'id': self.id,
            'priority': self.priority,
            'actions': [action.to_dict() for action in self.actions]
        }

    def to_json(self):
        return json.dumps(self.to_dict(), indent=4)


if __name__ == '__main__':
    with gateway.OneFileWorkspaceMock(temporal=True) as ws:

        action_1 = ActionPackage(
            type=ActionType.SIMPLE_SEQUENCE,
            action_id=ItemRegistry.get_id(gateway.OneFileWorkspaceMock.write_with),
            **{
                'workspace': ws,
                'text': 'Hello World!'
            }
        )

        action_2 = ActionPackage(
            type=ActionType.SIMPLE_SEQUENCE,
            action_id=ItemRegistry.get_id(gateway.OneFileWorkspaceMock.write_with),
            **{
                'workspace': ws,
                'text': 'Hello Action Package!'
            }
        )

        seq = SequencePackage(
            id='my_sequence',
            priority=4,
            actions=[action_1, action_2]
        )
        package = seq.to_json()
        print(package)
