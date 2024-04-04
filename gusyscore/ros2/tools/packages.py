import json
import pprint
from enum import Enum

import gusyscore.gateway.mocks.gateway as gateway
from gusyscore.ros2.tools.registry import ItemEncoder
from gusyscore.ros2.tools.registry import ItemRegistry


class ActionType(Enum):
    SIMPLE_SEQUENCE = 1
    PARALLEL_SEQUENCE = 2


class ActionPackage():
    def __init__(self, type: ActionType, action_id: int, *args, **kwargs) -> None:

        if not isinstance(type, ActionType):
            type = ActionType(type)

        self.type = type
        self.action_id = action_id
        self.args = args
        self.kwargs = kwargs

    def to_dict(self, autoencode: bool = True):

        type = self.type.value if autoencode else self.type.name
        args = [ItemEncoder.autoencode(arg) for arg in self.args] if autoencode else self.args
        kwargs = {key: ItemEncoder.autoencode(val) for key, val in self.kwargs.items()} if autoencode else self.kwargs

        return {
            'type': type,
            'action_id': self.action_id,
            'args': args,
            'kwargs': kwargs
        }

    def __str__(self) -> str:
        return str(self.to_dict(autoencode=False))

    @classmethod
    def from_dict(cls, dict_data: dict):

        required_keys = ['type', 'action_id', 'args', 'kwargs']
        if not all(key in dict_data for key in required_keys):
            raise ValueError(f"Dict passed to ActionPackage must contain {required_keys}")

        dict_data['args'] = [ItemEncoder.autodecode(val) for val in dict_data['args']]
        dict_data['kwargs'] = {name: ItemEncoder.autodecode(val) for name, val in dict_data['kwargs'].items()}

        return cls(**dict_data)


class SequencePackage():
    def __init__(self, id: str, priority: int, actions: list[ActionPackage]) -> None:

        self.id = id
        self.priority = priority
        self.actions = actions

    def to_dict(self, autoencode: bool = True):
        return {
            'id': self.id,
            'priority': self.priority,
            'actions': [action.to_dict(autoencode=autoencode) for action in self.actions]
        }

    def to_json(self):
        return json.dumps(self.to_dict(), indent=4)

    def __str__(self, pretty: bool = True) -> str:
        info = self.to_dict()
        info['actions'] = [action.to_dict(autoencode=False) for action in self.actions]

        if pretty:
            return pprint.pformat(info)

        return str(info)

    @classmethod
    def from_dict(cls, dict_data: dict):
        required_keys = ['id', 'priority', 'actions']
        if not all(key in dict_data for key in required_keys):
            raise ValueError(f"Dict passed to ActionPackage must contain {required_keys}")

        dict_data['actions'] = [ActionPackage.from_dict(action) for action in dict_data['actions']]
        return cls(**dict_data)

    @classmethod
    def from_json(cls, package: str):
        return SequencePackage.from_dict(json.loads(package))


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

        pkg = seq.to_json()
        print(" JSON Package Format ".center(60, "-"))
        print(pkg)

        decoded_seq = SequencePackage.from_json(pkg)
        print(" Decoded Package ".center(60, "-"))
        print(decoded_seq)
