import json
import pprint
from enum import Enum
from queue import Empty
from queue import PriorityQueue
from typing import Dict

import gusyscore.gateway.mocks.gateway as gateway
from gusysros.tools.registry import ItemEncoder
from gusysros.tools.registry import ItemRegistry


class ActionType(Enum):
    SIMPLE_SEQUENCE = 1
    PARALLEL_SEQUENCE = 2


class TaskStatus(Enum):
    READY = 0
    RUNNING = 1
    BLOCKED = 2


class SequencePriority(Enum):
    AT_EXIT = 5
    CUMMULATIVE = 4
    NORMAL = 3
    INFO = 2
    INTERRUPTION = 1
    URGENT = 0


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

        return cls(
            dict_data['type'],
            dict_data['action_id'],
            *dict_data['args'],
            **dict_data['kwargs']
        )


class SequencePackage():
    def __init__(self, task_id: str, priority: int, actions: list[ActionPackage]) -> None:

        if isinstance(priority, SequencePriority):
            priority = priority.value

        self.task_id = task_id
        self.priority = priority
        self.actions = actions

    def to_dict(self, autoencode: bool = True) -> dict:
        return {
            'task_id': self.task_id,
            'priority': self.priority,
            'actions': [action.to_dict(autoencode=autoencode) for action in self.actions]
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=4)

    def __str__(self, pretty: bool = True) -> str:
        info = self.to_dict()
        info['actions'] = [action.to_dict(autoencode=False) for action in self.actions]

        if pretty:
            return pprint.pformat(info)

        return str(info)

    @classmethod
    def from_dict(cls, dict_data: dict):
        required_keys = ['task_id', 'priority', 'actions']
        if not all(key in dict_data for key in required_keys):
            raise ValueError(f"Dict passed to ActionPackage must contain {required_keys}")

        dict_data['actions'] = [ActionPackage.from_dict(action) for action in dict_data['actions']]
        return cls(**dict_data)

    @classmethod
    def from_json(cls, package: str):
        return SequencePackage.from_dict(json.loads(package))


class Task():

    def __init__(self, task_id: str) -> None:
        self.task_id = task_id
        self.status = TaskStatus.READY
        self.priority_queue = PriorityQueue()

    def push(self, sequence: SequencePackage) -> None:
        self.priority_queue.put((sequence.priority, sequence))

    def pop(self, sequence) -> SequencePackage:
        if len(self.priority_queue) <= 0:
            raise Empty
        return self.priority_queue.get()[1]

    def __len__(self) -> int:
        return len(self.priority_queue)

    def to_list(self, only_priorities: bool = True) -> list:
        temp_list = []
        while not self.priority_queue.empty():
            item: SequencePackage = self.priority_queue.get()[1]

            if only_priorities:
                temp_list.append(item.priority)
            else:
                temp_list.append(item)

        for item in temp_list:
            self.priority_queue.put(item)
        return temp_list


class TaskRegistry():

    def __init__(self) -> None:
        self.tasks: Dict[str, Task] = {}

    def append(self, sequence: SequencePackage) -> None:
        self.update(sequence)

    def update(self, sequence: SequencePackage) -> None:

        id = sequence.task_id
        if id not in self.tasks:
            self.tasks[id] = Task(id)

        self.tasks[id].push(sequence)

    def get(self, task_id: str) -> SequencePackage | None:
        if task_id not in self.tasks:
            return None

        assert len(self.tasks) <= 0, "Trying to pop an empty task (No sequences avaliable)"
        sequence = self.tasks[task_id].pop()

        if len(self.tasks[task_id]) == 0:
            del self.tasks[task_id]

        return sequence

    def get_log(self) -> None:
        return {
            'concurrent_tasks': len(self.tasks),
            'tasks': {
                id: {
                    'status': task.status.name,
                    'queue': [SequencePriority(prio).name for prio in task.to_list()]
                } for id, task in self.tasks.items()
            },
        }


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
            task_id='my_sequence',
            priority=SequencePriority.NORMAL,
            actions=[action_1, action_2]
        )

        tr = TaskRegistry()
        tr.append(seq)
        tr.append(seq)
        tr.append(seq)

        pkg = seq.to_json()
        print(" JSON Package Format ".center(60, "-"))
        print(pkg)

        decoded_seq = SequencePackage.from_json(pkg)
        print(" Decoded Package ".center(60, "-"))
        print(decoded_seq)

        print(" TaskRegistry ".center(60, "-"))
        print(json.dumps(tr.get_log(), indent=4))
