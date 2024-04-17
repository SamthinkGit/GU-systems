"""
Task Management and Sequencing
==============================

This module provides a comprehensive set of classes and enums to manage tasks,
actions, and their sequencing with priorities. It includes the definition of
action types, task statuses, and sequence priorities, along with the core classes
for packaging actions and sequences, managing priority queues, and task registries.

Its main purpouse is to define the main behavior of the protocols used in
SequenceActionServer and SequenceActionClient.

Most important functions in line with package building, sending or ordering are in
this module
"""
import heapq
import itertools
import json
import pprint
from enum import Enum
from queue import Empty
from typing import Dict

from gusysros.tools.registry import ItemEncoder


class SequenceType(Enum):
    """
    Enumerates types of actions in a task sequence.

    - SIMPLE_SEQUENCE: A linear sequence of actions.
    - PARALLEL_SEQUENCE: Actions that can be executed in parallel.
    """

    SIMPLE_SEQUENCE = 0
    PARALLEL_SEQUENCE = 1


class TaskStatus(Enum):
    """
    Represents the current status of a task.

    - READY: Task is ready to be executed.
    - RUNNING: Task is currently in execution.
    - BLOCKED: Task is blocked and cannot proceed.
    """

    READY = 0
    RUNNING = 1
    BLOCKED = 2


class SequencePriority(Enum):
    """
    Defines the execution priority for task sequences.

    - URGENT: Highest priority, typically for emergency tasks.
    - INTERRUPTION: Immediate execution, interrupting current tasks.
    - INFO: For informational purposes, lower than normal priority.
    - NORMAL: Default priority for regular tasks (recommended and default).
    - CUMMULATIVE: For background tasks that can be deferred.
    - AT_EXIT: Executed just before a process or task exits.
    """

    AT_EXIT = 5
    CUMMULATIVE = 4
    NORMAL = 3
    INFO = 2
    INTERRUPTION = 1
    URGENT = 0


class ActionPackage:
    """
    Packages an action with its type, identifier, and parameters for execution within a task sequence.

    :param action_id: Unique identifier for the action (encoded function with ItemRegistry).
    :param args: Positional arguments for the action.
    :param kwargs: Keyword arguments for the action.

    .. code-block:: python
        from gusysros.tools.packages import ActionPackage

        # Build a function, if you want to send it in a package, you
        # should encode it with the ItemRegistry

        # WARNING: The ItemRegistry encoder and the client that executes
        # the function must be in the same memory space if they need to share variables.
        # If they do not share variables, it is enough to ensure the function is imported
        # for BOTH sender and client
        # TODO: Add a autoimport tool for simplifying this process

        @ItemRegistry.register_function
        def my_function(num, keyword_1, keyword_2):
            print("My num is ", num)
            print("My keyword_1 is ", keyword_1)
            print("My keyword_2 is ", keyword_2)

        my_num = 4
        action = ActionPackage(
            ItemRegistry.get_id(my_function),
            my_num,
            **{
                'keyword_1': 'Hello',
                'text': 'Action Package!'
            }
        )
    """

    def __init__(self, action_id: int, *args, **kwargs) -> None:

        self.action_id = action_id
        self.args = args
        self.kwargs = kwargs

    def to_dict(self, autoencode: bool = True):

        args = (
            [ItemEncoder.autoencode(arg) for arg in self.args]
            if autoencode
            else self.args
        )
        kwargs = (
            {key: ItemEncoder.autoencode(val) for key, val in self.kwargs.items()}
            if autoencode
            else self.kwargs
        )

        return {"action_id": self.action_id, "args": args, "kwargs": kwargs}

    def __str__(self) -> str:
        return str(self.to_dict(autoencode=False))

    @classmethod
    def from_dict(cls, dict_data: dict):

        required_keys = ["action_id", "args", "kwargs"]
        if not all(key in dict_data for key in required_keys):
            raise ValueError(
                f"Dict passed to ActionPackage must contain {required_keys}"
            )

        dict_data["args"] = [ItemEncoder.autodecode(val) for val in dict_data["args"]]
        dict_data["kwargs"] = {
            name: ItemEncoder.autodecode(val)
            for name, val in dict_data["kwargs"].items()
        }

        return cls(dict_data["action_id"], *dict_data["args"], **dict_data["kwargs"])


class SequencePackage:
    """
    Represents a sequence of actions (ActionPackages) with a specified priority within a task.

    :param task_id: Identifier for the task this sequence belongs to.
    :param type: The type/behavior of set of actions, defined by SequenceType.
    :param priority: Execution priority of this sequence, defined by SequencePriority.
    :param actions: A list of ActionPackage instances to be executed in this sequence.
    """

    def __init__(
        self,
        task_id: str,
        type: SequenceType,
        priority: int,
        actions: list[ActionPackage],
    ) -> None:

        if isinstance(priority, SequencePriority):
            priority = priority.value

        if isinstance(type, SequenceType):
            type = type.value

        self.type = type
        self.task_id = task_id
        self.priority = priority
        self.actions = actions

    def to_dict(self, autoencode: bool = True) -> dict:
        return {
            "task_id": self.task_id,
            "type": self.type if autoencode else SequenceType(self.type).name,
            "priority": self.priority,
            "actions": [
                action.to_dict(autoencode=autoencode) for action in self.actions
            ],
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=4)

    def __str__(self, pretty: bool = True) -> str:
        info = self.to_dict()
        info["actions"] = [action.to_dict(autoencode=False) for action in self.actions]

        if pretty:
            return pprint.pformat(info)

        return str(info)

    @classmethod
    def from_dict(cls, dict_data: dict):
        required_keys = ["task_id", "priority", "actions"]
        if not all(key in dict_data for key in required_keys):
            raise ValueError(
                f"Dict passed to ActionPackage must contain {required_keys}"
            )

        dict_data["actions"] = [
            ActionPackage.from_dict(action) for action in dict_data["actions"]
        ]
        return cls(**dict_data)

    @classmethod
    def from_json(cls, package: str):
        return SequencePackage.from_dict(json.loads(package))


class SimplePriorityQueue:
    """
    A simple priority queue implementation to manage tasks based on their priority.
    Utilizes a heap queue to ensure tasks are processed in the order of their priority.

    :note: We do not use standard PriorityQueue since we don't need thread-safe property
    and we need the exact size of the queue for ordering computations
    """

    def __init__(self):
        self._queue = []
        self._counter = itertools.count()

    def push(self, item, priority):
        count = next(self._counter)
        heapq.heappush(self._queue, (priority, count, item))

    def pop(self):
        return heapq.heappop(self._queue)[-1]

    def size(self):
        return len(self._queue)

    def to_list(self, only_priorities: bool = False):
        if only_priorities:
            return [priority for priority, _, _ in sorted(self._queue)]
        return [item for _, _, item in sorted(self._queue)]


class Task:
    """
    Manages a single task, holding a queue of sequences to be executed.
    It implements the ordering protocol stablished in the SequencePriority definitions
    by ordering the sequences by priority

    :param task_id: Unique identifier for the task.
    """

    def __init__(self, task_id: str) -> None:
        self.task_id = task_id
        self.status = TaskStatus.READY
        self.priority_queue = SimplePriorityQueue()

    def push(self, sequence: SequencePackage) -> None:
        self.priority_queue.push(sequence, priority=sequence.priority)

    def pop(self, sequence) -> SequencePackage:
        if self.priority_queue.size() <= 0:
            raise Empty
        return self.priority_queue.pop()

    def __len__(self) -> int:
        return self.priority_queue.size()

    def to_list(self, only_priorities: bool = True) -> list:
        return self.priority_queue.to_list(only_priorities=only_priorities)


class TaskRegistry:
    """
    A registry to manage multiple tasks, allowing for task updates, retrieval, and logging.

    Maintains a dictionary of tasks, facilitating the addition of new sequences to tasks and
    providing logging capabilities. It maintains priority protocols when using get(), update()
    or append()
    """

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

        assert (
            len(self.tasks) <= 0
        ), "Trying to pop an empty task (No sequences avaliable)"
        sequence = self.tasks[task_id].pop()

        if len(self.tasks[task_id]) == 0:
            del self.tasks[task_id]

        return sequence

    def get_log(self) -> None:
        return {
            "concurrent_tasks": len(self.tasks),
            "tasks": {
                id: {
                    "status": task.status.name,
                    "queue": [SequencePriority(prio).name for prio in task.to_list()],
                }
                for id, task in self.tasks.items()
            },
        }
