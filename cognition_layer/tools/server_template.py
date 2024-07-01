from typing import Callable
from typing import Optional

from agent_protocol import Agent
from agent_protocol import Step
from agent_protocol import Task

from cognition_layer.api import ServerAPI
from cognition_layer.constants import API_PORT
from ecm.shared import get_logger


class ServerFromIterator(ServerAPI):

    def __init__(
        self,
        name: str,
        iterator: Callable,
        content_getter: Callable,
        is_last_getter: Callable,
        async_iterator: bool = False,
        *args,
        step_name_getter: Optional[Callable] = None,
        **kwargs,
    ) -> None:
        ServerFromIterator.iterator = iterator
        ServerFromIterator._logger = get_logger(name)
        ServerFromIterator.step_name_getter = step_name_getter
        ServerFromIterator.content_getter = content_getter
        ServerFromIterator.is_last_getter = is_last_getter
        ServerFromIterator.async_iterator = async_iterator
        ServerFromIterator.steps = 0

    def start(self, *args, **kwargs) -> None:
        Agent.setup_agent(
            ServerFromIterator.task_handler, ServerFromIterator.step_handler
        ).start(port=API_PORT)

    @staticmethod
    async def task_handler(task: Task) -> None:
        ServerFromIterator._logger.debug("Initializing task")
        ServerFromIterator.steps = 0

        await Agent.db.create_step(
            task_id=task.task_id, input=task.input, name="start", is_last=False
        )

    @staticmethod
    async def step_handler(step: Step) -> Step:
        try:

            if step.name == "start":
                ServerFromIterator.iterator = ServerFromIterator.iterator(step.input)

            message = (
                await anext(ServerFromIterator.iterator)
                if ServerFromIterator.async_iterator
                else next(ServerFromIterator.iterator)
            )
            ServerFromIterator.steps += 1

            if ServerFromIterator.step_name_getter:
                name = ServerFromIterator.step_name_getter(message)
            else:
                name = ""

            if not message.is_last:
                await Agent.db.create_step(
                    task_id=step.task_id,
                    name=f"Step [{ServerFromIterator.steps}] {name}",
                )
            step.output = ServerFromIterator.content_getter(message)
            step.is_last = ServerFromIterator.is_last_getter(message)

            assert isinstance(
                step.output, str
            ), "Invalid output returned from step_handler"
            return step

        except Exception:
            ServerFromIterator._logger.error(
                "Error occurred in step_handler: ", exc_info=True
            )
