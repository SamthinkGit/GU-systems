"""
Server From Iterator Module
==============================

This module defines the ServerFromIterator class, which sets up a server to process
tasks and steps using a provided iterator function. It integrates with the Agent
Protocol to manage task and step handling, facilitating the iterative processing
of input queries. The server can handle both synchronous and asynchronous iterators
and manages the creation and updating of steps in the task processing workflow.

The module is used to easily create a server that complies with the Agent Protocol.

[HELP] An implemented server with this metodology can easily be controlled with
the mediator in cognition_layer.api.CognitionMediator
"""
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
        """
        Initializes the ServerFromIterator with the provided iterator function,
        content getter, is_last getter, and optional step name getter. Sets up
        logging and prepares the iterator for task processing.

        :param name: The name of the server for logging purposes.
        :param iterator: The function to generate the iterator for task processing.
        :param content_getter: The function to get the content from the iterator yielded message.
        :param is_last_getter: The function to determine if the current message is the last from the yielded message.
        :param async_iterator: Flag indicating if the iterator is asynchronous.
        :param step_name_getter: Optional function to get the name of the step from each yielded message.
        """
        ServerFromIterator.iterator = iterator
        ServerFromIterator._logger = get_logger(name)
        ServerFromIterator.step_name_getter = step_name_getter
        ServerFromIterator.content_getter = content_getter
        ServerFromIterator.is_last_getter = is_last_getter
        ServerFromIterator.async_iterator = async_iterator
        ServerFromIterator.steps = 0

    def start(self, *args, **kwargs) -> None:
        """
        Starts the server by setting up the agent to handle tasks and steps,
        and begins listening on the designated API port.
        """
        Agent.setup_agent(
            ServerFromIterator.task_handler, ServerFromIterator.step_handler
        ).start(port=API_PORT)

    @staticmethod
    async def task_handler(task: Task) -> None:
        """
        Use this function for quering the Agent with the user input.

        Handles the initialization of a new task.
        :param task: The task object containing task details.
        """
        ServerFromIterator._logger.debug("Initializing task")
        ServerFromIterator.steps = 0

        await Agent.db.create_step(
            task_id=task.task_id, input=task.input, name="start", is_last=False
        )

    @staticmethod
    async def step_handler(step: Step) -> Step:
        """
        Use this function to iterate over the steps of the Agent.

        This function will proccess each step of the task iteratively using the provided
        iterator function. Manages the state and output of each step, logging errors if
        they occur.

        :param step: The step object containing step details.
        :return: The updated step object with the output and completion status.
        """
        try:

            # Initialize the iterator
            if step.name == "start":
                ServerFromIterator.iterator_step = ServerFromIterator.iterator(step.input)

            # Obtain step
            message = (
                await anext(ServerFromIterator.iterator_step)
                if ServerFromIterator.async_iterator
                else next(ServerFromIterator.iterator_step)
            )
            ServerFromIterator.steps += 1

            # Set the properties and generate new step if it is not the last
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
