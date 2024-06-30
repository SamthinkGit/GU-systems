"""
Cognition Mediator Module
=================================

This module provides the `CognitionMediator` class for managing and executing tasks
using the Agent Protocol. The mediator acts as an intermediary, handling the interaction
between the server and the agents, and ensuring the smooth execution of task plans
through the various steps defined in the agent protocol.

The module is designed to work seamlessly with the `ServerAPI` defined class for each agent from the
cognition layer, facilitating the orchestration of tasks by integrating the agent protocol client.
This allows for the efficient handling of task creation, step execution, and result retrieval.
"""
import agent_protocol_client
from agent_protocol import Step
from agent_protocol import Task
from agent_protocol_client.models import TaskStepsListResponse
from agent_protocol_client.rest import ApiException

from ecm.shared import get_logger


class ServerAPI:

    def __init__(self, *args, **kwargs) -> None: ...
    def start(self, *args, **kwargs) -> None: ...
    async def task_handler(task: Task) -> None: ...
    async def step_handler(step: Step) -> Step: ...


class CognitionMediator:

    _logger = get_logger("Cognition Mediator")

    def __init__(self, api_client) -> None:
        self.api_instance = agent_protocol_client.AgentApi(api_client)

    async def get_task(self, input) -> Task:

        task = agent_protocol_client.TaskRequestBody(input=input)
        try:
            task_response: Task = await self.api_instance.create_agent_task(task)
        except ApiException:
            self._logger.error("Exception when calling AgentApi:\n", exc_info=True)

        return task_response

    async def run_task(self, task: Task) -> str | None:

        # Obtaining steps
        step_list: TaskStepsListResponse = (
            await self.api_instance.list_agent_task_steps(task_id=task.task_id)
        )
        self._logger.debug("Steps received from API")

        if len(step_list.steps) <= 0:
            self._logger.warn("Steps received are empty? Skip.")
            return None

        self._logger.info("Running Agent...")
        is_last = False

        counter = 0
        try:
            while not is_last:

                step_list: TaskStepsListResponse = (
                    await self.api_instance.list_agent_task_steps(task_id=task.task_id)
                )

                self._logger.debug(f"Running Step `{step_list.steps[counter].name}`")
                result = await self.api_instance.execute_agent_task_step(
                    task_id=task.task_id, step_request_body=step_list.steps[counter]
                )

                is_last = result.is_last
                counter += 1

            return result.output

        except Exception:
            self._logger.error(
                "Error occurred while running AgentProtocol Steps: ", exc_info=True
            )
