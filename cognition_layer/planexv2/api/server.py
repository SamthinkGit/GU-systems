from typing import Optional

from agent_protocol import Agent
from agent_protocol import Step
from agent_protocol import Task

from cognition_layer.api import ServerAPI
from cognition_layer.constants import API_PORT
from cognition_layer.planexv2.agents.planexv2 import PlanexV2
from cognition_layer.planexv2.agents.planexv2 import PlanexV2Message
from ecm.shared import get_logger


class PlanexV2Server(ServerAPI):
    # Deprecated: Replaced for server_from_iterator template

    _logger = get_logger("PlanexV2 Server")

    def __init__(self, verbose: Optional[bool] = False, *args, **kwargs) -> None:
        PlanexV2Server.verbose = verbose
        PlanexV2Server.planex = PlanexV2()
        PlanexV2Server.iterator = None

    def start(self, *args, **kwargs) -> None:
        Agent.setup_agent(
            PlanexV2Server.task_handler, PlanexV2Server.step_handler
        ).start(port=API_PORT)

    @staticmethod
    async def task_handler(task: Task) -> None:
        PlanexV2Server._logger.debug("Initializing task")
        await Agent.db.create_step(
            task_id=task.task_id, input=task.input, name="start (plan)", is_last=False
        )

    @staticmethod
    async def step_handler(step: Step) -> Step:
        try:
            if step.name == "start (plan)":
                PlanexV2Server.iterator = PlanexV2Server.planex.iter(
                    input=step.input, verbose=PlanexV2Server.verbose
                )

            message: PlanexV2Message = next(PlanexV2Server.iterator)

            if not message.is_last:
                await Agent.db.create_step(
                    task_id=step.task_id,
                    name=f"[{message.steps+1}]: {message.next_agent}",
                )
            step.output = message.content
            step.is_last = message.is_last

            assert isinstance(
                step.output, str
            ), "Invalid output returned from step_handler"
            return step

        except Exception:
            PlanexV2Server._logger.error(
                "Error occurred in step_handler: ", exc_info=True
            )
