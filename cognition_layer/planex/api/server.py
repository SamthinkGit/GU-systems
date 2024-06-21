from typing import Optional

from agent_protocol import Agent
from agent_protocol import Step
from agent_protocol import Task

from cognition_layer.api import ServerAPI
from cognition_layer.constants import API_PORT
from cognition_layer.planex.agents.planner import Planner
from cognition_layer.planex.agents.reducer import Reducer
from cognition_layer.planex.agents.translator import Translator
from ecm.shared import get_logger


class PlanexServer(ServerAPI):

    _logger = get_logger("Planex Server")
    planner: Planner | None = None
    reducer: Reducer | None = None
    translator: Translator | None = None
    verbose: bool = False

    def __init__(self, verbose: Optional[bool] = False) -> None:
        PlanexServer.planner = Planner()
        PlanexServer.reducer = Reducer()
        PlanexServer.translator = Translator()
        PlanexServer.verbose = verbose

    def start(self) -> None:
        Agent.setup_agent(PlanexServer.task_handler, PlanexServer.step_handler).start(
            port=API_PORT
        )

    @staticmethod
    async def task_handler(task: Task) -> None:
        PlanexServer._logger.debug("Initializing...")
        await Agent.db.create_step(task_id=task.task_id, input=task.input, name="plan")
        await Agent.db.create_step(
            task_id=task.task_id, input=task.input, name="reduce"
        )
        await Agent.db.create_step(
            task_id=task.task_id, input=task.input, name="translate", is_last=True
        )
        PlanexServer.reducer.auto_bind_actions()

    @staticmethod
    async def step_handler(step: Step) -> Step:

        PlanexServer._logger.debug(f"Running Step: {step.name}")

        match step.name:
            case "plan":
                result = PlanexServer.planner.plan(
                    step.input, verbose=PlanexServer.verbose
                ).content
                step.output = result

            case "reduce":
                result = PlanexServer.reducer.reduce(
                    step.input, verbose=PlanexServer.verbose
                ).content
                step.output = result

            case "translate":
                result = PlanexServer.translator.translate(
                    step.input, verbose=PlanexServer.verbose
                ).content
                step.output = result

        return step
