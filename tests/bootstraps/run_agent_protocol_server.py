from agent_protocol import Agent
from agent_protocol import Step
from agent_protocol import Task
from colorama import Fore
from colorama import Style

import tests.mocks.agent_actions  # noqa
from cognition_layer.planex.agents.planner import Planner
from cognition_layer.planex.agents.reducer import Reducer
from cognition_layer.planex.agents.translator import Translator


class AgentServer:

    planner = Planner()
    reducer = Reducer()
    translator = Translator()


async def task_handler(task: Task) -> None:
    # TODO: Create initial step(s) for the task
    print(Fore.GREEN + Style.BRIGHT + "[Server] Initializing..." + Style.RESET_ALL)
    await Agent.db.create_step(task_id=task.task_id, input=task.input, name="plan")
    await Agent.db.create_step(task_id=task.task_id, input=task.input, name="reduce")
    await Agent.db.create_step(task_id=task.task_id, input=task.input, name="translate")
    AgentServer.reducer.auto_bind_actions()


async def step_handler(step: Step) -> Step:
    print(
        Fore.GREEN
        + Style.BRIGHT
        + f"[Server] Running Step: {step.name}"
        + Style.RESET_ALL
    )
    match step.name:
        case "plan":
            result = AgentServer.planner.plan(step.input).content
            step.output = result

        case "reduce":
            result = AgentServer.reducer.reduce(step.input).content
            step.output = result

        case "translate":
            result = AgentServer.translator.translate(step.input).content
            step.output = result
            step.is_last = True

    return step


Agent.setup_agent(task_handler, step_handler).start()
