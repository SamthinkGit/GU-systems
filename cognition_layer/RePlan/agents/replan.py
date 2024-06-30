from dataclasses import dataclass
from typing import AsyncGenerator
from typing import Optional

from langchain.agents import AgentExecutor
from langchain.agents import create_react_agent
from langchain.prompts import PromptTemplate
from langchain.tools import tool
from langchain_openai import OpenAI

from cognition_layer.constants import DEFAULT_INSTRUCT_MODEL
from cognition_layer.planexv2.agents.planexv2 import PlanexV2
from cognition_layer.planexv2.agents.planexv2 import PlanexV2Message
from cognition_layer.RePlan.agents.prompts import RePlanPrompts
from ecm.mediator.feedback import ExecutionStatus
from ecm.mediator.feedback import Feedback
from ecm.shared import get_logger
from ecm.tools.async_interpreter import AsyncInterpreter


@dataclass
class ReplanResponse:
    name: str
    content: str
    is_last: bool


class RePlan:

    planexv2: PlanexV2
    verbose: bool = False
    _logger = get_logger("RePlan")
    _last_feedback: Optional[Feedback] = None
    _is_plan_running = False

    def __init__(
        self,
        async_interpreter: AsyncInterpreter,
        verbose: bool = False,
        max_planex_steps: Optional[int] = None,
    ) -> None:
        if max_planex_steps is None:
            max_planex_steps = 11

        RePlan.planexv2 = PlanexV2()
        RePlan.max_planex_steps = max_planex_steps
        RePlan.async_interpreter = async_interpreter
        RePlan.verbose = verbose
        tools = [
            plan,
            execute_exelent_code,
            obtain_only_last_feedback,
            get_all_feedback,
            deny_step,
            approve_step,
        ]

        prompt = PromptTemplate.from_template(RePlanPrompts.REPLAN_PROMPT)
        llm = OpenAI(temperature=0, model=DEFAULT_INSTRUCT_MODEL)
        agent = create_react_agent(llm, tools, prompt)
        RePlan.agent_executor = AgentExecutor(agent=agent, tools=tools, verbose=False, handle_parsing_errors=True)

    @classmethod
    async def iter(cls, query: str) -> AsyncGenerator[ReplanResponse, None]:
        async for chunk in cls.agent_executor.astream({"input": query}):
            if "actions" in chunk:
                action = chunk["actions"][0]
                yield ReplanResponse(
                    name=action.tool, content=action.log, is_last=False
                )
            elif "steps" in chunk:
                for step in chunk["steps"]:
                    cls._logger.debug(f"Tool Result: `{step.observation}`")

        yield ReplanResponse(
            name="FINISH", content="RePlan is exiting...", is_last=True
        )

    @staticmethod
    def get_meaning_of_feedback(feedback: Feedback):
        task_id = feedback.task_id
        match feedback._exec_status:
            case ExecutionStatus.RUNNING:
                return f"[RUNNING] {task_id}: Exelent task is running. Any problem has been detected."
            case ExecutionStatus.STEP:
                return f"[STEP] {task_id}: Succesfully executed `{feedback.object}` from the exelent code"
            case ExecutionStatus.SUCCESS:
                return f"[SUCCESS] {task_id}: A sequence of steps has completed with success."
            case ExecutionStatus.ABORT:
                return f"[ABORT] {task_id}: The execution of a sequence of steps has been aborted."
            case ExecutionStatus.FINISH:
                return f"[FINISH] {task_id}: An Exelent plan has fully finished its mission completion."
            case ExecutionStatus.REQUEST_TO_CONTINUE:
                return (
                    f"[REQUEST] {task_id}: The Step `{feedback.object[2]}` has been programmed to ask for "
                    "AI approval. Please use approve to continue, or deny for ending the task."
                )
            case _:

                RePlan._logger.warning(
                    f"Unidentified feedback returned from the interpreter: {feedback._exec_status}"
                )
                return "Unidentified action returned from the interpreter."


@tool
def plan(query: str) -> str:
    """Defines a plan for completing a query from the user inside the pc by using the defined actions.
    It returns Exelent Code. This function should be called for defining the plan for completing a query.
    """
    step: PlanexV2Message
    for step in RePlan.planexv2.iter(
        input=query, verbose=RePlan.verbose, max_steps=RePlan.max_planex_steps
    ):
        result = step.content

    RePlan.async_interpreter.set_exelent_code(result)
    return (
        f"The plan generated is ```{result}```. If the plan is correct, we should"
        "use execute_exelent_code in order to start the execution."
    )


@tool
async def execute_exelent_code(plan_name: str) -> str:
    """Starts the execution of a generated plan. It automatically finds the last plan generated.
    Should only be called to start the execution. After this, the execution tools are enabled.
    """
    await RePlan.async_interpreter.start_execution()
    RePlan._is_plan_running = True
    RePlan.async_interpreter.flush()
    return "Execution of exelent code has started. Keep feedback from it for finding its status."


@tool
async def obtain_only_last_feedback(plan_name: str) -> str:
    """Obtains feedback from the last step executed in the plan.
    Only should be used to ensure the last step. Usually you should use get_execution_status_history.
    """
    if not RePlan._is_plan_running:
        return "There are no plans runnning at the moment."

    feedback: Feedback = await AsyncInterpreter.get_feedback()
    RePlan._last_feedback = feedback
    if feedback._exec_status == ExecutionStatus.FINISH:
        RePlan._is_plan_running = False

    return RePlan.get_meaning_of_feedback(feedback)


@tool
def get_all_feedback(plan_name: str) -> str:
    """Shows all the feedback obtained from the beggining of the execution of the plan."""

    if not RePlan._is_plan_running:
        return "There are no plans runnning at the moment."

    RePlan._last_feedback = ExecutionStatus.REQUEST_TO_CONTINUE
    history = RePlan.async_interpreter.history.copy()
    if history[-1]._exec_status == ExecutionStatus.FINISH:
        RePlan._is_plan_running = False

    parsed_history = [RePlan.get_meaning_of_feedback(entry) for entry in history]
    return "\n".join(parsed_history)


@tool
def approve_step(last_step: str) -> str:
    """Approves the last step found in order to continue running appropiately. It can only be used if
    the last step asks for approval."""
    if not RePlan._is_plan_running:
        return "There are no plans runnning at the moment."

    if RePlan._last_feedback is None:
        return (
            "Can't approve an invalid step. Please first ensure you have planned, executed and obtained "
            "the feedback of the query."
        )

    if RePlan._last_feedback._exec_status != ExecutionStatus.REQUEST_TO_CONTINUE:
        return "This step does not need to be approved."

    RePlan._last_feedback.response(None, ExecutionStatus.CONTINUE)
    return "Step Approved"


@tool
def deny_step(last_step: str) -> str:
    """Denies the execution of the last step. This should be used if it is considered
    that the step will fail or if there are security risks. It only can be used if the
    last step asks for approval."""
    if not RePlan._is_plan_running:
        return "There are no plans runnning at the moment."

    if RePlan._last_feedback is None:
        return (
            "Can't deny an invalid step. Please first ensure you have planned, executed "
            "and obtained the feedback of the query."
        )

    if RePlan._last_feedback._exec_status != ExecutionStatus.REQUEST_TO_CONTINUE:
        return "This step can't be denied."

    RePlan._last_feedback.response(None, ExecutionStatus.ABORT)
    return "Step Denied, exiting."
