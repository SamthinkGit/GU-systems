"""
RePlan Module
==============================

This module defines the RePlan class and its associated tools, which orchestrate the
planning and execution of tasks using the PlanexV2 agent and the LangChain framework.

It is based on ReAct Agent methodology but using PlanexV2 as main planner. Also contains other
tools based on the execution layer for fully control the execution, being able to
stop, continue, approve steps etc.

[HELP] There is an example for using this module in tests/sandbox/replan.py
"""
from dataclasses import dataclass
from typing import AsyncGenerator
from typing import Optional

from colorama import Fore
from colorama import Style
from langchain.agents import AgentExecutor
from langchain.agents import create_react_agent
from langchain.prompts import PromptTemplate
from langchain.tools import tool
from langchain_openai import OpenAI

from cognition_layer.constants import DEFAULT_INSTRUCT_MODEL
from cognition_layer.planexv2.agents.planexv2 import PlanexV2
from cognition_layer.planexv2.agents.planexv2 import PlanexV2Message
from cognition_layer.RePlan.agents.prompts import RePlanPrompts
from ecm.mediator.execution_layer_wrapper import ExecutionLayerWrapper
from ecm.mediator.feedback import ExecutionStatus
from ecm.mediator.feedback import Feedback
from ecm.shared import get_logger


@dataclass
class ReplanResponse:
    """
    Represents the response from the RePlan iterative process. It includes the
    name of the action, the content of the response, and a flag indicating whether
    it is the last response.

    :param name: The name of the action or step.
    :param content: The content of the response.
    :param is_last: Boolean indicating if this is the last response.
    """

    name: str
    content: str
    is_last: bool


class RePlan:

    planexv2: PlanexV2
    verbose: bool = False
    _logger = get_logger("RePlan")
    _is_plan_running = False

    def __init__(
        self,
        execution_layer_wrapper: ExecutionLayerWrapper,
        verbose: bool = False,
        max_planex_steps: Optional[int] = None,
    ) -> None:
        """
        Initializes the RePlan class with an execution layer wrapper, verbosity
        settings, and the maximum number of PlanexV2 steps. Sets up the agent
        executor with defined tools and prompts.

        :param execution_layer_wrapper: The wrapper for the execution layer. You
        can use ecm.core.mediator.execution_layer_wrapper. It is used for synchronizing
        both cognition and execution layer.
        :param verbose: Optional flag to enable verbose logging.
        :param max_planex_steps: Optional maximum number of steps for PlanexV2.
        """
        if max_planex_steps is None:
            max_planex_steps = 11

        RePlan.planexv2 = PlanexV2()
        RePlan.max_planex_steps = max_planex_steps
        RePlan.execution_layer_wrapper = execution_layer_wrapper
        RePlan.verbose = verbose
        tools = [
            plan,
            execute_exelent_code,
            get_all_feedback,
            deny_step,
            approve_step,
            close_task,
        ]

        prompt = PromptTemplate.from_template(RePlanPrompts.REPLAN_PROMPT)
        llm = OpenAI(temperature=0, model=DEFAULT_INSTRUCT_MODEL)
        agent = create_react_agent(llm, tools, prompt)
        RePlan.agent_executor = AgentExecutor(
            agent=agent, tools=tools, verbose=False, handle_parsing_errors=True
        )

    @classmethod
    async def iter(cls, query: str) -> AsyncGenerator[ReplanResponse, None]:
        """
        Asynchronously processes the input query iteratively using the agent
        executor, yielding ReplanResponse objects for each step.

        :param query: The input query to be processed.
        :return: An asynchronous generator yielding ReplanResponse objects.

        [HELP] There is an example for using this module in tests/sandbox/replan.py
        """

        async for chunk in cls.agent_executor.astream({"input": query}):
            if "actions" in chunk:
                action = chunk["actions"][0]
                yield ReplanResponse(
                    name=action.tool, content=action.log, is_last=False
                )
            elif "steps" in chunk:
                for step in chunk["steps"]:
                    action = step.action
                    cls._logger.debug(
                        f"{Fore.YELLOW}{Style.BRIGHT}{action.log}{Style.RESET_ALL}"
                    )
                    cls._logger.debug(
                        f"{Fore.YELLOW}{Style.BRIGHT}[action]: `{action.tool}`{Style.RESET_ALL}"
                    )
                    cls._logger.debug(
                        f"{Fore.LIGHTGREEN_EX}{Style.BRIGHT}[Result]: `{step.observation}`{Style.RESET_ALL}"
                    )

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

    RePlan.execution_layer_wrapper.set_exelent_code(result)
    return (
        f"The plan generated is ```{result}```.\nIf the plan is correct, we should"
        "use execute_exelent_code in order to start the execution."
    )


@tool
def execute_exelent_code(plan_name: str) -> str:
    """Starts the execution of a generated plan. It automatically finds the last plan generated.
    Should only be called to start the execution. After this, the execution tools are enabled.
    This should only be called one time for executing all the code, repeating the call of this function
    may lead to invalid feedback.
    """
    if RePlan._is_plan_running:
        return "There is a plan already running. Keep feedback from it for finding its status."

    RePlan.execution_layer_wrapper.start_execution()
    RePlan._is_plan_running = True
    return "Execution of exelent code has started. Keep feedback from it for finding its status."


@tool
def get_all_feedback(plan_name: str) -> str:
    """Shows all the feedback obtained from the beggining of the execution of the plan."""

    if not RePlan._is_plan_running:
        return "There are no plans runnning at the moment."

    with RePlan.execution_layer_wrapper._lock:
        history = RePlan.execution_layer_wrapper.history.copy()

    if len(history) == 0:
        return "There are no feedback avaliable at the moment."

    parsed_history = [RePlan.get_meaning_of_feedback(entry) for entry in history]
    return "\n".join(parsed_history)


@tool
def approve_step(last_step: str) -> str:
    """Approves the last step found in order to continue running appropiately. It can only be used if
    the last step asks for approval."""
    if not RePlan._is_plan_running:
        return "There are no plans runnning at the moment."

    with RePlan.execution_layer_wrapper._lock:
        history = RePlan.execution_layer_wrapper.history
        if len(history) == 0:
            return "There are no feedback yet. No approval available"

        if history[-1]._exec_status != ExecutionStatus.REQUEST_TO_CONTINUE:
            return "The last step does not need to be approved."

        history[-1].response(None, ExecutionStatus.CONTINUE)
        history.pop()

    return "Step Approved"


@tool
def deny_step(last_step: str) -> str:
    """Denies the execution of the last step. This should be used if it is considered
    that the step will fail or if there are security risks. It only can be used if the
    last step asks for approval."""
    if not RePlan._is_plan_running:
        return "There are no plans runnning at the moment."

    with RePlan.execution_layer_wrapper._lock:
        history = RePlan.execution_layer_wrapper.history
        if len(history) == 0:
            return "There are no feedback yet. No approval available"

        if history[-1]._exec_status != ExecutionStatus.REQUEST_TO_CONTINUE:
            return "The last step does not need to be approved."

        history[-1].response(None, ExecutionStatus.ABORT)
    return "Step Denied, exiting."


@tool
def close_task(task_name: str) -> str:
    """Closes an executed task. Should be called after the feedback
    traces have been completed and before executing a new plan."""
    RePlan._is_plan_running = False
    RePlan.execution_layer_wrapper.flush()
    return "Task successfully closed"
