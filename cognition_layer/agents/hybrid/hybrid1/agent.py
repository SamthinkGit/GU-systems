import json
from typing import Generator

from langchain.tools import tool as build_tool
from langchain_core.messages import AIMessage
from langchain_core.messages import BaseMessage
from langchain_core.messages import HumanMessage
from langchain_core.messages import SystemMessage
from langchain_core.tools import StructuredTool
from pydantic import BaseModel
from pydantic import ConfigDict
from pydantic import Field

from action_space.meta.call_expert.sync import sync_experts
from action_space.meta.cognition_state.state import CognitionState
from cognition_layer.agents.hybrid.hybrid1.prompt import HYBRID1_PROMPT
from cognition_layer.agents.minimal_vfr.agents.agent import _convert_response_to_exelent
from cognition_layer.agents.minimal_vfr.agents.agent import TaskSummary
from cognition_layer.agents.planex.utils.format import format_tool
from cognition_layer.deploy.types import DeploySchema
from cognition_layer.memory.simple import SimpleCognitiveMemory
from cognition_layer.protocols.fast_ap import FastAPStep
from cognition_layer.routing.routers.simple_router.simple_router import SimpleRouter
from cognition_layer.tools.mutable_llm import MutableChatLLM
from ecm.mediator.feedback import ExecutionStatus
from ecm.mediator.feedback import Feedback
from ecm.mediator.Interpreter import Interpreter
from ecm.tools.registry import ItemRegistry


class Hybrid1Response(BaseModel):
    reasoning: str = Field(description="A reasoning for about the next steps")
    function: str = Field(
        description="The next function with pythonic notation. E.g: myfunc(2, 3, 'foo')."  # noqa
    )


class HybridCognitionState(BaseModel):
    model_config = ConfigDict(extra="allow")
    goal: str = Field(description="The goal of the task")
    scratchpad: str = Field(
        description="A scratchpad to store intermediate thoughts, calculations, or notes."
    )
    objective_completed: bool = Field(
        description="A boolean indicating whether the objective has been completed."
    )


class Hybrid1:

    @ItemRegistry.require_dependencies("meta")
    def __init__(
        self,
        schema: DeploySchema,
        interpreter: Interpreter = None,
        registry: ItemRegistry = ItemRegistry(),
        memory_capacity: int = 10,
    ):
        self.schema = schema
        self.router = SimpleRouter(schema)
        self.llm = MutableChatLLM().with_structured_output(Hybrid1Response)
        self.registry = registry
        self.interpreter = interpreter
        self.memory_capacity = memory_capacity
        self.cognition_state = None
        self.formatted_tools = None
        self.memory = None

    def _initialize_agent_task(self, input):
        """
        Initializes the agent's task with the given input.

        Args:
            input (str): The input prompt for the task.
        """
        self.cognition_state = CognitionState(HybridCognitionState)
        self.cognition_state.set("goal", input)
        self.cognition_state.set("objective_completed", False)
        self.cognition_state.set("scratchpad", "Empty")

        self.formatted_tools = "\n\n- ".join(self.get_formatted_actions())
        self.formatted_experts = json.dumps(self.router.summary_dict(), indent=2)
        self.memory = SimpleCognitiveMemory(
            capacity=self.memory_capacity, keep_images=False
        )
        self.memory.update([HumanMessage(content=input)])

    def _get_next_prompt(self) -> list[BaseMessage]:
        """
        Generates the next prompt for the AI based on the current state.

        Returns:
            list[BaseMessage]: A list of messages forming the prompt.
        """

        instructions = HYBRID1_PROMPT.format(
            tools=self.formatted_tools,
            experts=self.formatted_experts,
            cognition_state=self.cognition_state.summary(),
        )

        return [SystemMessage(content=instructions)] + self.memory.messages

    def _execute_response_from_agent(self, response: Hybrid1Response) -> None:
        """
        Executes the response received from the AI agent.

        Args:
            response (Hybrid1Response): The structured response from the AI.
        """
        task = _convert_response_to_exelent(response)
        self.latest_task = TaskSummary()
        if task is not None:
            self.interpreter.run(task, callback=self.retrieve_task_result)
        else:
            self.latest_task = TaskSummary(success=True, result="Invalid syntax")
        self.memory.update(
            [
                AIMessage(content=str(response)),
                AIMessage(
                    content=f"`{response.function}` returned the following summary: `{self.latest_task}`"
                ),
            ]
        )

    def retrieve_task_result(self, feedback: Feedback):
        """
        Updates the latest task summary based on the feedback received.
        Using this function enables the agent to retrieve the result andç
        possible errors from the interpreter.

        Args:
            feedback (Feedback): The feedback object containing execution status.
        """
        assert (
            self.latest_task is not None
        ), "No task has been initialized executed yet."
        self.latest_task: TaskSummary

        if feedback._exec_status == ExecutionStatus.SUCCESS:
            self.latest_task.success = True

        if feedback._exec_status == ExecutionStatus.RESULT:
            self.latest_task.result = feedback.object

    def get_formatted_actions(self) -> list[str]:
        """
        Retrieves and formats the actions available in the registry.

        Returns:
            list[str]: A list of formatted action strings.

        Examples:
            >>> agent.get_formatted_actions()
            ['Action do_something: do_something(*args, **kwargs)', ...]
        """
        actions = [action.content for action in self.registry.actions.values()]
        tools: list[StructuredTool] = [build_tool(a) for a in actions]
        return [format_tool(t) for t in tools]

    def complete_task(self, input: str) -> Generator[FastAPStep, None, None]:
        """
        Completes a task based on the given input prompt.

        Args:
            input (str): The input prompt describing the task.

        Yields:
            VFRFeedbackStep: Steps of the task execution process.

        Examples:
            >>> for step in agent.complete_task("Organize my files"):
            ...     print(step)
            FastAPStep(name='Hybrid1', content='...', is_last=False)
        """
        self._initialize_agent_task(input)
        exit = False

        while not exit:

            prompt = self._get_next_prompt()
            response: Hybrid1Response = self.llm.invoke(prompt)
            self.cognition_state.set("scratchpad", response.reasoning)
            obj = self.cognition_state.get("objective_completed")
            exit = (obj) or (isinstance(obj, str) and obj.lower() == "true")

            yield FastAPStep(name="Hybrid1", content=str(response), is_last=exit)

            sync_experts(self.schema, self.interpreter)
            self._execute_response_from_agent(response)
