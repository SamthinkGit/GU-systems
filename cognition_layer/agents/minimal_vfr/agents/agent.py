import random
from dataclasses import dataclass
from typing import Any
from typing import Generator
from typing import Literal

from langchain.tools import tool as build_tool
from langchain_core.messages import AIMessage
from langchain_core.messages import HumanMessage
from langchain_core.messages import SystemMessage
from langchain_core.messages.utils import get_buffer_string
from langchain_core.tools import StructuredTool
from pydantic import BaseModel
from pydantic import Field

from action_space.meta.cognition_state.state import CognitionState
from action_space.tools.image import ImageMessage
from action_space.tools.image import load_image
from cognition_layer.agents.minimal_vfr.agents.prompt import MINVFR_PROMPT
from cognition_layer.agents.planex.utils.format import format_tool
from cognition_layer.memory.simple import SimpleCognitiveMemory
from cognition_layer.tools.mutable_llm import MutableChatLLM
from cognition_layer.tools.ocr.image_edition import partial_image
from cognition_layer.tools.ocr.image_edition import resize_image
from ecm.exelent.builder import ExelentBuilder
from ecm.exelent.parser import ParsedTask
from ecm.mediator.feedback import ExecutionStatus
from ecm.mediator.feedback import Feedback
from ecm.mediator.Interpreter import Interpreter
from ecm.tools.item_registry_v2 import ItemRegistry

IMAGE_QUALITY = 0.4


@dataclass
class TaskSummary:
    success: bool = False
    result: Any = None


@dataclass
class VFRFeedbackStep:
    """
    Represents the response from the FastReact iterative process. It includes the
    name of the action, the content of the response, and a flag indicating whether
    it is the last response.

    :param name: The name of the action or step.
    :param content: The content of the response.
    :param is_last: Boolean indicating if this is the last response.
    """

    name: str
    content: str
    is_last: bool


class VFR_Response(BaseModel):
    reasoning: str = Field(description="A reasoning for about the next steps")
    function: str = Field(
        description="The next function with pythonic notation. E.g: myfunc(2, 3, 'foo'). Use 'Empty' if you don't want to call any function."  # noqa
    )


class VFR_CognitionState(BaseModel):
    goal: str = Field(description="The goal of the task")
    long_term_plan: str = Field(
        description="A long-term plan to achieve the goal. It can be a list of steps or a detailed plan."
    )
    scratchpad: str = Field(
        description="A scratchpad to store intermediate thoughts, calculations, or notes."
    )
    screen_focus: Literal[
        "center",
        "top left",
        "top",
        "top right",
        "right",
        "bottom right",
        "bottom",
        "bottom left",
        "left",
        "fullscreen",
    ] = Field(
        description="The location where you are zooming right now in the screen. Change it for changing the focus of the screenshots."  # noqa
    )
    objective_completed: bool = Field(
        description="A boolean indicating whether the objective has been completed."
    )


class MinimalVFR:

    @ItemRegistry.require_dependencies("screenshot", "meta")
    def __init__(
        self,
        interpreter: Interpreter = None,
        registry: ItemRegistry = ItemRegistry(),
        memory_capacity: int = 10,
    ):
        self.llm = MutableChatLLM().with_structured_output(VFR_Response)
        self.registry = registry
        self.interpreter = interpreter
        self.memory_capacity = memory_capacity
        self.latest_task: TaskSummary = None
        self.cognition_state = None
        self.formatted_tools = None
        self.memory = None

    def get_formatted_actions(self) -> list[str]:
        """
        Retrieves and formats the actions available in the registry.
        :return: A list of formatted action strings.
        """
        actions = [action.content for action in self.registry.actions.values()]
        tools: list[StructuredTool] = [build_tool(a) for a in actions]
        return [format_tool(t) for t in tools]

    def retrieve_task_result(self, feedback: Feedback):
        assert (
            self.latest_task is not None
        ), "No task has been initialized executed yet."
        self.latest_task: TaskSummary

        if feedback._exec_status == ExecutionStatus.SUCCESS:
            self.latest_task.success = True

        if feedback._exec_status == ExecutionStatus.RESULT:
            self.latest_task.result = feedback.object

    def _get_initial_cognition_state(self) -> CognitionState:
        cognition_state = CognitionState(VFR_CognitionState)
        initial_state = {
            "goal": input,
            "long_term_plan": "Empty",
            "scratchpad": "Empty",
            "screen_focus": "fullscreen",
            "objective_completed": False,
        }
        for key, value in initial_state.items():
            cognition_state.set(key, value)
        return cognition_state

    def _get_next_prompt(self) -> str:
        screenshot = load_image(self.registry.get("screenshot", type="tool").content())
        screenshot = partial_image(
            screenshot, position=self.cognition_state.get("screen_focus")
        )
        screenshot = resize_image(screenshot, IMAGE_QUALITY)

        instructions = MINVFR_PROMPT.format(
            tools=self.formatted_tools,
            history=get_buffer_string(self.memory.messages),
            cognition_state=self.cognition_state.summary(),
        )
        prompt = [ImageMessage(image=screenshot, input=instructions).as_human()]
        return prompt

    def _execute_response_from_agent(self, response: VFR_Response) -> None:
        if response.function != "Empty":
            task = _convert_response_to_exelent(response)
            self.latest_task = TaskSummary()
            self.interpreter.run(task, callback=self.retrieve_task_result)
            self.memory.update(
                [
                    AIMessage(content=str(response)),
                    SystemMessage(
                        content=f"`{response.function}` returned the following summary: `{self.latest_task}`"
                    ),
                ]
            )

    def complete_task(self, input: str) -> Generator[VFRFeedbackStep, None, None]:

        self.cognition_state = self._get_initial_cognition_state()
        self.formatted_tools = "\n\n- ".join(self.get_formatted_actions())
        self.memory = SimpleCognitiveMemory(
            capacity=self.memory_capacity, keep_images=False
        )
        self.memory.update([HumanMessage(content=input)])
        exit = False

        while not exit:

            prompt = self._get_next_prompt()
            response: VFR_Response = self.llm.invoke(prompt)

            self._execute_response_from_agent(response)

            self.cognition_state.set("scratchpad", response.reasoning)
            obj = self.cognition_state.get("objective_completed")
            exit = (obj) or (isinstance(obj, str) and obj.lower() == "true")

            yield VFRFeedbackStep(
                name="MinimalVFR", content=str(response), is_last=exit
            )


def _convert_response_to_exelent(
    frdict: VFR_Response, step_idx: int = random.randint(0, 1000)
) -> ParsedTask:
    """
    Converts a FastReact dictionary response into an Exelent task.

    :param frdict: The response dictionary from FastReact.
    :param step_idx: An optional index for the task step.
    :return: A ParsedTask representing the structured task.
    """
    builder = ExelentBuilder()
    builder.add_task(task_name=f"minimal_vfr_step_{step_idx}")
    builder.add_type("Sequential")
    builder.add_statement(frdict.function.replace("\\", "\\\\"))
    task = builder.compile()
    return task
