"""
FastReact Module
==============================
This module implements the FastReact class, which facilitates an iterative
process to complete tasks using a cognitive memory and a set of tools. It
uses the ChatOpenAI model and integrates with various components to manage
tool actions and responses effectively.
"""
import random
from dataclasses import dataclass
from typing import Generator
from typing import Literal

from langchain.tools import tool as build_tool
from langchain_core.messages import AIMessage
from langchain_core.messages import BaseMessage
from langchain_core.messages import HumanMessage
from langchain_core.messages import SystemMessage
from langchain_core.tools import StructuredTool

from action_space.tools.image import ImageMessage
from action_space.tools.image import load_image
from cognition_layer.fast_react.agents.prompt import DictFastReactResponse
from cognition_layer.fast_react.agents.prompt import FastReactParser
from cognition_layer.fast_react.agents.prompt import FR_PROMPT
from cognition_layer.memory.simple import SimpleCognitiveMemory
from cognition_layer.planex.utils.format import format_tool
from cognition_layer.tools.mutable_llm import MutableChatLLM
from cognition_layer.tools.ocr.engine import OCR
from cognition_layer.tools.ocr.labeler import Labeler
from ecm.exelent.builder import ExelentBuilder
from ecm.exelent.parser import ParsedTask
from ecm.mediator.Interpreter import Interpreter
from ecm.tools.item_registry_v2 import ItemRegistry


# ======================= CLASSES ============================
@dataclass
class FastReactResponse:
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


class FastReact:
    """
    Class that manages the FastReact iterative process.

    It uses an interpreter and a registry to perform actions, maintains a memory
    of interactions, and processes tasks through a sequence of formatted actions.
    """

    def __init__(
        self,
        interpreter: Interpreter = None,
        registry: ItemRegistry = ItemRegistry(),
        memory_capacity: int = 10,
        ocr_mode: bool = False,
        ocr_image_period: int = 3,
        ocr_type: Literal["text", "labelled"] = "labelled",
    ):
        """
        Initializes the FastReact instance.

        :param interpreter: An instance of Interpreter for processing tasks.
        :param registry: An instance of ItemRegistry for managing tools.
        :param memory_capacity: Maximum number of messages to keep in memory.
        """
        self.chain = MutableChatLLM() | FastReactParser
        self.memory_capacity = memory_capacity
        self.interpreter = interpreter
        self.registry = registry
        self.ocr_mode = ocr_mode
        self._ocr_round = 0
        self.ocr_image_period = ocr_image_period
        self.ocr_type = ocr_type

        self.ocr = None
        if self.ocr_mode:
            self.ocr = OCR()

    def get_formatted_actions(self) -> list[str]:
        """
        Retrieves and formats the actions available in the registry.

        :return: A list of formatted action strings.
        """

        actions = [action.content for action in self.registry.actions.values()]
        tools: list[StructuredTool] = [build_tool(a) for a in actions]
        return [format_tool(t) for t in tools]

    def complete_task(self, input: str) -> Generator[FastReactResponse, None, None]:
        """
        Completes a given task iteratively using available tools and cognitive memory.

        - Asserts that the screenshot tool is present in the registry.
        - Formats the tools and maintains task completion state across iterations.

        :param input: The task input to be processed.
        :yield: FastReactResponse objects containing task completion states.
        """

        assert (
            "screenshot.screenshot" in self.registry.tools
        ), "Screenshot tool must be loaded into the registry in order to use FastReact"

        self._ocr_round = 0

        keep_images = False
        if self.ocr_mode and self.ocr_type == "text":
            keep_images = True

        formatted_tools = "\n\n- ".join(self.get_formatted_actions())
        self.memory = SimpleCognitiveMemory(
            keep_images=keep_images,
            capacity=self.memory_capacity,
            preserve=[
                SystemMessage(content=FR_PROMPT),
                SystemMessage(
                    content=f"The current available actions are:\n\n- {formatted_tools}\n"
                ),
                HumanMessage(
                    f"The task you must complete using the JSON format is: ```{input}```"
                ),
            ],
        )

        task_completed = False
        num_actions = 0
        while not task_completed:

            current_state = self._get_current_state()
            history = self.memory.messages + current_state
            response: DictFastReactResponse = self.chain.invoke(history)
            formatted_response = f"```json\n{response}\n```"

            task_completed = response["all_tasks_completed"]
            self.memory.update([AIMessage(content=formatted_response)])

            task = _convert_frdict_to_exelent(response, step_idx=num_actions)
            self.interpreter.run(task)

            yield FastReactResponse(
                name="FastReact", content=str(response), is_last=task_completed
            )
            num_actions += 1

    def _get_current_state(self) -> list[BaseMessage]:
        """
        Retrieves the current state of the task.
        :return: An ImageMessage representing the current state.
        """
        frame = load_image(self.registry.get("screenshot", "tool").content())
        if self.ocr_type == "text":

            if self.ocr_mode:

                labels = self.ocr.invoke(frame)

                explained_labels = (
                    "This is my current state.\n"
                    + "Labels detected on screen:\n"
                    + "\n".join(
                        [f"- `{label.content}` at [{label.center}]" for label in labels]
                    )
                )

                if self._ocr_round == 0:
                    message = ImageMessage(
                        input=explained_labels + "\nI append an image of my screen with the text's labelled.",
                        image=frame,
                        detail="low",
                    ).as_human()
                else:
                    message = HumanMessage(content=explained_labels)

                self._ocr_round += 1
                if self._ocr_round >= self.ocr_image_period:
                    self._ocr_round = 0

                return [message]

            current_state = ImageMessage(
                input="This is the current state", image=frame
            ).as_human()
            return [current_state]

        elif self.ocr_type == "labelled":
            bboxes = self.ocr.invoke(frame)
            labeler = Labeler(frame, bboxes)
            labeler.freeze()

            text_descriptions: list[str] = []
            for label, bbox in labeler.labelled_boxes.items():
                if bbox.additional_info.get("type", "null") == "text":
                    text_descriptions.append(f"'{label}': '{bbox.content}'")

            icon_board = labeler.board(
                filter=lambda bbox: bbox.additional_info.get("type", "null") == "icon"
            )

            text_labels_description = "\n".join(text_descriptions)

            labels_message = ImageMessage(
                image=icon_board,
                input=(
                    f"The following <text> labels were detected:\n\n{text_labels_description}."
                    "\nI also append an image with the correspondent labels for each clickable <icon> detected."
                    "\nYou can use any of these labels to click on them."
                ),
                detail="high",
            ).as_human()

            state_message = ImageMessage(
                input="This is my current state, ensure to analyse what you see in your reasoning",
                image=frame,
                detail="low",
            ).as_human()
            return [state_message, labels_message]


# ======================= UTILITIES ============================
def _convert_frdict_to_exelent(
    frdict: DictFastReactResponse, step_idx: int = random.randint(0, 1000)
) -> ParsedTask:
    """
    Converts a FastReact dictionary response into an Exelent task.

    :param frdict: The response dictionary from FastReact.
    :param step_idx: An optional index for the task step.
    :return: A ParsedTask representing the structured task.
    """
    builder = ExelentBuilder()
    builder.add_task(task_name=f"fr_step_{step_idx}")
    builder.add_type("Sequential")
    builder.add_statement(frdict["function"].replace("\\", "\\\\"))
    task = builder.compile()
    return task
