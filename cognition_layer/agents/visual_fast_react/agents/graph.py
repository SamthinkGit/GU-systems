import random
from typing import Callable
from typing import Literal
from typing import TypedDict

from langchain.tools import tool as build_tool
from langchain_core.messages import AIMessage
from langchain_core.messages import SystemMessage
from langchain_core.messages.utils import get_buffer_string
from langchain_core.tools import StructuredTool
from langgraph.graph import StateGraph
from PIL.Image import Image
from pydantic import BaseModel
from pydantic import Field

from action_space.tools.image import ImageMessage
from action_space.tools.image import load_image
from cognition_layer.memory.simple import SimpleCognitiveMemory
from cognition_layer.planex.utils.format import format_tool
from cognition_layer.tools.mutable_llm import MutableChatLLM
from cognition_layer.tools.ocr.engine import OCR
from cognition_layer.tools.ocr.image_edition import partial_image
from cognition_layer.tools.ocr.image_edition import resize_image
from cognition_layer.tools.ocr.labeler import Labeler
from cognition_layer.visual_fast_react.agents.prompt import FastReactPrompt
from cognition_layer.visual_fast_react.agents.prompt import VisualAnalyzerPrompt
from ecm.exelent.builder import ExelentBuilder
from ecm.exelent.parser import ParsedTask
from ecm.mediator.Interpreter import Interpreter
from ecm.tools.item_registry_v2 import ItemRegistry

FULLSCREEN_QUALITY_REDUCTION = 0.5
PARTIAL_SCREEN_QUALITY_REDUCTION = 0.6


class VisualAnalyzerResponse(BaseModel):
    description_of_the_screen: str = Field(
        description="Description of the relevant information (related to the goal) in the screen."
    )
    reasoning: str = Field(description="A reasoning for solving the task")
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
    ] = Field(description="The location where you will interact in the next action.")
    summary: str = Field(
        description="A summary of this response using natural language (include description, reasoning and focus)"
    )


class FastReactResponse(BaseModel):
    reasoning: str = Field(description="A reasoning for about the next steps")
    function: str = Field(
        description="The next function with pythonic notation. E.g: myfunc(2, 3, 'foo')"
    )
    next_state: Literal["finish", "maintain_focus", "change_focus", "replanning"] = (
        Field(description="The next state of the agent.")
    )
    summary: str = Field(
        description="A summary of this response using natural language (include reasoning, function used and next state)"  # noqa
    )


class VfrState(TypedDict):
    registry: ItemRegistry = Field(description="Registry with actions/tools")
    query: str = Field(description="Original user query")
    memory: SimpleCognitiveMemory = Field(
        description="Memory object to store information."
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
    ] = Field(description="The current focus on the screen")
    ocr: OCR = Field(description="Image Local Analyzer")
    fr_next_state: Literal["finish", "maintain_focus", "change_focus", "replanning"] = (
        Field(description="Next state from fast react node")
    )
    interpreter: Interpreter = Field(
        description="Interpreted used to run tools/actions"
    )
    latest_summary: str = Field(description="Summaries for debugging purpouses")
    is_last: bool = Field(description="Is the last step")


def visual_analyzer_node(state: VfrState) -> dict:

    llm = MutableChatLLM().with_structured_output(VisualAnalyzerResponse)
    history_str = get_buffer_string(state["memory"].messages)

    screenshot_tool: Callable = state["registry"].get("screenshot", type="tool").content
    screenshot: Image = load_image(screenshot_tool())
    screenshot = resize_image(screenshot, FULLSCREEN_QUALITY_REDUCTION)

    prompt = [
        ImageMessage(
            image=screenshot,
            input=VisualAnalyzerPrompt.format(
                original_query=state["query"], history=history_str
            ),
        ).as_human()
    ]

    response: VisualAnalyzerResponse = llm.invoke(prompt)
    state["memory"].update([AIMessage(content=response.summary)])
    return {"screen_focus": response.screen_focus, "latest_summary": response.summary}


def fast_react_node(state: VfrState) -> dict:
    llm = MutableChatLLM().with_structured_output(FastReactResponse)
    history_str = get_buffer_string(state["memory"].messages)

    screenshot_tool: Callable = state["registry"].get("screenshot", type="tool").content
    screenshot: Image = load_image(screenshot_tool())
    cropped_image = partial_image(screenshot, position=state["screen_focus"])

    actions = [action.content for action in state["registry"].actions.values()]
    tools: list[StructuredTool] = [build_tool(a) for a in actions]
    formatted_tools = "\n\n- ".join([format_tool(t) for t in tools])

    bboxes = state["ocr"].invoke(cropped_image)
    labeler = Labeler(cropped_image, bboxes)
    labeler.freeze()

    text_descriptions: list[str] = []
    for label, bbox in labeler.labelled_boxes.items():
        if bbox.additional_info.get("type", "null") == "text":
            text_descriptions.append(f"'{label}': '{bbox.content}'")

    icon_board = labeler.board(
        filter=lambda bbox: bbox.additional_info.get("type", "null") == "icon"
    )
    text_labels_description = "\n".join(text_descriptions)

    prompt = [
        SystemMessage(
            content=FastReactPrompt.format(
                screen_focus=state["screen_focus"],
                tools=formatted_tools,
                original_user_query=state["query"],
                history=history_str,
            )
        ),
        ImageMessage(
            image=resize_image(cropped_image, PARTIAL_SCREEN_QUALITY_REDUCTION),
            input="This is the focused area of the screen",
            detail="high",
        ).as_human(),
        ImageMessage(
            image=icon_board,
            input=(
                f"Additionally, the following <text> labels were detected.\nYou can use:\n\n{text_labels_description}."  # noqa: E501
                "\nI also append an image with the correspondent labels for each clickable <icon> detected."
                "\nYou can use any of these labels to click on them. If you only want to write or use keyboard you can ignore them."  # noqa: E501
            ),
            detail="high",
        ).as_human(),
    ]

    response: FastReactResponse = llm.invoke(prompt)
    state["memory"].update([AIMessage(content=response.summary)])

    if not response.next_state == "replanning":
        task = _convert_response_to_exelent(response)
        state["interpreter"].run(task)

    return {
        "fr_next_state": response.next_state,
        "latest_summary": response.summary,
        "is_last": response.next_state == "finish",
    }


def should_refocus_edge(
    state: VfrState,
) -> Literal["fast_react_node", "visual_node", "__end__"]:

    if state["fr_next_state"] == "finish":
        return "__end__"

    if state["fr_next_state"] == "maintain_focus":
        return "fast_react_node"

    return "visual_node"


def _convert_response_to_exelent(
    response: FastReactResponse, step_idx: int = random.randint(0, 1000)
) -> ParsedTask:
    """
    Converts a FastReact dictionary response into an Exelent task.

    :param frdict: The response dictionary from FastReact.
    :param step_idx: An optional index for the task step.
    :return: A ParsedTask representing the structured task.
    """
    builder = ExelentBuilder()
    builder.add_task(task_name=f"vfr_step_{step_idx}")
    builder.add_type("Sequential")
    builder.add_statement(response.function.replace("\\", "\\\\"))
    task = builder.compile()
    return task


def compile_visual_fast_react_graph():

    graph_builder = StateGraph(VfrState)
    graph_builder.add_node("visual_node", visual_analyzer_node)
    graph_builder.add_node("fast_react_node", fast_react_node)

    graph_builder.set_entry_point("visual_node")
    graph_builder.add_edge("visual_node", "fast_react_node")
    graph_builder.add_conditional_edges("fast_react_node", should_refocus_edge)

    return graph_builder.compile()
