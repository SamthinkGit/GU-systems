import time
from typing import Callable

from langchain.pydantic_v1 import BaseModel
from langchain.pydantic_v1 import Field
from langchain_core.messages import SystemMessage
from langchain_core.prompts import ChatPromptTemplate
from langchain_openai import ChatOpenAI

from action_space.tools.image import ImageMessage
from action_space.tools.image import load_image
from cognition_layer.constants import DEFAULT_IMAGE_MODEL
from ecm.shared import get_logger
from ecm.tools.registry import ItemRegistry


class WaitForOutput(BaseModel):
    reasoning: str = Field(
        description="Short description of why the image shows that the condition of the user has "
        "been accomplished."
    )
    response: bool = Field(
        description=(
            "Return true if the user condition has been accomplished according to the "
            "image provided. Else false."
        )
    )


class WaitForAgent:

    _logger = get_logger(name="WaitForAgent")

    def __init__(self, screenshot_tool: Callable) -> None:
        self.screenshot_tool = screenshot_tool
        self.llm = ChatOpenAI(
            model=DEFAULT_IMAGE_MODEL, temperature=0
        ).with_structured_output(WaitForOutput)

    def wait_for(
        self, condition: str, sleep_time_seconds: float = 2, max_iterations: int = 5
    ):
        goal_reached = False
        iterations = 0
        while not goal_reached:

            if iterations >= max_iterations:
                raise RuntimeError(
                    "Maximum waiting time for the provided condition has been reached."
                )

            image = load_image(self.screenshot_tool())
            prompt = ChatPromptTemplate.from_messages(
                [
                    SystemMessage(
                        content="You are a verification AI assistant. Your goal is to determine "
                        "wether the condition asked by the user has been reached in the image "
                        "provided from the user."
                    ),
                    ImageMessage(
                        image=image,
                        input=f"Has this condition been reached?:\n```{condition}```\n",
                    ).as_human(),
                ]
            )
            chain = prompt | self.llm
            output: WaitForOutput = chain.invoke({})
            goal_reached: bool = output.response

            if not goal_reached:
                self._logger.debug(
                    f"Condition `{condition}` has not been reached. Sleeping..."
                )
                time.sleep(sleep_time_seconds)

            else:
                self._logger.debug(
                    f"Condition `{condition}` has been reached. Releasing Sleep..."
                )

import action_space.experimental.screenshot.actions  # noqa

DEFAULT_WAIT_FOR_AGENT = WaitForAgent(screenshot_tool=ItemRegistry._utils["screenshot"])
