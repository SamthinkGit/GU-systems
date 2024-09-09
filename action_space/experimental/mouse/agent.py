"""
MouseAgent
========================

The Mouse agent is responsible of using the MouseDescriptor
tool by receiving an image and selecting the appropiate cell
for a given element.

Thus, this agent will iterate until finding the cell corresponding
the described element, returning the absolute coordinates obtained
using the Grid calculations.

Note: For better results, the agent first realices a description of
the screen.
"""
import base64
from io import BytesIO

from langchain.pydantic_v1 import BaseModel
from langchain.pydantic_v1 import Field
from langchain_core.messages import BaseMessage
from langchain_core.messages import HumanMessage
from langchain_core.messages import SystemMessage
from langchain_core.runnables import Runnable
from langchain_openai import ChatOpenAI
from PIL import Image

from action_space.experimental.mouse.controller import MouseDescriptor
from cognition_layer.constants import DEFAULT_IMAGE_MODEL
from ecm.shared import get_logger

SCREEN_DESCRIPTION_PROMPT = """
You are an assistant designed to receive an image of a computer screen and a
user's request regarding a specific element within that screen. Your task is to
generate a detailed description of the element the user is interested in, by
analyzing the content and context of the screen image, which may include
multiple open windows, visible applications, and various visible elements.
For this goal, ensure which icon/button the user is asking for and describe
graphically the visual features of the item in order to be easily visually
identified.

Assume that the icon or button request is in the image, so select the most
probable place and describe it as stated in the guidelines. Only describe
graphical properties of the element in the image.

If the target element contains any text, specify literally as it is so it is
easier to identify.

The output response should not contain more than 200 words.
"""
ELEMENT_DETECTION_PROMPT = """
You are an assistant tasked with identifying the number within a grid cell from
a computer screen screenshot. When presented with an image, analyze it to detect
the grid lines and numerals within each cell.

Upon receiving a coordinate or a point of interest from a user, determine the
specific grid cell this point corresponds to. Accurately return the numeral
associated with that cell.

Advise: Note that if a label is received from the user, you must find exactly the
cell that contains that exact text match.
"""


class DescriptionResponse(BaseModel):
    item: str = Field(
        description="A comprehensive description of the element according to the guidelines."
    )
    position: str = Field(
        description="A comprehensive description of the position of the element and visual marks."
    )
    label: str = Field(
        description="Text labels or visual tags to identify the element (if any)."
    )


class FinderResponse(BaseModel):
    reasoning: str = Field(
        description="A comprehensive reasoning according to the description of the element "
        "referencing the original description and looking at the image provided.",
        min_length=100,
    )
    cell: int = Field(
        description="Number corresponding to the cell of the grid which contains the user element."
    )


class MouseAgent:

    llm: Runnable
    _built = False
    _logger = get_logger("MouseAgent")

    @classmethod
    def _build(cls):
        cls.llm = ChatOpenAI(model=DEFAULT_IMAGE_MODEL, temperature=0)
        cls._built = True

    @staticmethod
    def _upload_from_image(image: Image.Image):
        buffered = BytesIO()
        image.save(buffered, format="PNG")
        img_byte = buffered.getvalue()
        return base64.b64encode(img_byte).decode("utf-8")

    @staticmethod
    def _upload_from_path(image_path):
        with open(image_path, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode("utf-8")

    @classmethod
    def _invoke_descriptor(cls, query: str, image: bytes) -> BaseMessage:

        cls._verify_building()
        response = cls.llm.with_structured_output(DescriptionResponse).invoke(
            [
                SystemMessage(content=SCREEN_DESCRIPTION_PROMPT),
                HumanMessage(
                    content=[
                        {"type": "text", "text": query},
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{image}",
                                "detail": "high",
                            },
                        },
                    ]
                ),
            ]
        )
        return response

    @classmethod
    def _invoke_finder(
        cls, query: str, image: bytes, additional_sys_prompt: str = ""
    ) -> BaseMessage:
        cls._verify_building()
        response = cls.llm.with_structured_output(FinderResponse).invoke(
            [
                SystemMessage(content=ELEMENT_DETECTION_PROMPT + additional_sys_prompt),
                HumanMessage(
                    content=[
                        {"type": "text", "text": query},
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{image}",
                                "detail": "high",
                            },
                        },
                    ]
                ),
            ]
        )
        return response

    @classmethod
    def find(cls, element: str, iterations: int = 2):
        cls._verify_building()
        cls._logger.debug("Starting MouseAgent tool...")
        if iterations < 1:
            return SystemError("Find should use at least one iteration")

        grid = MouseDescriptor.start()

        image = cls._upload_from_image(grid.original_image)
        description = cls._invoke_descriptor(
            query=f"Describe where could be the element {element}", image=image
        )

        cls._logger.debug(f"Description of the Searched element: {str(description)}")

        image = cls._upload_from_image(grid.image)
        response = cls._invoke_finder(
            query=f"The number of the cell where is located the element: `{element}` "
            f"with description: {description.item}, position: {description.position}"
            f"and the attached label (optional): {description.label}",
            image=image,
        )

        result = response.cell
        for _ in range(iterations - 1):
            grid = MouseDescriptor.zoom(result)

            image = cls._upload_from_image(grid.image)
            response = cls._invoke_finder(
                query=f"The number of the cell where is located the element: `{element}` "
                f"with description: {description.item}, "
                f"and the attached label (optional): {description.label}",
                image=image,
            )
            result = response.cell

        MouseDescriptor.zoom(result)
        cls._logger.debug(
            f"Obtained pixel at [{MouseDescriptor._current_cell.center_x}, {MouseDescriptor._current_cell.center_y}]"
        )
        MouseDescriptor.move()

    @classmethod
    def _verify_building(cls):
        if not cls._built:
            cls._build()


if __name__ == "__main__":
    import sys

    MouseAgent.find(sys.argv[1])
