from dataclasses import dataclass
from typing import Generator

from langchain_core.messages import SystemMessage

from action_space.tools.image import ImageMessage
from action_space.tools.image import load_image
from cognition_layer.tools.mutable_llm import MutableChatLLM
from cognition_layer.tools.ocr.image_edition import resize_image
from ecm.tools.item_registry_v2 import ItemRegistry

IMAGE_QUALITY = 0.2


@dataclass
class SVAResponse:
    """
    Represents the response from the SmallVisionAgent process. It includes the
    name of the action, the content of the response, and a flag indicating whether
    it is the last response.

    :param name: The name of the action or step.
    :param content: The content of the response.
    :param is_last: Boolean indicating if this is the last response.
    """

    name: str
    content: str
    is_last: bool


class SmallVisionAgent:
    @ItemRegistry.require_dependencies("screenshot")
    def __init__(self, registry: ItemRegistry = ItemRegistry()):
        self.registry = registry
        self.llm = MutableChatLLM()

    def invoke(self, query: str) -> Generator[SVAResponse, None, None]:
        screenshot = load_image(self.registry.get("screenshot", type="tool").content())
        small_screenshot = resize_image(screenshot, IMAGE_QUALITY)
        prompt = [
            SystemMessage(
                "Give a short response to the user query based on the screenshot provided by the human."
            ),
            ImageMessage(
                small_screenshot,
                input=query,
            ).as_human(),
        ]
        response = self.llm.invoke(prompt)
        yield SVAResponse(
            name="SmallVisionAgent",
            content=response.content,
            is_last=True,
        )
