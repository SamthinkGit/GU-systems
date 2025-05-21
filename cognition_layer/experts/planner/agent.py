from typing import Generator

from action_space.tools.image import ImageMessage
from action_space.tools.image import load_image
from cognition_layer.protocols.fast_ap import FastAPStep
from cognition_layer.tools.mutable_llm import MutableChatLLM
from cognition_layer.tools.ocr.image_edition import resize_image
from ecm.tools.registry import ItemRegistry


PROMPT = """
Based on the image provided, generate a step by step plan for obtaining the next goal: `{goal}`.
Include the following information:
1. The next action to take.
2. Clicks, actions, writings or any other action that you will do.
3. Posible failures or assumptions that you are making.
"""


class PlannerExpert:

    @ItemRegistry.require_dependencies("screenshot")
    def __init__(
        self,
        registry: ItemRegistry,
        model: str = "gpt-4.1-nano",
        quality: float = 0.3,
        *args,
        **kwargs
    ):
        self.registry = registry
        self.llm = MutableChatLLM(model=model)
        self.quality = quality

    def invoke(self, query: str) -> Generator[FastAPStep, None, None]:
        image = load_image(self.registry.get("screenshot", type="tool").content())
        image = resize_image(image, self.quality)

        instructions = PROMPT.format(goal=query)
        response = self.llm.invoke(
            [
                ImageMessage(
                    image=image,
                    input=instructions,
                ).as_human()
            ]
        )
        yield FastAPStep(
            name="PlannerExpert",
            content=response,
            is_last=True,
        )
