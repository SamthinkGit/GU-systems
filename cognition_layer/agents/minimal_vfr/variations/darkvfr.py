from langchain_core.messages import AIMessage
from langchain_core.messages import BaseMessage
from langchain_core.messages import SystemMessage
from langchain_core.messages.utils import get_buffer_string

from action_space.tools.image import ImageMessage
from action_space.tools.image import load_image
from cognition_layer.agents.minimal_vfr.agents.agent import IMAGE_QUALITY
from cognition_layer.agents.minimal_vfr.agents.agent import MinimalVFR
from cognition_layer.agents.minimal_vfr.agents.prompt import MINVFR_PROMPT
from cognition_layer.tools.ocr.image_edition import partial_image
from cognition_layer.tools.ocr.image_edition import resize_image
from ecm.mediator.Interpreter import Interpreter
from ecm.tools.item_registry_v2 import ItemRegistry

ADDITIONAL_RULES_PROMPT = """
Note: Generally is a good idea to ask for a screenshot of the screen before starting the planning,
then, you should ask for a screenshot of the screen when actions fails and reason about why.
"""
FULLSCREEEN_QUALITY = 0.1


class DarkVFR(MinimalVFR):

    @ItemRegistry.require_dependencies("meta-look")
    def __init__(
        self,
        interpreter: Interpreter = None,
        registry: ItemRegistry = ItemRegistry(),
        memory_capacity: int = 10,
    ):
        self.image_requested = False
        super().__init__(interpreter, registry, memory_capacity)

    def complete_task(self, input):
        self.image_requested = False
        return super().complete_task(input)

    def _get_next_prompt(self) -> list[BaseMessage]:
        instructions = MINVFR_PROMPT.format(
            tools=self.formatted_tools,
            history=get_buffer_string(self.memory.messages),
            cognition_state=self.cognition_state.summary(),
            additional_rules=ADDITIONAL_RULES_PROMPT,
        )
        prompt = [SystemMessage(content=instructions)]
        if self.image_requested:
            screenshot = load_image(
                self.registry.get("screenshot", type="tool").content()
            )
            screenshot = partial_image(
                screenshot, position=self.cognition_state.get("screen_focus")
            )
            quality = (
                FULLSCREEEN_QUALITY
                if self.cognition_state.get("screen_focus") == "fullscreen"
                else IMAGE_QUALITY
            )
            screenshot = resize_image(screenshot, quality)
            prompt.append(
                ImageMessage(
                    image=screenshot,
                    input="As requested, here it is a screenshot of my screen.",
                ).as_human()
            )
            self.image_requested = False

        return prompt

    def _execute_response_from_agent(self, response):
        if "look_to_screen" in response.function:
            self.image_requested = True
            self.memory.update(
                [
                    AIMessage(content=str(response)),
                    SystemMessage(
                        content=f"`{response.function}` returned the following summary: `Image succesfully shown to AI`"
                    ),
                ]
            )
            return

        super()._execute_response_from_agent(response)
