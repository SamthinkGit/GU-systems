"""
darkvfr
==============================
This module implements the DarkVFR agent, a variant of the MinimalVFR agent
designed for the cognition layer. DarkVFR introduces the ability to decide
where to look on the screen using tools provided by the "meta-look" package.
The agent only uploads screenshots when explicitly requested through the
Cognition State, which can be modified via the "meta" package.

Key Features:
- Uses "meta-look" to decide where to focus on the screen.
- Uploads screenshots only when explicitly requested by the agent.
- Reduces screenshot quality since additional tools are used for visual
  processing, minimizing the need for detailed screen information.

Constants:
- `PARTIAL_QUALITY`: Quality of partial screenshots.
- `FULLSCREEEN_QUALITY`: Quality of fullscreen screenshots.
"""
from langchain_core.messages import AIMessage
from langchain_core.messages import BaseMessage
from langchain_core.messages import SystemMessage
from langchain_core.messages.utils import get_buffer_string

from action_space.tools.image import ImageMessage
from action_space.tools.image import load_image
from cognition_layer.agents.minimal_vfr.agents.agent import MinimalVFR
from cognition_layer.agents.minimal_vfr.agents.prompt import MINVFR_PROMPT
from cognition_layer.tools.ocr.image_edition import partial_image
from cognition_layer.tools.ocr.image_edition import resize_image
from ecm.mediator.Interpreter import Interpreter
from ecm.tools.item_registry_v2 import ItemRegistry

# ======================= CONSTANTS ============================
ADDITIONAL_RULES_PROMPT = """
Note: Generally is a good idea to ask for a screenshot of the screen before starting the planning,
then, you should ask for a screenshot of the screen when actions fails and reason about why.
Note2: You cannot interact or wait to the user, you must be autonomous until completing the task or exiting.

"""
PARTIAL_QUALITY = 0.2
FULLSCREEEN_QUALITY = 0.1


# ======================= CLASSES ============================
class DarkVFR(MinimalVFR):
    """
    DarkVFR is a variant of the MinimalVFR agent with enhanced autonomy and
    the ability to decide where to focus on the screen using "meta-look".

    This class extends MinimalVFR by introducing the ability to request
    screenshots dynamically based on the Cognition State and by reducing
    screenshot quality to optimize resource usage.
    """

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
        """
        Initializes the DarkVFR agent.

        Args:
            interpreter (Interpreter): The interpreter for executing tasks.
            registry (ItemRegistry): The registry containing tools and actions.
            memory_capacity (int): The capacity of the agent's memory.
        """
        self.image_requested = False
        return super().complete_task(input)

    def _get_next_prompt(self) -> list[BaseMessage]:
        """
        Generates the next prompt for the AI based on the current state.
        This method loads the default MinimalVFR prompt only adding the
        screenshot if requested.
        Returns:
            list[BaseMessage]: A list of messages forming the prompt.
        """
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
                else PARTIAL_QUALITY
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
        """
        Executes the response received from the AI agent.

        If the response includes a request to "look_to_screen", a screenshot
        is requested and added to the memory. Otherwise, the response is
        processed by MinimalVFR.

        Args:
            response (VFR_Response): The structured response from the AI.
        """
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
