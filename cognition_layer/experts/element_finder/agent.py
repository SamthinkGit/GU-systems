from typing import Generator

from langchain_core.messages import HumanMessage
from langchain_core.messages import SystemMessage

from cognition_layer.deploy.loader import get_deploy_model
from cognition_layer.experts.minimal_fastreact_expert.agent import custom_schema
from cognition_layer.experts.minimal_fastreact_expert.agent import DeploySchema
from cognition_layer.experts.minimal_fastreact_expert.agent import MfreResponse
from cognition_layer.experts.minimal_fastreact_expert.agent import (
    MinimalFastReactExpert,
)
from cognition_layer.protocols.fast_ap import FastAPStep
from ecm.shared import get_logger
from ecm.tools.item_registry_v2 import ItemRegistry

PROMPT = """
You are an expert in finding elements on the screen.
Your goal is to analyze the image according to the main goal and obtain the exact COORDINATES of the requested element.

You can use the following tools:
```
{tools}
```

Additionally you can call other experts:
```
{experts}
```

Use the following rules:

For analyzing the screen:
- If you need specific information (such as button descriptions, texts, elements, etc.) you can call the expert with natural language questions using the function call_expert.

For finding coordinates:
- Use the function read_screen to get the coordinates of all texts in screen. ¡Careful! Multiple texts can be similar, ensure to get the most relevant one according to the user query.
- Use obtain_coordinates with a visual description of a non-text element (visual, icon, etc) to retrieve its coordinates. However this method usually fails if its not a big icon, image or visual element.

WARNING: You must satisfy the user using at most {K} steps.
THE FINAL ANSWER MUST SATISFY THIS: The final answer will use the function `<stop_analysis>` and return about the coordinates (and a reasoning) in the reasoning field.
"""  # noqa


class ElementFinder(MinimalFastReactExpert):

    _logger = get_logger("ElementFinder")

    @ItemRegistry.require_dependencies(
        "moondream_element_finder",
        "expert-calling",
        "simple-read-ocr",
        "screenshot",
    )
    def __init__(
        self,
        max_steps: int = 3,
        threshold: float = 6,
        schema: DeploySchema = None,
        model: str = "gpt-4o-mini",
        *args,
        **kwargs,
    ):
        self.threshold = threshold
        self.max_steps = max_steps
        gpt_vision = get_deploy_model("small-vision-agent")
        gpt_vision["agent_description"] = "A small nlp vision agent."
        gpt_vision["use_case"] = "Use it to analyze the screen by sending questions."

        new_schema = custom_schema([gpt_vision])
        super().__init__(*args, **kwargs, schema=new_schema, model=model)

    def invoke(self, query: str) -> Generator[FastAPStep, None, None]:
        self.restart()

        self.memory.update([HumanMessage(content=query)])
        instructions = PROMPT.format(
            tools=self.formatted_tools,
            experts=self.formatted_experts,
            K=self.max_steps,
        )
        self._logger.debug(f"Starting localization with description: `{query}`")

        for _ in range(self.threshold):

            prompt = [SystemMessage(content=instructions)] + self.memory.messages
            response: MfreResponse = self.llm.invoke(prompt)

            if "stop_analysis" in response.function:
                self._logger.debug(f"Final response: {response}")
                yield FastAPStep(
                    name="ElementFinder",
                    content=response.reasoning,
                    is_last=True,
                )

            self._logger.debug(f"Response: {response}")
            yield FastAPStep(
                name="ElementFinder",
                content=response.reasoning,
                is_last=False,
            )
            self.execute_response(response)
