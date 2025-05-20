from typing import Generator

from langchain_core.messages import AIMessage
from langchain_core.messages import HumanMessage
from langchain_core.messages import SystemMessage
from pydantic import BaseModel
from pydantic import Field

from cognition_layer.experts.minimal_fastreact_expert.agent import (
    MinimalFastReactExpert,
)
from cognition_layer.protocols.fast_ap import FastAPStep
from cognition_layer.tools.mutable_llm import MutableChatLLM

PROMPT = """
You are CompositionRouter, an orchestration agent that solves user tasks by chaining calls
to specialized agents.

Your goals:
1. Break down the user's request into manageable subtasks.
2. Select and invoke the right experts
3. Verify each result to avoid hallucinations.
4. Continue until the task is fully completed or proven impossible.
5. Don't trust the experts blindly. Always verify their results.

You can call the following experts:
```
{experts}
```

Your last call must be `goal_completed` only when the task is fully completed or proven impossible.
"""


class CompositionRouterResponse(BaseModel):
    short_reasoning: str = Field(description="Short reasoning for the call")
    choice: str = Field(
        description="Name of the next expert to call (goal_completed if task is done)"
    )
    query: str = Field(description="Query to send to the next expert (if any)")


class CompositionRouter(MinimalFastReactExpert):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        model = kwargs.get("model", "gpt-4o-mini")
        self.llm = MutableChatLLM(model=model).with_structured_output(
            CompositionRouterResponse
        )

    def invoke(self, query: str) -> Generator[FastAPStep, None, None]:
        self.restart()
        self.memory.update([HumanMessage(content=query)])
        instructions = PROMPT.format(
            experts=self.formatted_experts,
        )

        exit = False
        while not exit:

            prompt = [SystemMessage(content=instructions)] + self.memory.messages
            response: CompositionRouterResponse = self.llm.invoke(prompt)

            if "goal_completed" in response.choice.lower():
                yield FastAPStep(
                    name="CompositionRouter",
                    content=response.short_reasoning,
                    is_last=True,
                )
                break

            yield FastAPStep(
                name="CompositionRouter",
                content=response.short_reasoning,
                is_last=False,
            )
            server = self.router.server(response.choice, interpreter=self.interpreter)
            result = None
            for step in server.send_task(response.query):
                result = step.content
                step.is_last = False
                yield step

            self.memory.update(
                [
                    AIMessage(
                        content=(
                            f"My reasoning was: {response.short_reasoning}, and i decided to "
                            f"call {response.choice} with the query: {response.query}"
                        )
                    ),
                    AIMessage(
                        content=f"The expert returned the following response: {result}"
                    ),
                ]
            )
