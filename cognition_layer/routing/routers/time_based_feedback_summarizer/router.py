import time
from typing import Generator

from langchain_core.messages import HumanMessage
from langchain_core.messages import SystemMessage

from cognition_layer.protocols.fast_ap import FastAPStep
from cognition_layer.routing.routers.simple_router.simple_router import DeploySchema
from cognition_layer.routing.routers.simple_router.simple_router import SimpleRouter
from cognition_layer.tools.mutable_llm import MutableChatLLM
from ecm.mediator.Interpreter import Interpreter

PROMPT = """
Generate a summary of the actions and reasonings performed in the following messages.
You must eliminate the agent names and always talk in first person, as if you were the one reasoning and acting.
"""


class TimeBasedFeedbackSummarizer:

    def __init__(
        self,
        schema: DeploySchema,
        interpreter: Interpreter,
        delay: int = 5,
        disable: bool = False,
        model: str = "gpt-4.1-nano",
        *args,
        **kwargs,
    ):
        self.interpreter = interpreter
        self.delay = delay
        self.disabled = disable
        self.router = SimpleRouter(schema)
        self.llm = MutableChatLLM(model=model)
        self.routered_agent = list(self.router.agents.keys())[0]

        assert (
            len(self.router.agents) == 1
        ), "TimeBasedFeedbackSummarizer only one worker."

    def invoke(self, query: str) -> Generator[FastAPStep, None, None]:
        server = self.router.server(self.routered_agent, self.interpreter)

        if server is None:
            yield FastAPStep(
                name="TimeBasedFeedbackSummarizer",
                content="Couldnt route to a worker.",
                is_last=True,
            )
            return

        step_queue = []
        frame = time.perf_counter()

        for step in server.send_task(query):

            if self.disabled:
                yield step
                continue

            step_queue.append(step)
            if step.is_last or time.perf_counter() - frame > self.delay:
                prompt = [
                    SystemMessage(PROMPT),
                    HumanMessage("\n\n".join([step.content for step in step_queue])),
                ]
                summary = self.llm.invoke(prompt).content
                yield FastAPStep(
                    name="TimeBasedFeedbackSummarizer",
                    content=summary,
                    is_last=step.is_last,
                )
                step_queue = []
                frame = time.perf_counter()
