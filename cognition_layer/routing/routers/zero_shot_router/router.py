import json
from typing import Generator

from langchain_core.messages import HumanMessage
from langchain_core.messages import SystemMessage
from pydantic import BaseModel
from pydantic import Field

from cognition_layer.deploy.types import DeploySchema
from cognition_layer.protocols.fast_ap import FastAPStep
from cognition_layer.routing.routers.simple_router.simple_router import SimpleRouter
from cognition_layer.tools.mutable_llm import MutableChatLLM
from ecm.mediator.Interpreter import Interpreter
from ecm.shared import get_logger


class ZeroShotRouterResponse(BaseModel):
    reasoning: str = Field(
        description="The reasoning behind the choice of agent. (Use one phrase at max)",
    )
    choice: str = Field(
        description="The name of the agent chosen to handle the request.",
    )


class ZeroShotRouter:

    _logger = get_logger("ZeroShotRouter")

    def __init__(self, schema: DeploySchema, interpreter: Interpreter, **kwargs):
        self.interpreter = interpreter
        self.llm = MutableChatLLM().with_structured_output(ZeroShotRouterResponse)
        self.router = SimpleRouter(schema)
        summary = self.router.summary_dict()
        self.prompt = (
            "Select the best agent to handle the user query:"
            f"\n\n{json.dumps(summary, indent=2)}\n\n"
        )

    def invoke(self, query: str) -> Generator[ZeroShotRouterResponse, None, None]:
        prompt = [SystemMessage(self.prompt), HumanMessage(query)]
        response: ZeroShotRouterResponse = self.llm.invoke(prompt)
        self._logger.debug(f"Reasoning: {response.reasoning}")
        self._logger.debug(f"Choice: {response.choice}")

        server = self.router.server(response.choice, self.interpreter)

        if server is None:
            yield FastAPStep(
                name="ZeroShotRouter",
                content="Couldnt route to an expert agent.",
                is_last=True,
            )
            return

        yield from server.send_task(query)
