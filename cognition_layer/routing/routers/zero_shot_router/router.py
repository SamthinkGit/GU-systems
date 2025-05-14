import json
import uuid
from typing import Generator

from langchain_core.messages import HumanMessage
from langchain_core.messages import SystemMessage
from langchain_core.prompts import ChatPromptTemplate
from pydantic import BaseModel
from pydantic import Field

from cognition_layer.deploy.loader import deploy
from cognition_layer.deploy.loader import get_deploy_model
from cognition_layer.deploy.types import DeploySchema
from cognition_layer.protocols.fast_ap import FastAPStep
from cognition_layer.tools.mutable_llm import MutableChatLLM
from ecm.mediator.Interpreter import Interpreter
from ecm.shared import get_logger
from ecm.tools.item_registry_v2 import ItemRegistry


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
        llm = MutableChatLLM().with_structured_output(ZeroShotRouterResponse)

        self.agents = {
            agent["name"]: (agent, get_deploy_model(agent["model"]))
            for agent in schema["workers"]
        }

        summaries = [
            {
                "name": name,
                "description": model["agent_description"],
                "use_case": model["use_case"],
            }
            for name, (_, model) in self.agents.items()
        ]

        prompt = (
            "Select the best agent to handle the user query:"
            f"\n\n{json.dumps(summaries, indent=2)}\n\n"
        )
        prompt_t = ChatPromptTemplate.from_messages(
            [
                SystemMessage(prompt),
                HumanMessage("{input}"),
            ]
        )
        self.chain = prompt_t | llm

    def invoke(self, query: str) -> Generator[ZeroShotRouterResponse, None, None]:
        response: ZeroShotRouterResponse = self.chain.invoke({"input": query})

        self._logger.debug(f"Reasoning: {response.reasoning}")
        self._logger.debug(f"Choice: {response.choice}")

        agent, model = self.agents.get(response.choice)

        if model is None:
            yield FastAPStep(
                name="ZeroShotRouter",
                content="Couldnt route to an expert agent.",
                is_last=True,
            )
            return

        temp_name = f"{agent['name']}-{uuid.uuid4()}"
        registry = ItemRegistry(name=temp_name)
        server = deploy(
            model,
            interpreter=self.interpreter,
            registry=registry,
            schema=agent,
            packages=agent["packages"],
            config=agent["config"],
        )

        yield from server.send_task(query)
