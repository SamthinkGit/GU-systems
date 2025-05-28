import uuid

from cognition_layer.deploy.loader import deploy
from cognition_layer.deploy.loader import get_deploy_model
from cognition_layer.deploy.types import DeploySchema
from cognition_layer.protocols.fast_ap import FastAgentProtocol
from ecm.mediator.Interpreter import Interpreter
from ecm.tools.item_registry_v2 import ItemRegistry


class SimpleRouter:

    def __init__(self, schema: DeploySchema, *args, **kwargs):
        self.agents = {
            agent["name"]: (agent, get_deploy_model(agent["model"]))
            for agent in schema["workers"]
        }

    def summary_dict(self) -> list[dict]:
        return [
            {
                "name": name,
                "description": model["agent_description"],
                "use_case": model["use_case"],
            }
            for name, (_, model) in self.agents.items()
        ]

    def server(
        self, target: str, interpreter: Interpreter, **kwargs
    ) -> FastAgentProtocol:

        agent_model = self.agents.get(target)

        if agent_model is None:
            raise ValueError(f"Agent {target} not found in the schema.")

        agent, model = agent_model

        if model is None:
            return None

        temp_name = f"{agent['name']}-{uuid.uuid4()}"
        registry = ItemRegistry(name=temp_name)
        server = deploy(
            model,
            interpreter=interpreter,
            registry=registry,
            schema=agent,
            packages=agent["packages"],
            config=agent["config"],
            **kwargs,
        )
        return server
