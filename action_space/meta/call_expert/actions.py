from functools import cache

from langsmith import traceable

from cognition_layer.deploy.loader import deploy
from cognition_layer.deploy.loader import get_deploy_model
from cognition_layer.deploy.types import DeployModel
from cognition_layer.deploy.types import DeploySchema
from cognition_layer.protocols.fast_ap import FastAgentProtocol
from ecm.tools.item_registry_v2 import ItemRegistry
from ecm.tools.item_registry_v2 import Storage

PKG_NAME = "expert-calling"
STORAGE_KEY = "EXPERT_CALLING"


@ItemRegistry.register(type="action", package=PKG_NAME, labels=["enforce-host"])
def call_expert(name: str, message: str) -> None:
    """
    Calls an expert with the given name and message.
    Args:
        name (str): The name of the expert to call.
        message (str): The message to send to the expert.

    Note:
        - The first message should be "<start>" to initiate the conversation.
        - The expert's response will be returned as a string.
    """
    return _call_expert(name, message)


def _call_expert(name: str, message: str) -> None:
    server, model = load_model(name)
    if message == "<start>":
        if "welcome_message" in model:
            return model["welcome_message"]
        return "This expert does not have a welcome message. Please send the first message."

    @traceable(run_type="chain", name=server.name)
    def execute_user_query(query: str):
        results = []
        for step in server.send_task(query):
            results.append(f"{step.name}: {step.content}")
        return results

    return "\n".join(execute_user_query(message))


@cache
def load_model(name: str) -> tuple[FastAgentProtocol, DeployModel]:
    storage = Storage(STORAGE_KEY)
    experts: list[DeploySchema] = storage.get("experts", [])
    interpreter = storage.get("interpreter", None)

    if not experts:
        raise ValueError("No experts available.")
    target_schema = None
    for schema in experts:
        if schema["name"] == name:
            target_schema = schema

    if target_schema is None:
        raise ValueError(
            f"Expert {name} not found in Storage({STORAGE_KEY})['experts']."
        )
    model = get_deploy_model(target_schema["model"])
    server = deploy(
        model,
        interpreter,
        schema=target_schema,
        config=schema["config"],
    )
    return server, model
