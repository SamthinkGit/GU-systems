from cognition_layer.deploy.types import DeploySchema
from ecm.mediator.Interpreter import Interpreter
from ecm.tools.registry import Storage


def sync_experts(schema: DeploySchema, interpreter: Interpreter) -> None:
    storage = Storage("EXPERT_CALLING")
    storage["experts"] = schema["workers"]
    storage["interpreter"] = interpreter
