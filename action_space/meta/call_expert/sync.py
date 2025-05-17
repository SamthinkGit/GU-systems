from cognition_layer.deploy.types import DeploySchema
from ecm.tools.registry import Storage


def sync_experts(schema: DeploySchema) -> None:
    storage = Storage("EXPERT_CALLING")
    storage["experts"] = schema["workers"]
