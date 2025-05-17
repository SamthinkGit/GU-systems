import importlib
import json
import uuid
from functools import cache
from pathlib import Path
from typing import Optional

from cognition_layer.deploy.types import DeployModel
from cognition_layer.deploy.types import DeploySchema
from cognition_layer.protocols.fast_ap import FastAgentProtocol
from ecm.mediator.Interpreter import Interpreter
from ecm.shared import get_root_path
from ecm.tools.registry import ItemRegistry

DESCRIPTION_FILE_NAME = "description.py"


@cache
def discover_deploy_models(
    root_path: Path = get_root_path() / "cognition_layer",
) -> list[DeployModel]:
    agents = []

    for description_file in root_path.rglob(DESCRIPTION_FILE_NAME):

        module_parts = (
            description_file.with_suffix("").relative_to(get_root_path()).parts
        )
        module_path = ".".join(module_parts)

        module = importlib.import_module(module_path)
        deploy_model = getattr(module, "DEPLOY_MODEL", None)
        agents.append(deploy_model)

    return agents


@cache
def get_deploy_model(identifier: str) -> DeployModel:
    """
    Dynamically loads a deploy model (agent) given its name or alias.
    - identifier: can be the name (e.g., 'fastreact') or an alias (e.g., 'fr').
    raises ValueError if the identifier is not found.
    """
    models = discover_deploy_models()
    target_model = None
    for model in models:
        if (
            identifier.lower() in model["alias"]
            or identifier.lower() == model["name"].lower()
        ):
            target_model = model

    if target_model is None:
        raise ValueError(f"Invalid Agent identifier: {identifier}.")
    return target_model


def deploy(
    model: DeployModel,
    interpreter: Interpreter,
    registry: Optional[ItemRegistry] = None,
    packages: Optional[list[str]] = None,
    schema: Optional[DeploySchema] = None,
    config: Optional[dict] = None,
) -> FastAgentProtocol:
    """
    Dynamically loads a deploy model (agent) given its name or alias.
    - model: the deploy model (agent) to be loaded.
    - interpreter: the Interpreter instance to be used by the agent.
    - registry: the ItemRegistry instance to be used by the agent.
    - packages: a list of package names to be autoloaded. If None, the default packages
      for the agent will be used.
    - schema: the schema to be used by the agent. If None, the default schema for the
      agent will be used.
    Returns the server instance of the agent.
    Example:
        >>> from cognition_layer.deploy.loader import deploy
        >>> model = get_deploy_model("darkvfr")
        >>> server = deploy(model, interpreter)
        >>> for step in server("Open spotify"):
        >>>     print(step)
    """
    if registry is None:
        registry_name = f"{model['name']}-{uuid.uuid4()}"
        registry = ItemRegistry(registry_name)

    if packages is None:
        packages = []

    if schema is not None:
        packages += schema["packages"]

    if "default" in packages:
        packages += model["packages"]
        packages.remove("default")

    for pkg in packages:
        registry.autoload(pkg)

    for pkg in packages:
        ItemRegistry().autoload(pkg)  # Execution layer can uses this

    if model["type"] == "router" and schema is None:
        raise ValueError(
            "Schema is required for router models. Please provide a schema."
        )

    if config is None:
        config = {}

    server_loader = model["server"]
    server_getter = server_loader()
    server: FastAgentProtocol = server_getter(
        interpreter=interpreter, registry=registry, schema=schema, **config
    )
    return server


def autodeploy_schema(id: str, interpreter: Interpreter) -> FastAgentProtocol:
    """
    Deploys a schema given its id.
    - id: the id of the schema to be deployed.
    - interpreter: the Interpreter instance to be used by the agent.
    Returns the server instance of the agent.
    Example:
        >>> from cognition_layer.deploy.loader import autodeploy_schema
        >>> server = autodeploy_schema("full_feedback_routing", interpreter)
        >>> for step in server("Open spotify"):
        >>>     print(step)
    """
    path = get_root_path() / "cognition_layer" / "routing" / "schemas" / f"{id}.json"

    schema = json.loads(path.read_text())
    model = get_deploy_model(schema["model"])
    server = deploy(model, interpreter, schema=schema, config=schema["config"])
    return server
