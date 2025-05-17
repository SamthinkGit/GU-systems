from langsmith import traceable

from cognition_layer.deploy.loader import deploy
from cognition_layer.deploy.loader import get_deploy_model
from ecm.tools.registry import ItemRegistry
from execution_layer.pyxcel.interpreter.pyxcel_interpreter import PyxcelInterpreter


def call_action(name: str, package: str, *args, **kwargs):
    registry = ItemRegistry()
    registry.autoload(package)
    action = registry.get(name, type="action")
    if action is None:
        raise ValueError(f"Action {name} not found in package {package}")
    return action.content(*args, **kwargs)


@traceable
def call_expert(
    identifier: str, query: str, config: dict = None, packages: list[str] = None
) -> str:
    if config is None:
        config = {}
    if packages is None:
        packages = ["default"]
    model = get_deploy_model(identifier)
    interpreter = PyxcelInterpreter()
    server = deploy(
        model,
        interpreter=interpreter,
        packages=packages,
        config=config,
        registry=ItemRegistry(identifier),
    )
    for step in server.send_task(query):
        print(f"Step:\n-> {step}")
