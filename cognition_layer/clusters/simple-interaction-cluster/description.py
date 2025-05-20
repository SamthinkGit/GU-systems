# flake8: noqa
from ecm.shared import get_root_path


def server_loader():
    from cognition_layer.deploy.loader import autodeploy_schema

    schema_path = (
        get_root_path()
        / "cognition_layer"
        / "clusters"
        / "simple-interaction-cluster"
        / "simple-interaction-cluster.json"
    )

    def server_loader(*args, **kwargs):
        interpreter = kwargs.get("interpreter")
        server = autodeploy_schema(id="", interpreter=interpreter, path=schema_path)
        return server

    return server_loader


DEPLOY_MODEL = {
    "name": "simple-interaction-cluster",
    "alias": ["simple-interaction-cluster"],
    "agent_description": "A cluster of agents that can perform clicks and keyboard actions on the current display.",
    "response_description": "Failure or success on clicking or typing the request",
    "use_case": "Use it to click or write on the current display.",
    "welcome_message": "Send me a context about where am I, a target (search bar, button, etc.) and if you want me to click it or write on it.",
    "type": "agent",
    "packages": [],
    "server": server_loader,
}
