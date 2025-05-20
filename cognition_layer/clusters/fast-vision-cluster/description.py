# flake8: noqa
from ecm.shared import get_root_path


def server_loader():
    from cognition_layer.deploy.loader import autodeploy_schema

    schema_path = (
        get_root_path()
        / "cognition_layer"
        / "clusters"
        / "fast-vision-cluster"
        / "fast-vision-cluster.json"
    )

    def server_loader(*args, **kwargs):
        interpreter = kwargs.get("interpreter")
        server = autodeploy_schema(id="", interpreter=interpreter, path=schema_path)
        return server

    return server_loader


DEPLOY_MODEL = {
    "name": "fast-vision-cluster",
    "alias": ["fast-vision-cluster"],
    "agent_description": "A cluster of agents that can process the current display and retrieve information or give responses to queries.",
    "response_description": "Response about the screen",
    "use_case": "Use it to give responses to the user based on the display/screenshot or to response about graphical queries/questions.",
    "welcome_message": "Send me a question about the current screen or image and I will do my best to assist you.",
    "type": "agent",
    "packages": ["screenshot"],
    "server": server_loader,
}
