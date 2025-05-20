# flake8: noqa


def server_loader():
    from cognition_layer.experts.schema_listener.server import get_fast_ap_server

    return get_fast_ap_server


DEPLOY_MODEL = {
    "name": "SchemaListener",
    "alias": ["schema-listener"],
    "agent_description": "A router that can listen and return steps from all levels of the schema.",
    "response_description": "Returns all the steps generated inside the schema",
    "use_case": "-",
    "welcome_message": "I should not be used directly by an agent.",
    "type": "router",
    "packages": [],
    "server": server_loader,
}
