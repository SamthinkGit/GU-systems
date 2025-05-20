from operator import attrgetter

from cognition_layer.api import FastAgentProtocol
from cognition_layer.experts.schema_listener.agent import SchemaListener


def get_fast_ap_server(schema_name: str, **kwargs) -> FastAgentProtocol:

    listener = SchemaListener(schema_name=schema_name)
    return FastAgentProtocol(
        name="SchemaListener",
        iterator=lambda input: listener.invoke(input),
        step_name_getter=attrgetter("name"),
        content_getter=attrgetter("content"),
        is_last_getter=attrgetter("is_last"),
    )
