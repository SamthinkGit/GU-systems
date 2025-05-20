from operator import attrgetter

from cognition_layer.api import FastAgentProtocol
from cognition_layer.experts.cluster_listener.agent import ClusterListener


def get_fast_ap_server(schema_name: str, **kwargs) -> FastAgentProtocol:

    listener = ClusterListener(schema_name=schema_name)
    return FastAgentProtocol(
        name="ClusterListener",
        iterator=lambda input: listener.invoke(input),
        step_name_getter=attrgetter("name"),
        content_getter=attrgetter("content"),
        is_last_getter=attrgetter("is_last"),
    )
