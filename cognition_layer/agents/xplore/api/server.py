import random

from cognition_layer.api import FastAgentProtocol
from cognition_layer.xplore.agents.xplore import Xplore
from ecm.mediator.Interpreter import Interpreter


def get_fast_ap_server(interpreter: Interpreter, **kwargs) -> FastAgentProtocol:

    xplore = Xplore(interpreter=interpreter)
    config = {"configurable": {"thread_id": str(random.randint(5000, 15000))}}
    iterator = lambda input: xplore.graph.stream(  # noqa
        {"query": input}, config, stream_mode="values"
    )

    return FastAgentProtocol(
        name="FastReact Server",
        iterator=iterator,
        step_name_getter=lambda event: event.get("current_node"),
        content_getter=lambda event: dict(event),
        is_last_getter=lambda event: False,
    )
