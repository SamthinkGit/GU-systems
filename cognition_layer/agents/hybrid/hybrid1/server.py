from operator import attrgetter

from cognition_layer.agents.hybrid.hybrid1.agent import Hybrid1
from cognition_layer.api import FastAgentProtocol
from ecm.mediator.Interpreter import Interpreter
from ecm.tools.registry import ItemRegistry


def get_fast_ap_server(
    interpreter: Interpreter,
    registry: ItemRegistry = ItemRegistry(),
    memory_capacity: int = 10,
    **kwargs
) -> FastAgentProtocol:

    schema = kwargs.get("schema")
    if schema is None:
        raise ValueError(
            "Schema is required for the Hybrid1 agent " "(Required for expert-calling)"
        )
    hybrid = Hybrid1(
        interpreter=interpreter,
        registry=registry,
        memory_capacity=memory_capacity,
        schema=schema,
    )
    return FastAgentProtocol(
        name="Hybrid1 Server",
        iterator=lambda input: hybrid.complete_task(input),
        step_name_getter=attrgetter("name"),
        content_getter=attrgetter("content"),
        is_last_getter=attrgetter("is_last"),
    )
