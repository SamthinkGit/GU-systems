from operator import attrgetter

from cognition_layer.api import FastAgentProtocol
from cognition_layer.deploy.types import DeploySchema
from cognition_layer.routing.routers.zero_shot_router.router import ZeroShotRouter
from ecm.mediator.Interpreter import Interpreter


def get_fast_ap_server(
    interpreter: Interpreter, schema: DeploySchema, **kwargs
) -> FastAgentProtocol:

    router = ZeroShotRouter(
        schema=schema,
        interpreter=interpreter,
    )
    return FastAgentProtocol(
        name="ZeroShotRouter Server",
        iterator=lambda input: router.invoke(input),
        step_name_getter=attrgetter("name"),
        content_getter=attrgetter("content"),
        is_last_getter=attrgetter("is_last"),
    )
