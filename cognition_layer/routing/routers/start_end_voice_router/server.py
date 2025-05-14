from operator import attrgetter

from cognition_layer.api import FastAgentProtocol
from cognition_layer.deploy.types import DeploySchema
from cognition_layer.routing.routers.start_end_voice_router.router import (
    StartEndVoiceRouter,
)
from ecm.mediator.Interpreter import Interpreter


def get_fast_ap_server(
    interpreter: Interpreter, schema: DeploySchema, model: str, **kwargs
) -> FastAgentProtocol:

    router = StartEndVoiceRouter(
        schema=schema,
        interpreter=interpreter,
        model=model,
        **kwargs,
    )
    return FastAgentProtocol(
        name="StartEndVoiceRouter Server",
        iterator=lambda input: router.invoke(input),
        step_name_getter=attrgetter("name"),
        content_getter=attrgetter("content"),
        is_last_getter=attrgetter("is_last"),
    )
