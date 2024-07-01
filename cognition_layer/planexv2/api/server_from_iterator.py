"""
PlanexV2 Server Module
==============================

This module provides the server setup for the PlanexV2 agent, integrating it into a
server framework that allows it to process input queries iteratively. It uses the
ServerFromIterator utility to create a server that manages the interactions and data
flow for each step of the PlanexV2 agent's processing workflow, facilitating the
execution and response generation for incoming requests.

Note: The deployment uses AgentProtocol as framework.
"""
from operator import attrgetter

from cognition_layer.api import ServerAPI
from cognition_layer.planexv2.agents.planexv2 import PlanexV2
from cognition_layer.tools.server_template import ServerFromIterator


def get_server(verbose: bool = False) -> ServerAPI:
    """
    Initializes and returns a ServerAPI instance configured to use the PlanexV2
    agent for processing input queries. This server setup allows for the iterative
    handling of queries, utilizing the PlanexV2 agent's iterative process.

    :param verbose: Flag to enable verbose logging of the server's processing steps.
    :return: An instance of ServerAPI configured with the PlanexV2 agent.

    Note: Remember to start the server when using it:
    ```python
        server = get_server()
        server.start()
    ```
    """
    agent = PlanexV2()
    return ServerFromIterator(
        name="PlanexV2",
        iterator=lambda input: agent.iter(input, verbose=verbose),
        step_name_getter=attrgetter("next_agent"),
        content_getter=attrgetter("content"),
        is_last_getter=attrgetter("is_last"),
    )
