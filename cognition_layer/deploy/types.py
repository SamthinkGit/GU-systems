from typing import Any
from typing import Callable
from typing import Literal
from typing import TypedDict


class DeployModel(TypedDict):
    """
    A dictionary representing the requirements, configuration and
    server loader function for a deploy model (agent).

    This is implemented by the developer adding all the information
    about the behavior of the agent.
    """

    name: str
    alias: list[str]
    agent_description: str
    response_description: str
    welcome_message: str
    use_case: str
    type: str
    packages: list[str]
    server: Callable


class DeploySchema(TypedDict):
    """
    A dictionary representing a the node in the deployment network.
    It could be an agent, router, or any other component that can be deployed
    in the network.
    It contains the information about its position in the network and the
    configuration of the agent or subagents (if any).

    This is configured automatically by the Schema Deployer.
    """

    name: str
    type: Literal["agent", "router"]
    model: str
    config: dict[str, Any]
    packages: list[str]
    workers: list["DeploySchema"]
