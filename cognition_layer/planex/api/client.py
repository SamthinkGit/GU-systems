import agent_protocol_client

from cognition_layer.planex.constants import API_ADDRESS
from cognition_layer.planex.constants import API_PORT
from cognition_layer.templates import ClientAPI


class PlanexClient(ClientAPI):

    def __init__(self) -> None:
        configuration = agent_protocol_client.Configuration(
            host=f"http://{API_ADDRESS}:{API_PORT}"
        )
        self._api_client = agent_protocol_client.ApiClient(configuration)
        self.api_instance = agent_protocol_client.AgentApi(self._api_client)

    async def __aenter__(self):
        return self.api_instance

    async def __aexit__(self, exc_type, exc_value, traceback):
        await self.api_client.close()
