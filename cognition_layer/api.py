# flake8: noqa
from cognition_layer.protocols.fast_ap import FastAgentProtocol
from cognition_layer.protocols.fast_ap import FastAPStep

ENABLE_BACKWARD_COMPATIBILITY = False
if ENABLE_BACKWARD_COMPATIBILITY:
    from cognition_layer.protocols.old_ap_api import CognitionMediator
    from cognition_layer.protocols.old_ap_api import ServerAPI
else:

    class ServerAPI:
        def __init__(self, *args, **kwargs):
            raise NotImplementedError(
                "ServerAPI is not implemented in the new API. Use FastAgentProtocol instead."
            )

    def CognitionMediator(*args, **kwargs):
        raise NotImplementedError(
            "CognitionMediator is not implemented in the new API. Use FastAgentProtocol instead."
        )
