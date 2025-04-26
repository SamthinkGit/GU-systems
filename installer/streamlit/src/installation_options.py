from config import COGNITION_LAYER_OPTIONS
from config import EXECUTION_LAYER_OPTIONS
from config import LATEST_COGNITION_LAYER
from config import LATEST_EXECUTION_LAYER


def get_execution_layer_options():
    """
    Get the execution layer options for the installation.
    """

    return [
        (
            f"{value} (Latest)"
            if value.lower() == LATEST_EXECUTION_LAYER.lower()
            else value
        )
        for value in EXECUTION_LAYER_OPTIONS
    ]


def get_cognition_layer_options():
    """
    Get the cognition layer options for the installation.
    """

    return [
        (
            f"{value} (Latest)"
            if value.lower() == LATEST_COGNITION_LAYER.lower()
            else value
        )
        for value in COGNITION_LAYER_OPTIONS
    ]
