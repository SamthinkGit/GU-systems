# flake8: noqa


def server_loader():
    from cognition_layer.experts.element_finder.server import (
        get_fast_ap_server,
    )

    return get_fast_ap_server


DEPLOY_MODEL = {
    "name": "ElementFinder",
    "alias": ["element-finder"],
    "agent_description": "A vision agent that can find the coordinates of elements in images/screenshots or the current display.",
    "response_description": "Coordinates of the requested element",
    "use_case": "Use it to find the coordinates of an element in the screen (buttons, images, clickables, texts, etc.)",
    "welcome_message": "Send me the element you want to find, a purpouse for it (is a button?, is an icon?) and contextual information you consider relevant.",
    "type": "agent",
    "packages": [
        "moondream_element_finder",
        "expert-calling",
        "simple-read-ocr",
        "screenshot",
    ],
    "server": server_loader,
}
