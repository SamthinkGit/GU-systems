import asyncio
import logging

import ecm.shared
ecm.shared.LOG_LEVEL = logging.DEBUG

from ecm.core.main import main # noqa

# ------ ACTIONS (This actions will be used by the AI) ------
import action_space.keyboard.pynput # noqa
import action_space.window.focus # noqa

# -----------------------------------------------------------


if __name__ == "__main__":
    LOG_LEVEL = logging.DEBUG
    asyncio.run(
        main(
            cognition_layer="PLANEX",
            execution_layer="ROSA",
            verbose=True
        )
    )
