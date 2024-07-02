"""
Main ECM Runner
==============================

This script sets up and runs the ECM, integrating the execution layer and
cognition layer to handle tasks using the Agent Protocol. It initializes the necessary components,
including the server, interpreter, and agent protocol client, and manages the workflow for
processing tasks and executing plans.

[WARNING] This module will run the action in the hosts computer, take any preventions you
consider necessary before executing this module
"""
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

    # You can change here the cognition / execution layer
    # TODO: Copy main.py args
    asyncio.run(
        main(
            cognition_layer="PLANEX",
            execution_layer="ROSA",
            verbose=True
        )
    )
