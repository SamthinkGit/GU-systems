import argparse
import logging

from langchain.globals import set_debug
from langsmith import traceable

import ecm.shared
from ecm.shared import get_logger
from ecm.tools.item_registry_v2 import ItemRegistry

COGNITION_LAYERS = ["fastreact", "xplore", "vfr", "darkvfr"]
EXECUTION_LAYERS = ["rosa", "pyxcel"]
DEFAULT_COGNITION = "DarkVFR"
DEFAULT_EXECUTOR = "pyxcel"


def main():

    argparser = argparse.ArgumentParser(description="Run the ECM project.")
    argparser.add_argument(
        "--verbose", action="store_true", help="Enable verbose outputs from the agents"
    )
    argparser.add_argument("--debug", action="store_true", help="Enable debug output")
    argparser.add_argument(
        "--langchain-debug",
        action="store_true",
        help="Enable LangChain Debug information",
    )
    argparser.add_argument(
        "--show-exelent",
        action="store_true",
        help="Shows Exelent file obtained from the Cognition Layer",
    )

    argparser.add_argument(
        "--host",
        action="store_true",
        help="[CAUTION] Executes the exelent generated code into the host machine. Please use in a safe environment.",
    )

    argparser.add_argument(
        "--agent",
        type=str,
        help="Select the Cognition Layer interface",
        choices=COGNITION_LAYERS,
        default=DEFAULT_COGNITION,
    )

    argparser.add_argument(
        "--executor",
        type=str,
        help="Select the Execution Layer interface",
        choices=EXECUTION_LAYERS,
        default=DEFAULT_EXECUTOR,
    )
    args = argparser.parse_args()

    # ----- Settling Layers ----
    match args.executor.lower():
        case "rosa":
            from execution_layer.rosa.interpreter.rosa_interpreter import (
                RosaInterpreter,
            )

            interpreter = RosaInterpreter()

        case "pyxcel":
            from execution_layer.pyxcel.interpreter.pyxcel_interpreter import (
                PyxcelInterpreter,
            )

            interpreter = PyxcelInterpreter()

        case _:
            return ValueError(
                "Execution Layer not valid, the unique supported values are: "
                + str(EXECUTION_LAYERS)
            )

    match args.agent.lower():
        case "darkvfr":
            from cognition_layer.agents.minimal_vfr.api.darkvfr_server import (
                get_fast_ap_server,
            )
            from cognition_layer.agents.minimal_vfr.supported_actions import (
                load_darkvfr_supported_actions,
            )

            load_darkvfr_supported_actions()
            server = get_fast_ap_server(interpreter=interpreter)

        case "vfr":
            from cognition_layer.agents.visual_fast_react.api.server import (
                get_fast_ap_server,
            )
            from cognition_layer.agents.visual_fast_react.supported_actions import (
                load_vfr_supported_actions,
            )

            load_vfr_supported_actions()
            server = get_fast_ap_server(interpreter=interpreter)

        case "fastreact":
            from cognition_layer.agents.fast_react.api.server import get_fast_ap_server
            from cognition_layer.agents.fast_react.supported_actions import (
                load_fastreact_supported_actions
            )

            load_fastreact_supported_actions()
            server = get_fast_ap_server(interpreter=interpreter)

        case "xplore":
            from cognition_layer.agents.xplore.api.server import get_fast_ap_server

            server = get_fast_ap_server(interpreter=interpreter)

        case _:
            return ValueError(
                "Cognition Layer not valid, the unique supported values are: "
                + str(COGNITION_LAYERS)
            )

    logger = get_logger("main")
    logger.debug(f"Running {args.agent} as Cognition Layer")
    logger.debug(f"Running {args.executor} as Execution Layer")

    if args.debug:
        ecm.shared.LOG_LEVEL = logging.DEBUG
    if args.langchain_debug:
        set_debug(True)
    if not args.host:
        ItemRegistry().invalidate(tools=False)

    ItemRegistry().load_all()
    logger.debug(f"Running {args.agent} as Cognition Layer")
    logger.debug("Initialization finished. Starting...")
    ItemRegistry().summary()

    @traceable
    def execute_user_query(query: str):
        for step in server.send_task(query):
            logger.debug(f"Step completed successfully:\n-> {step}")

    # -------- Running Tasks -------
    while True:
        query = input("Request a Task: ")
        execute_user_query(query)


if __name__ == "__main__":
    main()
