import argparse
import ecm.shared
import logging
from ecm.shared import get_logger
from ecm.tools.item_registry_v2 import ItemRegistry

from langchain.globals import set_debug

COGNITION_LAYERS = ["fastreact", "xplore"]
EXECUTION_LAYERS = ["rosa", "pyxcel"]
DEFAULT_COGNITION = "fastreact"
DEFAULT_EXECUTOR = "pyxcel"


def main(args):

    logger = get_logger("main")

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

    logger.debug(f"Running {args.executor} as Execution Layer")
    match args.agent.lower():
        case "fastreact":
            from cognition_layer.fast_react.api.server import get_fast_ap_server

            server = get_fast_ap_server(interpreter=interpreter)
        case "xplore":
            from cognition_layer.xplore.api.server import get_fast_ap_server

            server = get_fast_ap_server(interpreter=interpreter)

        case _:
            return ValueError(
                "Cognition Layer not valid, the unique supported values are: "
                + str(COGNITION_LAYERS)
            )

    logger.debug(f"Running {args.agent} as Cognition Layer")
    logger.debug("Initialization finished. Starting...")
    ItemRegistry().summary()

    # -------- Running Tasks -------
    while True:
        query = input("Request a Task: ")
        for step in server.send_task(query):
            logger.debug(f"Step completed successfully:\n-> {step}")


if __name__ == "__main__":
    # ---------- ACTION SPACE ---------
    import action_space.keyboard.actions  # noqa
    import action_space.experimental.screenshot.actions  # noqa

    # ----------------------------------
    ItemRegistry().load_all()

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

    if args.debug:
        ecm.shared.LOG_LEVEL = logging.DEBUG
    if args.langchain_debug:
        set_debug(True)
    if not args.host:
        ItemRegistry().invalidate(tools=False)

    main(args)
