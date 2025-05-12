import argparse
import logging

from langchain.globals import set_debug
from langsmith import traceable

import ecm.shared
from cognition_layer.routing.descriptions import DEPLOY_MODELS
from ecm.shared import get_logger
from ecm.tools.item_registry_v2 import ItemRegistry

EXECUTION_LAYERS = ["rosa", "pyxcel"]
DEFAULT_COGNITION = "dark-vfr"
DEFAULT_EXECUTOR = "pyxcel"


def main():

    logger = get_logger("main")
    argparser = argparse.ArgumentParser(description="Run the ECM project.")
    argparser.add_argument(
        "--verbose", action="store_true", help="Enable verbose outputs from the agents"
    )
    argparser.add_argument(
        "--client",
        action="store_true",
        help="Set this machine as a ECM client (listener)",
    )
    argparser.add_argument(
        "--autodiscover",
        action="store_true",
        help="Discovers and saves a peer to connnect to the ECM",
    )
    argparser.add_argument(
        "--server",
        action="store_true",
        help="Set this machine as a ECM server (master)",
    )
    argparser.add_argument(
        "--localhost",
        action="store_true",
        help="Set this machine as a ECM server (master) with localhost connection detection",
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
    # ----- Autodiscovering Peer ----
    if args.autodiscover:
        from ecm_communications.bootstraps.autodiscover import autodiscover

        logger.info("Waiting for peer to connect...")
        autodiscover(allow_localhost=args.localhost)

    # ----- Discovering deploy model (agent) ----
    deploy_model = None
    for model in DEPLOY_MODELS:
        if args.agent.lower() in model["alias"]:
            deploy_model = model
            break

    if deploy_model is None:
        raise ValueError("Invalid Agent layer passed as argument")

    # ----- Loading Action Space ----
    actions_loader = deploy_model["supported_actions"]
    server_loader = deploy_model["server"]()
    actions_loader()

    # ----- Listening (Only for clients) ----
    if args.client:
        from ecm_communications.tools.listener import Listener

        registry = ItemRegistry()
        registry.load_all()
        registry.summary()

        listener = Listener()
        listener.listen()
        exit()

    # ----- Settling Execution Layer ----
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

    server = server_loader(interpreter)

    logger.debug(f"Running {args.agent} as Cognition Layer")
    logger.debug(f"Running {args.executor} as Execution Layer")

    if args.debug:
        ecm.shared.LOG_LEVEL = logging.DEBUG
    if args.langchain_debug:
        set_debug(True)
    if not args.host:
        ItemRegistry().invalidate(tools=False)

    if args.server:
        from ecm_communications.tools.server import enable_server_mode

        enable_server_mode()

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
