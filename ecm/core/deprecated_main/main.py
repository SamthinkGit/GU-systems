"""
Main ECM Runner
==============================

This script sets up and runs the ECM, integrating the execution layer and
cognition layer to handle tasks using the Agent Protocol. It initializes the necessary components,
including the server, interpreter, and agent protocol client, and manages the workflow for
processing tasks and executing plans. The script supports different cognition layers and provides
options for verbosity, debugging, and displaying intermediate results.
"""
import argparse
import asyncio
import atexit
import ecm.exelent.parser as parser
import ecm.shared
import logging
import multiprocessing
import time
from cognition_layer.api import CognitionMediator
from cognition_layer.api import ServerAPI
from cognition_layer.constants import API_ADDRESS
from cognition_layer.constants import API_PORT
from cognition_layer.planex.utils.format import extract_python_code
from ecm.mediator.Interpreter import Interpreter
from ecm.shared import get_logger

import agent_protocol_client
import aiohttp.client_exceptions
import requests
from langchain.globals import set_debug

# TOOLS

DEBUG_SHOW_EXELENT = False


def clean(interpreter: Interpreter):
    interpreter.kill()


async def main(
    cognition_layer: str = "FastReact",
    execution_layer: str = "Pyxcel",
    verbose: bool = False,
):
    # ---- Initializing ----
    host = f"http://{API_ADDRESS}:{API_PORT}"
    configuration = agent_protocol_client.Configuration(host=host)
    logger = get_logger("main")
    execution_layer_managed_by_server = False

    # ---- Settling Layers ----
    logger.debug(f"Running {execution_layer} as execution_layer")
    match execution_layer:
        case "ROSA":
            from execution_layer.rosa.interpreter.rosa_interpreter import (
                RosaInterpreter,
            )

            interpreter_class = RosaInterpreter

        case "Pyxcel":
            from execution_layer.pyxcel.interpreter.pyxcel_interpreter import (
                PyxcelInterpreter,
            )

            interpreter_class = PyxcelInterpreter
        case _:
            return ValueError(
                "Execution Layer not valid, the unique supported values are: "
                + str(
                    [
                        "ROSA",
                    ]
                )
            )

    logger.debug(f"Running {cognition_layer} as cognition_layer")
    match cognition_layer.lower():
        case "planex":

            from cognition_layer.planex.api.server import PlanexServer

            server = PlanexServer(verbose=verbose)
            interpreter = interpreter_class()

        case "planexv2":
            from cognition_layer.planexv2.api.server_from_iterator import get_server

            server = get_server(verbose=verbose)
            interpreter = interpreter_class()

        case "replan":
            from cognition_layer.RePlan.api.server import get_server

            server = get_server(interpreter_class=interpreter_class, verbose=verbose)
            execution_layer_managed_by_server = True
            interpreter = None

        case "fastreact":
            from cognition_layer.fast_react.api.server import get_server

            server = get_server(interpreter_class=interpreter_class)
            execution_layer_managed_by_server = True
            interpreter = None

        case _:
            return ValueError(
                "Cognition Layer not valid, the unique supported values are: "
                + str(
                    [
                        "PLANEX",
                        "PlanexV2",
                        "RePlan",
                        "FastReact",
                    ]
                )
            )

    server: ServerAPI
    interpreter: Interpreter | None

    # ---- Deploying AgentProtocol ----
    server_process = multiprocessing.Process(target=server.start)
    server_process.start()

    if interpreter:
        atexit.register(clean, interpreter)

    atexit.register(lambda p: p.join(), server_process)

    info_shown = False
    status_code = 404

    while status_code != 200:

        try:
            status_code = requests.get(host + "/ap/v1/agent/tasks").status_code
            print(status_code)
        except Exception:
            logger.debug("API not available yet:", exc_info=True)

        if not info_shown:
            logger.info("Waiting for API server to be alive...")
            info_shown = True
        time.sleep(0.5)

    # ---- Running ECM ----
    async with agent_protocol_client.ApiClient(configuration) as api_client:

        mediator = CognitionMediator(api_client)
        previous_query = None

        while True:

            query = previous_query if previous_query else input("Request a Task: ")
            previous_query = None

            # ------------ COGNITION LAYER -------------------
            try:
                task = await mediator.get_task(input=query)
                result = await mediator.run_task(task)

            except aiohttp.client_exceptions.ClientOSError:
                logger.error("Conection lost, retrying...")
                mediator = CognitionMediator(api_client)
                previous_query = query
                continue
            except Exception:
                logger.error(
                    "Exception ocurred during the call of the cognition layer",
                    exc_info=True,
                )
                break

            if execution_layer_managed_by_server:
                continue

            if not isinstance(result, str):
                logger.error(f"Cognition Layer returned invalid task: {result}. Exit")
                break

            # ------------ PARSING EXELENT -------------------
            if result.startswith("```python"):
                result = extract_python_code(result)

            if DEBUG_SHOW_EXELENT:
                logger.info(result)

            try:
                plan = parser.parse(target_str=result)
            except Exception:
                logger.error("Invalid plan generated.", exc_info=True)
                continue

            # ------------ EXECUTION LAYER -------------------
            try:
                logger.debug("Generated Packages:")
                for pkg in interpreter._generate_packages_from_parsed_task(plan):
                    logger.debug(pkg.to_json())

                logger.debug("Running Packages...")
                interpreter.run(plan, callback="silent")

            except NameError:
                logger.error(
                    "Cognition Layer generated an undefined function.", exc_info=True
                )

            except Exception:
                logger.error("Invalid plan generated.", exc_info=True)


if __name__ == "__main__":

    # ---------- ACTION SPACE ---------
    import action_space.keyboard.actions  # noqa
    import action_space.window.focus  # noqa

    # ----------------------------------

    from ecm.tools.registry import ItemRegistry

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
        "--agent", type=str, help="Select the Agent used for the cognition layer"
    )

    args = argparser.parse_args()

    if args.debug:
        ecm.shared.LOG_LEVEL = logging.DEBUG
    if args.langchain_debug:
        set_debug(True)
    if args.show_exelent:
        DEBUG_SHOW_EXELENT = True
    if not args.host:
        ItemRegistry().invalidate()

    cognition_layer = args.agent if args.agent else "FastReact"

    asyncio.run(
        main(
            execution_layer="Pyxcel",
            cognition_layer=cognition_layer,
            verbose=args.verbose,
        )
    )
