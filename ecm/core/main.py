import argparse
import asyncio
import atexit
import multiprocessing
import time

import agent_protocol_client
import aiohttp.client_exceptions
import requests

import ecm.exelent.parser as parser
from cognition_layer.api import CognitionMediator
from cognition_layer.api import ServerAPI
from cognition_layer.constants import API_ADDRESS
from cognition_layer.constants import API_PORT
from cognition_layer.planex.utils.format import extract_python_code
from ecm.mediator.Interpreter import Interpreter
from ecm.shared import get_logger

# TOOLS


async def main(
    cognition_layer: str = "PLANEX",
    execution_layer: str = "ROSA",
    verbose: bool = False,
):

    # ---- Initializing ----
    host = f"http://{API_ADDRESS}:{API_PORT}"
    configuration = agent_protocol_client.Configuration(host=host)
    logger = get_logger("main")

    # ---- Settling Layers ----
    logger.debug(f"Running {cognition_layer} as cognition_layer")
    match cognition_layer:
        case "PLANEX":

            from cognition_layer.planex.api.server import PlanexServer

            server = PlanexServer(verbose=verbose)

        case _:
            return ValueError(
                "Cognition Layer not valid, the unique supported values are: "
                + str(
                    [
                        "PLANEX",
                    ]
                )
            )

    logger.debug(f"Running {execution_layer} as execution_layer")
    match execution_layer:
        case "ROSA":
            from ecm.mediator.rosa_interpreter import RosaInterpreter

            interpreter = RosaInterpreter()

        case _:
            return ValueError(
                "Execution Layer not valid, the unique supported values are: "
                + str(
                    [
                        "ROSA",
                    ]
                )
            )

    server: ServerAPI
    interpreter: Interpreter

    # ---- Deploying AgentProtocol ----
    server_process = multiprocessing.Process(target=server.start)
    server_process.start()
    atexit.register(lambda p: p.join(), server_process)

    info_shown = False
    status_code = 404

    while status_code != 200:

        try:
            status_code = requests.get(host + "/ap/v1/agent/tasks").status_code
        except Exception:
            pass

        if not info_shown:
            logger.info("Waiting for API server to be alive...")
            info_shown = True
        time.sleep(0.5)

    # ---- Running ECM ----
    async with agent_protocol_client.ApiClient(configuration) as api_client:

        mediator = CognitionMediator(api_client)

        while True:

            # ------------ COGNITION LAYER -------------------
            try:
                task = await mediator.get_task(input=input("Request a Task: "))
                result = await mediator.run_task(task)
            except aiohttp.client_exceptions.ClientOSError:
                logger.error("Conection lost, retrying...")
                mediator = CognitionMediator(api_client)
                continue

            # ------------ PARSING EXELENT -------------------
            if result.startswith("```python"):
                result = extract_python_code(result)

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

    import tests.mocks.agent_actions  # noqa

    argparser = argparse.ArgumentParser(
        description="Run Planex Server with optional verbosity."
    )
    argparser.add_argument(
        "--verbose", action="store_true", help="Enable verbose output"
    )
    args = argparser.parse_args()

    asyncio.run(
        main(execution_layer="ROSA", cognition_layer="PLANEX", verbose=args.verbose)
    )
