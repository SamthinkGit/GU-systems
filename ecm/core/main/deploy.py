import argparse

from langsmith import traceable

from cognition_layer.deploy.loader import autodeploy_schema
from cognition_layer.protocols.broadcaster import deploy_backend
from cognition_layer.protocols.broadcaster import exit_fast_ap
from cognition_layer.protocols.broadcaster import terminate_backend
from cognition_layer.protocols.fast_ap import config_fast_ap
from ecm.shared import get_logger
from ecm.tools.item_registry_v2 import ItemRegistry
from ecm.tools.item_registry_v2 import Storage
from ecm.tools.prettify import pretty_print_schema
from execution_layer.pyxcel.interpreter.pyxcel_interpreter import PyxcelInterpreter

LATEST = "hybrid1"


def main():

    logger = get_logger("main")
    argparser = argparse.ArgumentParser(description="Run the ECM project.")

    argparser.add_argument(
        "schema",
        help="The name of the schema to parse (found at /cognition_layer/routing/schemas)."
        "\nUse 'latest' to use the latest schema.",
    )

    argparser.add_argument(
        "--invalidate",
        action="store_true",
        help="Invalidate the item registry to only print instead of make actions (useful for testing).",
    )

    argparser.add_argument(
        "--api",
        action="store_true",
        help="Publish all the steps to the FastAP API.",
    )
    argparser.add_argument(
        "--backend",
        action="store_true",
        help="Run the backend server for FastAP.",
    )
    argparser.add_argument(
        "--prompt",
        type=str,
        help="Sends this prompt to the schema.",
    )
    argparser.add_argument(
        "--absolute",
        action="store_true",
        help=(
            "The path to the schema file will be considered absolute."
            " Use this to deploy custom schemas or clusters."
        ),
    )

    args = argparser.parse_args()

    if args.schema == "latest":
        args.schema = LATEST

    if args.api:
        config_fast_ap(enable_api=True)

    if args.backend:
        deploy_backend()

    if args.absolute:
        pretty_print_schema(path=args.schema)
    else:
        pretty_print_schema(id=args.schema)

    # ----- Loading Interpreter ----
    interpreter = PyxcelInterpreter()
    logger.debug("Pyxcel Interpreter loaded successfully.")

    # ----- Deploying Schema ----
    if not args.absolute:
        server = autodeploy_schema(
            args.schema,
            interpreter=interpreter,
        )
    else:
        server = autodeploy_schema(
            id="",
            interpreter=interpreter,
            path=args.schema,
        )

    logger.debug(f"Schema `{args.schema}` deployed successfully.")

    # ----- Invalidating (if needed) ----
    if args.invalidate:
        ItemRegistry().invalidate(tools=False)

    # -------- Running Tasks -------
    @traceable(run_type="chain", name=server.name)
    def execute_user_query(query: str):
        for step in server.send_task(query):
            logger.debug(f"Step completed successfully:\n-> {step}")

    try:
        if args.prompt:
            execute_user_query(args.prompt)
        else:
            while True:
                query = input("Request a Task: ")
                execute_user_query(query)
    finally:
        if args.backend:
            terminate_backend()

        if args.api:
            exit_fast_ap(port=Storage("FAST_AP_CONFIG")["port"])


if __name__ == "__main__":
    main()
