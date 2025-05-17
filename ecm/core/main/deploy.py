import argparse

from langsmith import traceable

from cognition_layer.deploy.loader import autodeploy_schema
from ecm.shared import get_logger
from ecm.tools.item_registry_v2 import ItemRegistry
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

    args = argparser.parse_args()

    if args.schema == "latest":
        args.schema = LATEST
    pretty_print_schema(args.schema)

    # ----- Loading Interpreter ----
    interpreter = PyxcelInterpreter()
    logger.debug("Pyxcel Interpreter loaded successfully.")

    # ----- Deploying Schema ----
    server = autodeploy_schema(
        args.schema,
        interpreter=interpreter,
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

    while True:
        query = input("Request a Task: ")
        execute_user_query(query)


if __name__ == "__main__":
    main()
