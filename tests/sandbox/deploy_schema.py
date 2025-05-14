import json

from cognition_layer.deploy.loader import deploy
from cognition_layer.deploy.loader import get_deploy_model
from ecm.shared import get_logger
from ecm.shared import get_root_path
from ecm.tools.registry import ItemRegistry
from execution_layer.pyxcel.interpreter.pyxcel_interpreter import PyxcelInterpreter

if __name__ == "__main__":
    logger = get_logger("Main")
    interpreter = PyxcelInterpreter()
    path = (
        get_root_path()
        / "cognition_layer"
        / "routing"
        / "schemas"
        / "full_feedback_routing.json"
    )

    schema = json.loads(path.read_text())
    model = get_deploy_model(schema["model"])
    server = deploy(model, interpreter, schema=schema, config=schema["config"])

    ItemRegistry().summary()

    query = input("Enter your query: ")

    for step in server.send_task(query):
        logger.info(f"Step: {step}")
