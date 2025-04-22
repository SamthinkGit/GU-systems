import time

import action_space.experimental.screenshot.actions  # noqa
import action_space.keyboard.actions  # noqa
import action_space.meta.actions  # noqa
import action_space.mouse.molmo_based.actions  # noqa
from cognition_layer.agents.minimal_vfr.agents.agent import MinimalVFR
from ecm.tools.item_registry_v2 import ItemRegistry
from execution_layer.pyxcel.interpreter.pyxcel_interpreter import PyxcelInterpreter

if __name__ == "__main__":
    interpreter = PyxcelInterpreter()
    registry = ItemRegistry()
    registry.load_all()
    registry.summary()

    agent = MinimalVFR(interpreter=interpreter, registry=registry)
    prompt = input("Prompt: ")
    time.sleep(1)

    for step in agent.complete_task(prompt):
        print(step)
