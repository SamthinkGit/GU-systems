from langsmith import traceable

import action_space.experimental.screenshot.actions  # noqa
import action_space.keyboard.actions  # noqa
import action_space.meta.cognition_state.actions  # noqa
import action_space.meta.fake.actions  # noqa
import action_space.mouse.molmo_based.actions  # noqa
from cognition_layer.agents.minimal_vfr.variations.darkvfr import DarkVFR
from ecm.tools.item_registry_v2 import ItemRegistry
from execution_layer.pyxcel.interpreter.pyxcel_interpreter import PyxcelInterpreter


@traceable
def complete_task(prompt: str):
    for step in agent.complete_task(prompt):
        print(step)


if __name__ == "__main__":
    interpreter = PyxcelInterpreter()
    registry = ItemRegistry()
    registry.load_all()
    registry.summary()

    agent = DarkVFR(interpreter=interpreter, registry=registry)
    prompt = input("Prompt: ")
    complete_task(prompt)
