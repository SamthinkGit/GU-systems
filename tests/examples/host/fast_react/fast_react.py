import action_space.experimental.screenshot.actions  # noqa
import action_space.keyboard.actions  # noqa
from cognition_layer.fast_react.agents.fast_react import FastReact
from ecm.tools.item_registry_v2 import ItemRegistry
from execution_layer.pyxcel.interpreter.pyxcel_interpreter import PyxcelInterpreter

from langsmith import traceable


@traceable
def fast_react():
    ItemRegistry().load_all()

    interpreter = PyxcelInterpreter()
    fr = FastReact(interpreter=interpreter)

    for step in fr.complete_task("Escribe Hello World!"):
        print(step)


if __name__ == "__main__":
    fast_react()
