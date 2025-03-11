import action_space.experimental.screenshot.actions  # noqa
import action_space.keyboard.actions  # noqa
from cognition_layer.fast_react.agents.fast_react import FastReact
from ecm.tools.item_registry_v2 import ItemRegistry
from execution_layer.pyxcel.interpreter.pyxcel_interpreter import PyxcelInterpreter

if __name__ == "__main__":
    ItemRegistry().load_all()

    interpreter = PyxcelInterpreter()
    fr = FastReact(interpreter=interpreter)

    for step in fr.complete_task("Escribe un pequeño poema"):
        print(step)
