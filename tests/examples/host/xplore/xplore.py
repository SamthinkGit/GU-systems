import random
import time

import action_space.experimental.mouse.actions  # noqa
import action_space.experimental.wait_for.actions  # noqa
import action_space.keyboard.actions  # noqa
from ecm.tools.item_registry_v2 import ItemRegistry

from cognition_layer.xplore.agents.xplore import Xplore
from ecm.tools.prettify import pretty_head
from ecm.tools.prettify import pretty_print
from execution_layer.rosa.interpreter.rosa_interpreter import RosaInterpreter

# =====================================================
# [WARNING] This script will run in the HOST machine!!
# =====================================================

if __name__ == "__main__":

    ItemRegistry().load_all()

    rosa = RosaInterpreter()
    xplore = Xplore(interpreter=rosa)
    print(pretty_head("Graph"))
    print(xplore.graph.get_graph().draw_ascii())

    user_input = input("Xplore Graph: ")
    time.sleep(1)
    config = {"configurable": {"thread_id": str(random.randint(5000, 15000))}}

    try:
        for event in xplore.graph.stream(
            {"query": user_input}, config, stream_mode="values"
        ):
            pretty_print(event, header=event.get("current_node"))

    except KeyboardInterrupt:
        print("Process Interrupted, exiting")

    finally:
        rosa.kill()
