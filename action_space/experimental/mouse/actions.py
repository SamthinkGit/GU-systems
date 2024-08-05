from action_space.experimental.mouse.calibrated_move import move
from ecm.shared import load_env
from ecm.tools.registry import ItemRegistry
load_env()


@ItemRegistry.register_util
def move_mouse_to(x, y):
    move(x, y)
