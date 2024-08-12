from typing import Callable

import matplotlib.pyplot as plt
from PIL.Image import Image

import action_space.experimental.mouse.actions  # noqa
import action_space.experimental.screenshot.actions  # noqa
from action_space.experimental.mouse.grid import Cell
from action_space.experimental.mouse.grid import Grid
from action_space.tools.image import load_image
from ecm.tools.registry import ItemRegistry

# ---- Required Actions in ItemRegistry ----


class MouseDescriptor:

    screenshot_tool: Callable
    resolution: int
    iterations: int
    font_size: int
    _built: bool = False

    level: int = 0
    _grid: Grid = None
    _current_cell: Cell = None

    @classmethod
    def build(
        cls,
        screenshot_tool: Callable[[], Image],
        move_tool: Callable[[float, float], None],
        resolution: int = 10,
        font_size: int = 40,
    ) -> None:
        cls.screenshot_tool = screenshot_tool
        cls.resolution = resolution
        cls.font_size = font_size
        cls.move_tool = move_tool
        cls._built = True

    @classmethod
    def start(cls):
        cls._verify_building()

        screenshot = cls.screenshot_tool()
        cls._grid = Grid.from_image(
            screenshot, size=cls.resolution, font_size=cls.font_size
        )
        return cls._grid

    @classmethod
    def zoom(cls, cell: int):
        cls._verify_building()
        if cls._grid is None:
            raise SystemError(
                "Mouse descriptor has not been started. Please call MouseDescriptor.start() before using this tool."
            )

        cls._current_cell = cls._grid.cells[cell]
        cls._grid = cls._grid.zoom(cell)
        return cls._grid

    @classmethod
    def move(cls):
        coords = cls._current_cell.absolute()
        cls.move_tool(coords.center_x, coords.center_y)

    @classmethod
    def _verify_building(cls) -> None:
        if not cls._built:
            raise SystemError(
                "Mouse descriptor has not been built. Please call MouseDescriptor.build() before using this tool."
            )


def screenshot_tool():
    return load_image(ItemRegistry._utils["screenshot"]())


def move_tool(x, y):
    return ItemRegistry._utils["move_mouse_to"](x, y)


MouseDescriptor.build(
    screenshot_tool=screenshot_tool,
    move_tool=move_tool,
    resolution=10,
)


if __name__ == "__main__":

    grid = MouseDescriptor.start()
    plt.imshow(grid.image)
    plt.show()
    cell = int(input("Next cell: "))

    grid = MouseDescriptor.zoom(cell)
    plt.imshow(grid.image)
    plt.show()

    cell = int(input("Move to: "))
    MouseDescriptor.zoom(cell)
    MouseDescriptor.move()
