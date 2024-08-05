"""
This script defines classes and functions to handle a grid system over an image.
It allows zooming into specific cells and getting the absolute position of cells
based on hierarchical zoom operations.

[Warning] Absolute coordinates does not work with variable grid size.
"""
import functools
from dataclasses import dataclass
from dataclasses import field
from typing import Dict
from typing import Optional
from typing import Sequence

import matplotlib.pyplot as plt
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont


@dataclass
class Cell:
    """
    Represents a single cell within a grid.
    """

    id: int
    left: float
    right: float
    top: float
    bottom: float
    father: "Grid" = field(repr=False)
    center_x: float = field(init=False)
    center_y: float = field(init=False)

    def __post_init__(self):
        self.center_x = (self.left + self.right) / 2
        self.center_y = (self.top + self.bottom) / 2

    def merge(self, other: "Cell", size: int) -> "Cell":
        """
        Merges another cell into this one, adjusting for the size scaling.

        Args:
            other (Cell): The other cell to merge.
            size (int): The scaling size factor.

        Returns:
            Cell: A new merged cell.
        """
        return Cell(
            id=other.id,
            father=other.father,
            left=self.left + other.left / size,
            right=self.left + other.right / size,
            top=self.top + other.top / size,
            bottom=self.top + other.bottom / size,
        )

    def absolute(self) -> "Cell":
        """
        Calculates the absolute position of the cell within the nested grid structure.

        Warning:
            This function assumes the size of the father equal for all nodes.
            (get_absolute_cell() dependant.)

        Returns:
            Cell: The cell with absolute coordinates.
        """
        nodes = [self.id]
        father = self.father
        while father.id is not None:
            nodes.append(father.id)
            father = father.father
        return father.get_absolute_cell(nodes[::-1])


@dataclass
class RelativeCell:
    cell: Cell
    size: int


@dataclass
class Grid:
    """
    Represents a grid of cells over an image.
    """

    size: int
    image: Image.Image
    original_image: Image.Image
    cells: Dict[int, Cell]
    cell_width: float
    cell_height: float

    id: int = None
    father: "Grid" = None

    @classmethod
    def from_image(
        cls,
        image: Image.Image,
        size: int,
        font_size: int = 40,
    ) -> "Grid":
        """
        Creates a grid from an image by dividing it into cells. The returned grid will
        contain the grid drawn into the .image attribute.

        Args:
            image (Image.Image): The image to create the grid from.
            size (int): The number of cells along one dimension.
            font_size (int, optional): Font size for cell numbering. Defaults to 40.

        Returns:
            Grid: The generated grid.
        """

        image_copy = image.copy()
        original_image = image.copy()
        draw = ImageDraw.Draw(image_copy)

        width, height = image.size
        cell_width = width / size
        cell_height = height / size

        font = ImageFont.load_default(size=font_size)
        cells: dict[int, Cell] = {}

        grid = Grid(
            original_image=original_image,
            image=image_copy,
            cells=cells,
            size=size,
            cell_height=cell_height,
            cell_width=cell_width,
        )

        for i in range(size):
            for j in range(size):

                number = i * size + j + 1
                left, top, right, bottom = Grid.get_cell_coords(
                    i, j, cell_width, cell_height
                )

                cell = Cell(
                    number, left=left, right=right, top=top, bottom=bottom, father=grid
                )

                draw.rectangle([left, top, right, bottom], outline="white", width=2)
                draw.rectangle([left, top, right, bottom], outline="grey", width=1)

                text = str(number)
                draw.text(
                    (cell.center_x, cell.center_y),
                    text,
                    fill="white",
                    font=font,
                    stroke_width=2,
                    stroke_fill="blue",
                )
                cells[number] = cell

        return grid

    def zoom(self, cell_id: int, size: Optional[int] = None) -> "Grid":
        """
        Zooms into a specific cell and creates a new grid from it.

        Args:
            cell_id (int): The ID of the cell to zoom into.
            size (Optional[int], optional): The size of the new grid. Defaults to None.
            rows (Optional[int], optional): Number of rows for the new grid. Defaults to None.

        Returns:
            Grid: The new grid created by zooming into the specified cell.
        """

        if size is None:
            size = self.size

        original_size = self.image.size
        cell = self.cells[cell_id]

        cropped_image = self.original_image.crop(
            (cell.left, cell.top, cell.right, cell.bottom)
        )
        resized_image = cropped_image.resize(original_size, Image.LANCZOS)
        grid = Grid.from_image(resized_image, size=size)
        grid.father = self
        grid.id = cell_id

        return grid

    def get_absolute_cell(self, cells_id: Sequence[int]):
        """
        Gets the absolute cell position based on a sequence of cell IDs through hierarchical grids.

        Args:
            cells_id (Sequence[int]): The sequence of cell IDs to trace back.

        Example:
            grid = Grid.from_image(image, size=10)
            cell = grid.get_absolute_cell([1,14,2])

        Returns:
            Cell: The cell with absolute coordinates.
        """
        relations = []
        grid = self
        for id in cells_id:
            size = grid.size
            cell = grid.cells[id]
            grid = grid.zoom(id)
            relations.append(RelativeCell(cell, size))

        relations[-1] = relations[-1].cell

        result: Cell = functools.reduce(
            lambda x, y: y.cell.merge(x, size=y.size),
            relations[::-1],
        )
        return result

    @staticmethod
    def get_cell_coords(
        pos_x, pos_y, cell_width, cell_height
    ) -> list[float, float, float, float]:
        """
        Calculates the coordinates of a cell in the grid.

        Args:
            pos_x (int): The x position of the cell in the grid.
            pos_y (int): The y position of the cell in the grid.
            cell_width (float): The width of the cell.
            cell_height (float): The height of the cell.

        Returns:
            list: The coordinates [left, top, right, bottom] of the cell.
        """

        left = pos_x * cell_width
        top = pos_y * cell_height
        right = left + cell_width
        bottom = top + cell_height
        return [left, top, right, bottom]


if __name__ == "__main__":

    image_path = "screenshot.png"
    image = Image.open(image_path)
    grid = Grid.from_image(image, size=10)
    plt.figure(figsize=(image.width / 100, image.height / 100), dpi=100)
    plt.title("Original")
    plt.imshow(grid.image)
    plt.axis("off")
    plt.show()

    selection = int(input("Cell: "))
    sub_grid = grid.zoom(selection, size=5)
    plt.figure(figsize=(image.width / 100, image.height / 100), dpi=100)
    plt.title("Zoom")
    plt.imshow(sub_grid.image)
    plt.axis("off")
    plt.show()
