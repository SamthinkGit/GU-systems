import string
from itertools import count
from itertools import product
from typing import Callable
from typing import Generator
from typing import Literal

import cv2
import numpy as np
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

from cognition_layer.tools.ocr.template import BoundingBox
from cognition_layer.tools.ocr.template import cv2_to_pil
from cognition_layer.tools.ocr.template import pil_to_cv2


class Labeler:

    _latest_instance: "Labeler" = None

    def __init__(self, image: Image, boxes: list[BoundingBox]):
        """
        Initialize the Labeler with an image and bounding boxes.

        :param image: Path to the image file.
        :param boxes: List of BoundingBox objects representing detected regions.
        """
        self.image = image
        self.boxes = boxes
        self.labelled_boxes = {
            label: box for label, box in zip(self.stream_labels(), self.boxes)
        }

    def draw(
        self,
        color=(255, 0, 0),
        box_thickness=2,
        labellize: bool = True,
        font_scale: float = 0.5,
        font_thickness: int = 2,
        font_color: tuple[int, int, int] | Literal["random"] = "random",
        filter: Callable = lambda x: True,
    ) -> Image.Image:
        """
        Draw bounding boxes on the image.
        :param color: Color of the bounding boxes (default is red).
        :param thickness: Thickness of the bounding box lines.
        :return: Image with bounding boxes drawn.
        """
        cvimage = pil_to_cv2(self.image)

        for label, box in self.labelled_boxes.items():
            if not filter(box):
                continue

            result_color = color
            if font_color == "random" or color == "random":
                result_color = get_random_color()

            top_left = tuple(map(int, box.top_left))
            bottom_right = tuple(map(int, box.bottom_right))
            cvimage = cv2.rectangle(
                cvimage, top_left, bottom_right, result_color, box_thickness
            )
            if labellize:
                font = cv2.FONT_HERSHEY_SIMPLEX
                (text_width, text_height), _ = cv2.getTextSize(
                    label, font, font_scale, font_thickness
                )

                if text_width > box.bottom_right[0] - box.top_left[0]:
                    textpos = (box.bottom_right[0] - text_width, box.top_left[1])
                elif text_height > box.bottom_right[1] - box.top_left[1]:
                    textpos = (box.top_left[0], box.bottom_right[1] - text_height)
                else:
                    textpos = (box.top_left[0], box.top_left[1])

                textpos = (int(textpos[0]), int(textpos[1]))
                cvimage = cv2.putText(
                    cvimage,
                    label,
                    textpos,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale,
                    result_color,
                    font_thickness,
                )
        return cv2_to_pil(cvimage)

    def freeze(self) -> "Labeler":
        """
        Freeze the current instance of the Labeler class.
        :return: The frozen instance of the Labeler class.
        """
        Labeler._latest_instance = self
        return self

    @classmethod
    def get_freezed_instance(cls) -> "Labeler":
        """
        Get the latest frozen instance of the Labeler class.
        :return: The latest frozen instance of the Labeler class.
        """
        if cls._latest_instance is None:
            raise ValueError("No instance has been frozen yet.")
        return cls._latest_instance

    def board(
        self,
        columns: int = 14,
        icon_size: int = 40,
        header_height: int = 20,
        font_size: int = 16,
        filter: Callable = lambda x: True,
    ) -> Image.Image:
        """
        Create a board of icons with labels.
        :param columns: Number of columns in the board.
        :param icon_size: Size of each icon in pixels.
        :param header_height: Height of the header in pixels.
        :param font_size: Font size for the labels.
        :return: Image of the board with icons and labels.
        """
        frame_size = (icon_size, icon_size + header_height)
        processed_icons = []

        for label, bbox in self.labelled_boxes.items():
            if not filter(bbox):
                continue

            x_min = min(bbox.top_left[0], bbox.bottom_left[0])
            x_max = max(bbox.top_right[0], bbox.bottom_right[0])
            y_min = min(bbox.top_left[1], bbox.top_right[1])
            y_max = max(bbox.bottom_left[1], bbox.bottom_right[1])

            board = self.image.crop((x_min, y_min, x_max, y_max))
            board = board.resize((icon_size, icon_size))

            frame = Image.new("RGB", frame_size, "white")
            frame.paste(board, (0, header_height))

            draw = ImageDraw.Draw(frame)
            try:
                font = ImageFont.truetype("arial.ttf", font_size)
            except OSError:
                font = ImageFont.load_default()

            bbox_text = draw.textbbox((0, 0), label, font=font)
            text_width = bbox_text[2] - bbox_text[0]
            text_height = bbox_text[3] - bbox_text[1]
            draw.text(
                ((icon_size - text_width) // 2, (header_height - text_height) // 2),
                label,
                fill="black",
                font=font,
            )

            processed_icons.append(frame)

        rows = (len(processed_icons) + columns - 1) // columns
        width_tabla = columns * icon_size
        height_tabla = rows * frame_size[1]

        table = Image.new("RGB", (width_tabla, height_tabla), "white")

        for idx, icono in enumerate(processed_icons):
            row = idx // columns
            col = idx % columns
            x = col * icon_size
            y = row * frame_size[1]
            table.paste(icono, (x, y))

        return table

    @staticmethod
    def stream_labels() -> Generator[str, None, None]:
        """
        Generate infinite labels for the bounding boxes.
        Example:
            A1, A2, ..., A9, B1, ..., Z9, AA1, ..., ZZ9, AAA1, ...
        :return: A generator yielding labels for each bounding box.
        """
        letters = string.ascii_uppercase

        for size in count(1):
            for prefix in product(letters, repeat=size):
                prefix_str = "".join(prefix)
                for number in range(1, 10):
                    yield f"{prefix_str}{number}"


def get_random_color() -> tuple[int, int, int]:
    """
    Generate a random color in RGB format.
    :return: A tuple representing the RGB color.
    """
    return tuple(
        int(c) for c in (np.random.rand(3) * 255).astype(int).tolist()
    )  # type: ignore
