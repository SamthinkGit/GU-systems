from abc import ABC
from abc import abstractmethod
from dataclasses import dataclass
from dataclasses import Field

from PIL import Image


@dataclass
class BoundingBox:
    top_left: int
    top_right: int
    bottom_left: int
    bottom_right: int
    center: tuple[int, int]

    content: str
    additional_info: dict = Field(default_factory=dict)


class OcrEngine(ABC):

    def __init__(self, *args, **kwargs):
        pass

    @abstractmethod
    def invoke(self, image: Image.Image, *args, **kwargs) -> list[BoundingBox]:
        raise NotImplementedError()

    def clean(self, *args, **kwargs):
        pass
