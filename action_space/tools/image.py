"""
ImageMessage
================
This script enhances the langchain BaseMessages with a new
ImageMessage, which will contain an image and can be sent as
AI/Human/System to the OpenaiAPI.

Example:

```python
    prompt = ChatPromptTemplate(
        [
            SystemMessage(content="Im a system Message"),
            ImageMessage(
                image=screenshot,
                input="Look into this image",
            ).as_human()
        ]
    )
```
"""
import base64
import io
from typing import Literal

from langchain_core.messages import AIMessage
from langchain_core.messages import BaseMessage
from langchain_core.messages import HumanMessage
from langchain_core.messages import SystemMessage
from PIL import Image


class ImageMessage:

    def __init__(
        self,
        image: Image.Image,
        input: str = "",
        llm: Literal["GPT"] = "GPT",
        detail: Literal["high", "low"] = "high",
    ) -> None:
        image_bytes = return_image(image)
        if llm == "GPT":
            self.content = [
                {"type": "text", "text": input},
                {
                    "type": "image_url",
                    "image_url": {
                        "url": f"data:image/jpeg;base64,{image_bytes}",
                        "detail": detail,
                    },
                },
            ]
        else:
            raise SystemError(f"ImageMessages for {llm} are not supported yet.")

    def as_human(self) -> HumanMessage:
        return HumanMessage(content=self.content)

    def as_ai(self) -> HumanMessage:
        return AIMessage(content=self.content)

    def as_system(self) -> HumanMessage:
        return SystemMessage(content=self.content)

    def wrap(self, message_class: BaseMessage):
        return message_class(content=self.content)


def return_image(image: Image.Image):
    buffered = io.BytesIO()
    image.save(buffered, format="PNG")
    img_bytes = buffered.getvalue()
    return base64.b64encode(img_bytes).decode("utf-8")


# Maintained in order to support previous versions
def load_image(image_bytes: bytes) -> Image.Image:
    img_bytes = base64.b64decode(image_bytes)
    buffered = io.BytesIO(img_bytes)
    img = Image.open(buffered)
    return img
