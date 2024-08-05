import base64
import io

from PIL import Image


def return_image(image: Image.Image):
    buffered = io.BytesIO()
    image.save(buffered, format="PNG")
    img_bytes = buffered.getvalue()
    return base64.b64encode(img_bytes).decode("utf-8")


def load_image(image_bytes: bytes):
    img_bytes = base64.b64decode(image_bytes)
    buffered = io.BytesIO(img_bytes)
    img = Image.open(buffered)
    return img
