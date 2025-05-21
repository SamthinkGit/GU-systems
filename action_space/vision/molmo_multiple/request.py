import re
import tempfile

import replicate
from PIL import Image


def parse_molmo_points_response(molmo_response: str) -> list[tuple[int, int]]:
    matches = re.findall(r'[xX](\d*)="([\d.]+)"\s+[yY]\1="([\d.]+)"', molmo_response)
    points = [(float(x), float(y)) for _, x, y in matches]
    return points


def search_points_on_image(prompt: str, image: Image.Image) -> list[tuple[int, int]]:
    with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as tmp:
        image.save(tmp.name)

        output = replicate.run(
            "zsxkib/molmo-7b:76ebd700864218a4ca97ac1ccff068be7222272859f9ea2ae1dd4ac073fa8de8",
            input={
                "text": f"Point to all the objects that matches: `{prompt}` (at most 3)",
                "image": open(tmp.name, "rb"),
                "top_k": 5,
                "top_p": 1,
                "temperature": 1,
                "length_penalty": 1,
                "max_new_tokens": 150,
            },
        )

    return parse_molmo_points_response(output)
