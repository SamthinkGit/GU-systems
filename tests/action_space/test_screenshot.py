import action_space.experimental.screenshot.actions  # noqa
from action_space.tools.image import load_image
from ecm.tools.item_registry_v2 import ItemRegistry

import numpy as np

RED = np.array([255, 0, 0], dtype=np.uint8)
BLUE = np.array([0, 0, 255], dtype=np.uint8)


def test_screenshot_capturing_colors(tracker):
    del tracker.clicks[:]
    registry = ItemRegistry(name="test_screenshot_capturing_colors")
    registry.load_all()
    screenshot = load_image(registry.tools["screenshot"].content())
    matrix = np.array(screenshot)

    height, width, _ = matrix.shape
    middle_width = width / 2
    middle_height = height / 2

    assert np.array_equal(
        matrix[int(middle_height), int(middle_width + middle_width / 2)], RED
    )
    assert np.array_equal(
        matrix[int(middle_height), int(middle_width - middle_width / 2)], BLUE
    )
