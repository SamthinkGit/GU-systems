import time

import action_space.mouse.ostools.actions  # noqa
from ecm.tools.item_registry_v2 import ItemRegistry


def test_mouse_is_clicking(tracker):
    del tracker.clicks[:]
    registry = ItemRegistry(name="test_mouse_is_clicking")
    registry.load_all()
    click_function = registry.get("send_click_event").content
    for _ in range(3):
        click_function()

    time.sleep(0.1)
    assert len(tracker.clicks) == 3


def test_moving_to_corners(tracker):
    del tracker.clicks[:]
    registry = ItemRegistry(name="test_moving_mouse_to_corners")
    registry.load_all()
    click_function = registry.get("send_click_event").content

    INF = 10000000

    registry.get("move_mouse_to").content(0, 0)
    click_function()
    time.sleep(0.1)
    assert tracker.clicks[-1].side == "blue"

    registry.get("move_mouse_to").content(0, INF)
    click_function()
    time.sleep(0.1)
    assert tracker.clicks[-1].side == "blue"

    registry.get("move_mouse_to").content(INF, 0)
    click_function()
    time.sleep(0.1)
    assert tracker.clicks[-1].side == "red"

    registry.get("move_mouse_to").content(INF, INF)
    click_function()
    time.sleep(0.1)
    assert tracker.clicks[-1].side == "red"


def test_moving_mouse(tracker):
    del tracker.clicks[:]
    registry = ItemRegistry(name="test_moving_mouse")
    registry.load_all()
    click_function = registry.get("send_click_event").content

    registry.get("move_mouse_to").content(300, 300)
    click_function()
    time.sleep(0.1)
    assert tracker.clicks[-1].x == 300
    assert tracker.clicks[-1].y == 300

    registry.get("move_mouse_to").content(600, 600)
    click_function()
    time.sleep(0.1)
    assert tracker.clicks[-1].x == 600
    assert tracker.clicks[-1].y == 600

    registry.get("move_mouse_to").content(200, 600)
    click_function()
    time.sleep(0.1)
    assert tracker.clicks[-1].x == 200
    assert tracker.clicks[-1].y == 600


def test_mouse_buttons(tracker):
    del tracker.clicks[:]
    registry = ItemRegistry(name="test_mouse_buttons")
    registry.load_all()
    registry.get("send_click_event").content()
    time.sleep(0.1)
    assert tracker.clicks[-1].button == "Left"
