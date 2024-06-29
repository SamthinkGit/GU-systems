import pytest

import ecm.exelent.parser as parser
from ecm.exelent.verify import verify_valid_actions
from ecm.shared import get_root_path
from ecm.tools.registry import ItemRegistry


def test_parse_tasks() -> None:
    path = get_root_path() / "tests" / "resources" / "hello_world.xlnt"
    task = parser.parse(path)
    assert task.name == "my_task"

    # Now we will combine the 2 nested sequences (with 2 actions each) into 1
    linear_task = parser.linerize_task(task)
    assert len(linear_task.sequence[0].contains) == 4

    path = get_root_path() / "tests" / "resources" / "build_pickaxe.xlnt"
    task = parser.parse(path)
    assert task.name == "build_pickaxe"


def test_parse_nested_sequences():
    path = get_root_path() / "tests" / "resources" / "nested_sequences.xlnt"
    task = parser.parse(path)
    linear_task = parser.linerize_task(task)
    assert len(linear_task.sequence[0].contains) == 6


def test_parse_multiple_defs():
    path = get_root_path() / "tests" / "resources" / "multiple_defs.xlnt"
    task = parser.parse(path)
    assert len(task.sequence) == 3 and task.name == "my_task"


def test_misspelled_uppercase():
    path = get_root_path() / "tests" / "resources" / "uppercase_typo_task.xlnt"
    task = parser.parse(path)
    assert task.sequence[0].type.name == "Sequential"
    assert task.sequence[0].contains[0].name == "click"


@ItemRegistry.register_function
def valid():
    """MOCK"""
    ...


def test_invalid_actions():

    path = get_root_path() / "tests" / "resources" / "invalid_actions.xlnt"
    task = parser.parse(path)
    with pytest.raises(KeyError):
        verify_valid_actions(task)
