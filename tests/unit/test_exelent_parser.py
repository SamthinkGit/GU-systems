import ecm.exelent.parser as parser
from ecm.shared import get_root_path


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
