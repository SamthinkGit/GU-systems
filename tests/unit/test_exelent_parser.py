import ecm.exelent.parser as parser
from ecm.shared import get_root_path


def test_parse_tasks() -> None:
    path = get_root_path() / "tests" / "resources" / "hello_world.xlnt"
    task = parser.parse(path)
    assert task.name == "my_task"

    path = get_root_path() / "tests" / "resources" / "build_pickaxe.xlnt"
    task = parser.parse(path)
    assert task.name == "build_pickaxe"
