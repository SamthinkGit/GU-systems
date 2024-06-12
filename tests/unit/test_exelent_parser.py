import ecm.exelent.parser as parser
from execution_layer.rosa.shared import get_root_path


def test_parse_tasks() -> None:
    path = get_root_path() / "tests" / "resources" / "hello_world.xlnt"
    print(parser.parse(path))
