import ecm.exelent.parser as parser
from ecm.exelent.builder import ExelentBuilder
from ecm.shared import get_root_path


def test_simple_building():
    path = get_root_path() / "tests" / "resources" / "write.xlnt"
    parsed_task = parser.parse(path)

    builder = ExelentBuilder()
    builder.add_task("write_task")
    builder.add_type("Sequential")
    builder.add_statement('write("hello world")')
    compiled_task = builder.compile()

    assert parsed_task == compiled_task
    assert parsed_task.sequence == compiled_task.sequence
    assert parsed_task.sequence[0] == compiled_task.sequence[0]
