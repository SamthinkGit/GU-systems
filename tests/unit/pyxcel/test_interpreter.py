import ecm.exelent.parser as parser
from ecm.shared import get_root_path
from ecm.tools.item_registry_v2 import ItemRegistry
from execution_layer.pyxcel.interpreter.pyxcel_interpreter import PyxcelInterpreter


class TestPyxcelInterpreter:

    pyxcel: PyxcelInterpreter
    mock = None
    clicks = 0
    types = 0
    tests = 0

    def setup(self):
        self.pyxcel = PyxcelInterpreter()

        @ItemRegistry.register(type="action")
        def click(*args, **kwargs):
            """MOCK"""
            TestPyxcelInterpreter.clicks += 1

        @ItemRegistry.register_function
        def type(*args, **kwargs):
            """MOCK"""
            TestPyxcelInterpreter.types += 1

        @ItemRegistry.register_function
        def test(*args, **kwargs):
            """MOCK"""
            TestPyxcelInterpreter.tests += 1

        ItemRegistry().load_all()

    def test_function_calling(self):

        path = get_root_path() / "tests" / "resources" / "open_http_server.xlnt"
        task = parser.parse(path)
        self.pyxcel.run(task, callback="silent")
        assert TestPyxcelInterpreter.clicks == 3
        assert TestPyxcelInterpreter.types == 7
        TestPyxcelInterpreter.clicks = 0
        TestPyxcelInterpreter.types = 0

    def test_large_task(self):

        path = get_root_path() / "tests" / "resources" / "large_task.xlnt"
        task = parser.parse(path)
        self.pyxcel.run(task, callback="silent")
        assert TestPyxcelInterpreter.clicks == 3
        assert TestPyxcelInterpreter.types == 9
        TestPyxcelInterpreter.clicks = 0
        TestPyxcelInterpreter.types = 0

    def test_nested_tasks(self):
        path = get_root_path() / "tests" / "resources" / "nested_sequences.xlnt"
        task = parser.parse(path)
        self.pyxcel.run(task, callback="silent")
        assert TestPyxcelInterpreter.tests == 6
        TestPyxcelInterpreter.tests = 0
