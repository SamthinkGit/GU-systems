import ecm.exelent.parser as parser
from ecm.mediator.rosa_interpreter import ExecutionStatus
from ecm.mediator.rosa_interpreter import RosaFeedbackWrapper as Feedback
from ecm.mediator.rosa_interpreter import RosaInterpreter
from ecm.shared import get_root_path
from ecm.tools.registry import ItemRegistry


class TestRosaInterpreter:

    rosa: RosaInterpreter
    mock = None
    clicks = 0
    types = 0

    def setup(self):
        self.rosa = RosaInterpreter()

        @ItemRegistry.register_function
        def click(*args, **kwargs):
            """MOCK"""
            TestRosaInterpreter.clicks += 1

        @ItemRegistry.register_function
        def type(*args, **kwargs):
            """MOCK"""
            TestRosaInterpreter.types += 1

    def test_function_calling(self):

        path = get_root_path() / "tests" / "resources" / "open_http_server.xlnt"
        task = parser.parse(path)
        self.rosa.run(task, callback="silent")
        assert TestRosaInterpreter.clicks == 3
        assert TestRosaInterpreter.types == 7
        TestRosaInterpreter.clicks = 0
        TestRosaInterpreter.types = 0

    def test_large_task(self):

        path = get_root_path() / "tests" / "resources" / "large_task.xlnt"
        task = parser.parse(path)
        self.rosa.run(task, callback="silent")
        assert TestRosaInterpreter.clicks == 3
        assert TestRosaInterpreter.types == 9
        TestRosaInterpreter.clicks = 0
        TestRosaInterpreter.types = 0

    def test_controlled_sequence(self):

        counter = 0

        def callback(message):
            nonlocal counter

            feedback = Feedback.parse(message)
            if counter == 3 and feedback._exec_status == ExecutionStatus.REQUEST_TO_CONTINUE:
                feedback.response(None, ExecutionStatus.ABORT)
                return

            if feedback._exec_status == ExecutionStatus.REQUEST_TO_CONTINUE:
                counter += 1
                feedback.response(None, ExecutionStatus.CONTINUE)

        path = get_root_path() / "tests" / "resources" / "controlled_plan.xlnt"
        task = parser.parse(path)
        self.rosa.run(task, callback=callback)
        assert TestRosaInterpreter.clicks == 3
