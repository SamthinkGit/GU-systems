"""
Execution Layer Wrapper Module
==============================

This module defines the `ExecutionLayerWrapper` class, which provides a wrapper for managing and executing tasks
using an interpreter. It handles the initialization of the interpreter, parsing of Exelent code, and execution
of the parsed plan. It also manages feedback from the execution layer, maintaining a history of feedback messages.

[HELP] This module is used by the cognition_layer.RePlan.agents.replan in order to fully manage the execution layer
easily with an agent, use it as an example.
[HELP] Use the tests/unit/test_execution_layer_wrapper.py file as an example of this module implemented with ROSA
as execution layer.
"""
from operator import methodcaller
from threading import Lock

import ecm.exelent.parser as parser
from cognition_layer.planex.utils.format import extract_python_code
from ecm.mediator.feedback import Feedback
from ecm.mediator.Interpreter import Interpreter
from ecm.shared import get_logger


class ExecutionLayerWrapper:

    _last_exelent_code: str = None
    _interpreter: Interpreter
    _logger = get_logger("ExecutionLayerWrapper")
    history: list[Feedback] = []
    _lock = Lock()

    @classmethod
    def build(cls, interpreter_class: Interpreter, feedback_class: Feedback):
        # TODO: Change this to __init__

        cls._interpreter = interpreter_class
        cls._feedback_class = feedback_class
        cls._interpreter_inited = False
        assert hasattr(
            feedback_class, "parse"
        ), "Invalid feedback class passed to the Interpreter, ensure "
        "to implement all functions from feedback templates."

    @classmethod
    def feedback_callback(cls, message):
        """
        Callback function to handle feedback messages from the execution layer.
        It will just parse it and save it to the history.
        :param message: The feedback message to be parsed and stored.
        """

        try:
            feedback = methodcaller("parse", message)(cls._feedback_class)
            if not issubclass(feedback.__class__, Feedback):
                raise ValueError(
                    "Feedback received is not valid. Verify the feedback types."
                )

            with cls._lock:
                cls.history.append(feedback)

        except Exception as e:
            print(f"Error in feedback_callback: {e}")

    @classmethod
    def flush(cls):
        cls.history = []

    @classmethod
    def set_exelent_code(cls, code: str):
        cls._last_exelent_code = code

    @classmethod
    def start_execution(cls):
        """
        Starts the execution of the Exelent code using the interpreter. Initializes the
        interpreter if it hasn't been initialized yet, parses the Exelent code, and
        executes the parsed plan. Collects feedback during execution.
        You can obtain the feedback from this execution by looking with:
        ```python
        with execution_layer_wrapper._lock():
          print(execution_layer_wrapper.history)
        ```
        """

        # IMPORTANT: We init all the interpreter memory here so there are not different threads
        # when running the interpreter commands. It could run into different memory spaces and fail.
        if not cls._interpreter_inited:
            cls._interpreter = cls._interpreter()
            cls._interpreter_inited = True

        # ------------ PARSING EXELENT -------------------
        cls._logger.debug(f"Exelent received: {cls._last_exelent_code}")
        plan = cls._last_exelent_code
        if plan.startswith("```python"):
            plan = extract_python_code(plan)

        try:
            plan = parser.parse(target_str=plan)

        except Exception as exception_for_ai:
            cls._logger.error("Invalid plan generated.", exc_info=True)
            raise exception_for_ai

        # ------------ EXECUTION LAYER -------------------
        try:
            cls._logger.debug("Generated Packages:")
            for pkg in cls._interpreter._generate_packages_from_parsed_task(plan):
                cls._logger.debug(pkg.to_json())
            cls._logger.debug("Running Packages...")
            cls._interpreter.arun(plan, cls.feedback_callback)

        except NameError:
            cls._logger.error(
                "Cognition Layer generated an undefined function.", exc_info=True
            )

        except Exception:
            cls._logger.error("Invalid plan generated.", exc_info=True)
