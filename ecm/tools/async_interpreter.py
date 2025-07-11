# DEPRECATED: Use ecm.mediator.execution_layer_wrapper.py instead
import asyncio
from asyncio import Lock as ALock
from operator import methodcaller
from threading import Lock

import ecm.exelent.parser as parser
from cognition_layer.planex.utils.format import extract_python_code
from ecm.mediator.feedback import Feedback
from ecm.mediator.Interpreter import Interpreter
from ecm.shared import get_logger


class AsyncInterpreter:

    _last_feedback: asyncio.Future[Feedback] = None
    _last_exelent_code: str = None
    _interpreter: Interpreter
    _loop: asyncio.AbstractEventLoop
    _logger = get_logger("AsyncInterpreter")
    history: list[Feedback] = []
    _lock = Lock()
    _alock = ALock()

    @classmethod
    def build(cls, interpreter: Interpreter, feedback_class: Feedback):

        cls._loop = asyncio.get_event_loop()
        cls._interpreter = interpreter
        cls._feedback_class = feedback_class
        assert hasattr(
            feedback_class, "parse"
        ), "Invalid feedback class passed to the Interpreter, ensure "
        "to implement all functions from feedback templates."

    @classmethod
    def feedback_callback(cls, message):
        try:
            feedback = methodcaller("parse", message)(cls._feedback_class)
            if not issubclass(feedback.__class__, Feedback):
                raise ValueError(
                    "Feedback received is not valid. Verify the feedback types."
                )

            with cls._lock:
                cls.history.append(feedback)

                if cls._last_feedback is None or cls._last_feedback.done():
                    cls._last_feedback = cls._loop.create_future()

                cls._loop.call_soon_threadsafe(cls._last_feedback.set_result, feedback)

        except Exception as e:
            print(f"Error in feedback_callback: {e}")

    @classmethod
    async def get_feedback(cls):
        try:
            if cls._last_feedback is None:
                async with cls._alock:
                    if cls._last_feedback is None:  # Double-checked locking
                        cls._last_feedback = cls._loop.create_future()

            result = await cls._last_feedback

            async with cls._alock:
                cls._last_feedback = cls._loop.create_future()

            return result

        except Exception as e:
            print(f"Error in get_feedback: {e}")
            raise

    @classmethod
    def flush(cls):
        cls.history = []

    @classmethod
    def set_exelent_code(cls, code: str):
        cls._last_exelent_code = code

    @classmethod
    async def start_execution(cls):

        # ------------ PARSING EXELENT -------------------
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

            cls._loop.run_in_executor(
                None,
                cls._interpreter.arun,
                plan,
                cls.feedback_callback,
            )

        except NameError:
            cls._logger.error(
                "Cognition Layer generated an undefined function.", exc_info=True
            )

        except Exception:
            cls._logger.error("Invalid plan generated.", exc_info=True)
