"""
RePlan Server Module
==============================

This module defines functions to set up servers for handling tasks using the RePlan
and AsyncRePlan agents. It integrates with the cognition layer's execution framework
and provides server configurations for both synchronous and asynchronous processing
of input queries.

Note that this servers fully integrates with Agent Protocol
"""
from operator import attrgetter

from cognition_layer.api import ServerAPI
from cognition_layer.RePlan.agents.async_replan import AsyncRePlan
from cognition_layer.RePlan.agents.replan import RePlan
from cognition_layer.tools.server_template import ServerFromIterator
from ecm.mediator.execution_layer_wrapper import ExecutionLayerWrapper
from ecm.mediator.Interpreter import Interpreter
from ecm.tools.async_interpreter import AsyncInterpreter


def get_server(
    verbose, interpreter_class: Interpreter, max_planex_steps: int = 11
) -> ServerAPI:
    """
    Initializes and returns a ServerAPI instance configured to use the RePlan
    agent for processing queries.

    :param verbose: Flag to enable verbose logging of the server's processing steps.
    :param interpreter_class: The interpreter class to be used by the execution layer.
    You can use any interpreter from the ecm / execution layer
    :param max_planex_steps: Maximum number of steps for the RePlan agent.
    :return: An instance of ServerAPI configured with the RePlan agent.
    """
    execution_layer_wrapper = ExecutionLayerWrapper()
    execution_layer_wrapper.build(
        interpreter_class=interpreter_class, feedback_class=interpreter_class.feedback_message_class
    )
    replan = RePlan(
        execution_layer_wrapper=execution_layer_wrapper,
        verbose=verbose,
        max_planex_steps=max_planex_steps,
    )
    return ServerFromIterator(
        name="RePlan Server",
        iterator=lambda input: replan.iter(input),
        async_iterator=True,  # Only the iterator is async here.
        step_name_getter=attrgetter("name"),
        content_getter=attrgetter("content"),
        is_last_getter=attrgetter("is_last"),
    )


# DEPRECATED:
def get_async_server(
    verbose, interpreter: Interpreter, max_planex_steps: int = 11
) -> ServerAPI:
    async_interpreter = AsyncInterpreter()
    async_interpreter.build(
        interpreter=interpreter, feedback_class=interpreter.feedback_message_class
    )
    replan = AsyncRePlan(
        async_interpreter=async_interpreter,
        verbose=verbose,
        max_planex_steps=max_planex_steps,
    )
    return ServerFromIterator(
        name="Async RePlan Server",
        iterator=lambda input: replan.iter(input),
        async_iterator=True,
        step_name_getter=attrgetter("name"),
        content_getter=attrgetter("content"),
        is_last_getter=attrgetter("is_last"),
    )
