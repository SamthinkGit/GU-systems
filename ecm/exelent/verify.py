"""
Parsed Task Utilities Module
==============================

This module provides utility functions for walking through and verifying parsed tasks in the Exelent framework.
It includes functions to traverse parsed tasks and validate the actions within them, ensuring that all components
are properly processed and verified.
"""
from typing import Any

from ecm.exelent.parser import ParsedAction
from ecm.exelent.parser import ParsedTask
from ecm.exelent.parser import ParsedWith
from ecm.tools.registry import ItemRegistry


def walk(parsed_task: ParsedTask) -> Any:
    """
    Generator function that walks through the entire parsed task, yielding each component in the sequence.

    :param parsed_task: The ParsedTask object to walk through.
    :yield: Each component (ParsedTask, ParsedWith, ParsedAction) in the parsed task.
    :raises TypeError: If an invalid node type is found.
    """
    yield parsed_task

    for parsed_with in parsed_task.sequence:
        yield parsed_with

        for item in parsed_with.contains:
            if isinstance(item, ParsedWith):
                yield from walk_parsed_with(item)

            elif isinstance(item, ParsedAction):
                yield item

            else:
                raise TypeError(
                    f"Invalid node found inside a with 'contains': {type(item)}"
                )


def walk_parsed_with(parsed_with: ParsedWith) -> Any:
    """
    Helper generator function that walks through a ParsedWith object, yielding each item contained within it.

    :param parsed_with: The ParsedWith object to walk through.
    :yield: Each item (ParsedWith, ParsedAction) contained within the ParsedWith object.
    :raises TypeError: If an invalid node type is found.
    """
    yield parsed_with
    for item in parsed_with.contains:
        if isinstance(item, ParsedWith):
            yield from walk_parsed_with(item)
        elif isinstance(item, ParsedAction):
            yield item
        else:
            raise TypeError(
                f"Invalid node found inside a with 'contains': {type(item)}"
            )


def verify_valid_actions(parsed_task: ParsedTask) -> Exception | None:
    """
    Verifies that all actions within a parsed task are valid by checking them against the ItemRegistry.

    :param parsed_task: The ParsedTask object to verify.
    :return: None if all actions are valid, otherwise raises an appropriate exception.
    """
    for node in walk(parsed_task):
        if isinstance(node, ParsedAction):
            node: ParsedAction
            ItemRegistry.get_from_name(node.name)
