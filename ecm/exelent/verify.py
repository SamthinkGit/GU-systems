from typing import Any

from ecm.exelent.parser import ParsedAction
from ecm.exelent.parser import ParsedTask
from ecm.exelent.parser import ParsedWith
from ecm.tools.registry import ItemRegistry


def walk(parsed_task: ParsedTask) -> Any:
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
    for node in walk(parsed_task):
        if isinstance(node, ParsedAction):
            node: ParsedAction
            ItemRegistry.get_from_name(node.name)
