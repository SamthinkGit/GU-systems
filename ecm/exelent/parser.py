"""
Exelent Parser Module
=================================

This module provides the `ExelentParser` class for parsing and analyzing tasks defined
in Python code. The parser can handle tasks specified as functions and their associated
actions and types within `with` statements. Key classes include `ParsedTask`, `ParsedWith`,
`ParsedAction`, and `ParsedType`, each representing various elements of a task.

The `ExelentParser` can be initialized with either a file path or a string containing the
code to be parsed. The module supports parsing the function calls and with-blocks,
distinguishing between actions and types based on naming conventions.

Additionally, the module offers the `linerize_task` function to linearize nested `with`
blocks into a sequence of actions for easier processing.

Functions:
- [Recommended] parse: Parses a given target file or string, returning a ParsedTask object.

Classes:
- ExelentParser: Main class for parsing Exelent Files (.xlnt).
- ParsedAction: Represents a parsed Action from a function call.
- ParsedType: Represents a parsed Type from a type call.
- ParsedWith: Represents a parsed With block containing types and actions.
- ParsedTask: Represents a parsed Task containing a sequence of with blocks.
"""
import ast
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from typing import Optional
from typing import TypeVar


@dataclass
class ParsedAction:
    name: str
    args: list[str]
    kwargs: dict[str, str]


@dataclass
class ParsedType:
    name: str
    args: list[str]
    kwargs: dict[str, str]


ParsedWithT = TypeVar("ParsedWith")  # type: ignore


@dataclass
class ParsedWith:
    type: ParsedType
    contains: list[ParsedWithT | ParsedAction]


@dataclass
class ParsedTask:
    name: str
    sequence: list[ParsedWith]


class ExelentParser:

    def __init__(
        self, target_file: Optional[Path | str] = None, target_str: Optional[str] = None
    ) -> None:
        """Initialize the parser with a file or string content."""

        assert not (
            target_file and target_str
        ), "target_file and target_str are mutually exclusive, please select only one argument"

        if target_file:
            with open(file=target_file, encoding="utf-8") as fd:
                self.content = fd.read()

        if target_str:
            self.content = target_str

        self.tree = None

    def parse(self) -> ParsedTask | None:
        """Parse the settled content file/str and return a ParsedTask object."""

        self.tree = ast.parse(self.content)
        tasks = []
        for node in ast.walk(self.tree):
            if isinstance(node, ast.FunctionDef):
                tasks.append(self._parse_task(node))

        return linearize_multiple_defs(tasks)

    @classmethod
    def _parse_action(cls, action: ast.Call) -> ParsedAction | None:
        """
        Parse a function call (Action) into a ParsedAction object.
        Example: my_func(a, b, c=foo)
        """
        name: str = action.func.id

        if not isinstance(action, ast.Call):
            return None

        # Caution: If it starts with upper, it will be changed to lower
        if name[0].isupper():
            name = name.lower()

        args = [cls._parse_name_or_const(val) for val in action.args]
        kwargs = {
            key.arg: cls._parse_name_or_const(key.value) for key in action.keywords
        }
        return ParsedAction(name, args, kwargs)

    @classmethod
    def _parse_type(cls, type: ast.Call) -> ParsedType | None:
        """
        Parse a type call into a ParsedType object.
        Example: Sequential()
        """
        # Example: Sequential(on_fail=True)
        name: str = type.func.id

        if not isinstance(type, ast.Call):
            return None

        # If it starts with lowercase, then it is an action
        if name[0].islower():
            return None

        args = [cls._parse_name_or_const(val) for val in type.args]
        kwargs = {key.arg: cls._parse_name_or_const(key.value) for key in type.keywords}

        return ParsedType(name, args, kwargs)

    @classmethod
    def _parse_name_or_const(cls, input: ast.Constant | ast.Name) -> Any:
        """Parse a name or constant from AST nodes."""
        if isinstance(input, ast.Constant):
            input: ast.Constant
            return input.value
        elif isinstance(input, ast.Name):
            input: ast.Name
            return input.id
        elif isinstance(input, ast.List) or isinstance(input, ast.Tuple):
            input: ast.List
            return [cls._parse_name_or_const(var) for var in input.elts]
        elif isinstance(input, ast.Expr):
            input: ast.Expr
            return cls._parse_name_or_const(input.value)

    @classmethod
    def _parse_with(cls, _with: ast.With) -> ParsedWith | None:
        """
        Parse a with definition into a ParsedWith object.
        Example: with Sequential():
        """

        if not isinstance(_with, ast.With):
            return None

        # Only one item in with supported at the moment
        type_call = _with.items[0].context_expr
        type = cls._parse_type(type_call)
        assert type_call is not None, "Invalid type passed in With block"

        items = []
        for node in _with.body:

            if isinstance(node, ast.Expr):
                node = node.value

            assert isinstance(
                node, ast.Call | ast.With
            ), "Invalid function/line passed inside a With statement:"
            items.append(cls._parse_with(node) or cls._parse_action(node))

        return ParsedWith(type=type, contains=items)

    @classmethod
    def _parse_task(cls, task: ast.FunctionDef) -> ParsedTask | None:
        """
        Parse a function definition into a ParsedTask object.
        Example: def my_task():
        """

        # Note: The arguments not supported yet, will be ignored
        if not isinstance(task, ast.FunctionDef):
            return None

        name = task.name

        assert all([isinstance(node, ast.With) for node in task.body])
        sequence = [cls._parse_with(node) for node in task.body]

        return ParsedTask(name, sequence)


def linearize_multiple_defs(tasks: list[ParsedTask]) -> ParsedTask:
    _withs = []
    for task in tasks:
        _withs.extend(task.sequence)

    result = ParsedTask(name=tasks[0].name, sequence=_withs)
    return result


def linerize_task(task: ParsedTask) -> ParsedTask:
    """
    Linearize a ParsedTask object into a sequence of actions.
    Example:
    with Sequential():
        with Sequential():
            a()
        with Sequential():
            b()
    Result:
    with Sequential():
        a()
        b()
    """
    assert (
        task.sequence is not None
    ), "Given task does not contain any sequences? Check the definition of the task."
    task = deepcopy(task)
    result = []
    for _with in task.sequence:

        if not isinstance(_with, ParsedWith):
            continue

        result.append(_linearize_with(_with))
    task.sequence = result
    return task


def _linearize_with(_with: ParsedWith) -> ParsedWith:
    """Helper function to linearize a ParsedWith object."""

    if all([isinstance(node, ParsedAction) for node in _with.contains]):
        return _with

    actions = []
    for node in _with.contains:
        if isinstance(node, ParsedAction):
            actions.append(node)
            continue

        if not isinstance(node, ParsedWith):
            raise TypeError(f"Expected With type and obtained: {node}. ¿Maybe there is a misspell?")

        node: ParsedWith
        node = _linearize_with(node)
        actions.extend(node.contains)

    _with.contains = actions
    return _with


def parse(
    target_file: Optional[Path | str] = None, target_str: Optional[str] = None
) -> ParsedTask | None:
    """[Recommended] Parse a target file or string and return a ParsedTask object."""
    parser = ExelentParser(target_file=target_file, target_str=target_str)
    return parser.parse()
