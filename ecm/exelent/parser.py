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

        assert not (
            target_file and target_str
        ), "target_file and target_str are mutually exclusive, please select only one argument"

        if target_file:
            with open(file=target_file, encoding="utf-8") as fd:
                self.content = fd.read()

        if target_str:
            self.content = target_str

    def parse(self) -> ParsedTask | None:

        # Note: Currently multiple tasks are not supported

        self.tree = ast.parse(self.content)
        for node in ast.walk(self.tree):
            if isinstance(node, ast.FunctionDef):
                return self._parse_task(node)

    @classmethod
    def _parse_action(cls, action: ast.Call) -> ParsedAction | None:
        # Example: mi_func(a,b, foo=c)
        name: str = action.func.id

        if not isinstance(action, ast.Call):
            return None

        # Caution: If it starts with upper, then it is a type
        if name[0].isupper():
            return None

        args = [cls._parse_name_or_const(val) for val in action.args]
        kwargs = {
            key.arg: cls._parse_name_or_const(key.value) for key in action.keywords
        }
        return ParsedAction(name, args, kwargs)

    @classmethod
    def _parse_type(cls, type: ast.Call) -> ParsedType | None:
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
        if isinstance(input, ast.Constant):
            input: ast.Constant
            return input.value
        elif isinstance(input, ast.Name):
            input: ast.Name
            return input.id
        elif isinstance(input, ast.Expr):
            input: ast.Expr
            return cls._parse_name_or_const(input.value)

    @classmethod
    def _parse_with(cls, _with: ast.With) -> ParsedWith | None:

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

        # Note: The arguments not supported yet, will be ignored
        if not isinstance(task, ast.FunctionDef):
            return None

        name = task.name

        assert all([isinstance(node, ast.With) for node in task.body])
        sequence = [cls._parse_with(node) for node in task.body]

        return ParsedTask(name, sequence)


def linerize_task(task: ParsedTask) -> ParsedTask:
    assert task.sequence is not None, "Given task does not contain any sequences? Check the definition of the task."
    task = deepcopy(task)
    result = []
    for _with in task.sequence:

        if not isinstance(_with, ParsedWith):
            continue

        result.append(_linearize_with(_with))
    task.sequence = result
    return task


def _linearize_with(_with: ParsedWith) -> ParsedWith:

    if all([isinstance(node, ParsedAction) for node in _with.contains]):
        return _with

    actions = []
    for node in _with.contains:
        if isinstance(node, ParsedAction):
            actions.append(node)
            continue

        node: ParsedWith
        node = _linearize_with(node)
        actions.extend(node.contains)

    _with.contains = actions
    return _with


def parse(
    target_file: Optional[Path | str] = None, target_str: Optional[str] = None
) -> ParsedTask | None:
    parser = ExelentParser(target_file=target_file, target_str=target_str)
    return parser.parse()
