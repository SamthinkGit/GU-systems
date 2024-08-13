import ecm.exelent.parser as parser


class ExelentBuilder:

    _lines: list[str]
    _current_indentation: int

    def __init__(self) -> None:
        self._lines = []
        self._current_indentation = 0

    def add_task(self, task_name: str) -> None:
        self._lines.append(f"def {task_name}():")
        self._current_indentation += 1

    def add_type(self, type: str) -> None:
        tabulation = self._current_indentation * "\t"
        if type[0].islower():
            raise ValueError(f"{type} is not a valid type.")

        self._lines.append(f"{tabulation}with {type}():")
        self._current_indentation += 1

    def add_statement(self, statement: str) -> None:
        tabulation = self._current_indentation * "\t"
        self._lines.append(tabulation + statement)

    def close(self):
        if self._current_indentation <= 0:
            raise ValueError(
                "Cannot close at starting point. Please first use add_task()"
            )
        self._current_indentation -= 1

    def flush(self):
        self._lines = []
        self._current_indentation = 0

    def status(self):
        return "\n".join(self._lines)

    def compile(self) -> parser.ParsedTask:
        code = "\n".join(self._lines)
        return parser.parse(target_str=code)
