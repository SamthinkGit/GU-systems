from ecm.exelent.parser import ParsedTask


class Executor:

    def __init__(self, interpreter: str) -> None:
        self.interpreter = interpreter

    def run(task: ParsedTask) -> None: ...
