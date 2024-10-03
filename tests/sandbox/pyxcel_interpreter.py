from ecm.exelent.parser import parse
from ecm.shared import get_root_path
from execution_layer.pyxcel.interpreter.pyxcel_interpreter import PyxcelInterpreter

if __name__ == "__main__":
    pyxcel = PyxcelInterpreter()
    path = get_root_path() / "tests" / "resources" / "open_http_server.xlnt"
    task = parse(path)
    pyxcel.run(task)
