import ecm.exelent.parser as parser
from ecm.shared import get_root_path
from ecm.tools.registry import ItemRegistry
from execution_layer.rosa.interpreter.rosa_interpreter import RosaInterpreter


@ItemRegistry.register_function
def click(location: str):
    print(f"Clicked {location}!")


@ItemRegistry.register_function
def write(text: str):
    print(text)


if __name__ == "__main__":
    path = get_root_path() / "tests" / "resources" / "hello_world.xlnt"
    task = parser.parse(path)
    interpreter = RosaInterpreter()
#    for pkg in interpreter._generate_packages_from_parsed_task(task):
#        print(pkg.to_json())
    interpreter.run(task, callback="silent")
    interpreter.rosa.kill()
