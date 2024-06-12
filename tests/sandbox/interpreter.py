import ecm.exelent.parser as parser
from ecm.mediator.rosa_interpreter import RosaInterpreter
from ecm.registry import ItemRegistry
from ecm.shared import get_root_path


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
