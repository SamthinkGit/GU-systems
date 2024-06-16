from colorama import Fore
from colorama import Style
from langchain.globals import set_debug
from langchain.globals import set_verbose

import ecm.exelent.parser as parser
from cognition_layer.planex.agents.planner import Planner
from cognition_layer.planex.agents.reducer import Reducer
from cognition_layer.planex.agents.translator import Translator
from cognition_layer.planex.utils.format import extract_python_code
from ecm.mediator.rosa_interpreter import RosaInterpreter
from ecm.tools.registry import ItemRegistry

if __name__ == "__main__":

    set_verbose(False)
    set_debug(True)

    query = input("Select Task (Press ENTER for deafault): ")
    if query == "":
        query = "How can I open spotify?"

    @ItemRegistry.register_function
    def click(place: str) -> None:
        """Clicks on the specified place"""
        print("Clicked on ", place)

    @ItemRegistry.register_function
    def type(text: str) -> None:
        """Types the passed text."""
        print("Typing: ", text)

    @ItemRegistry.register_function
    def press(key: str) -> None:
        """Presses the passed key."""
        print("Typing: ", key)

    planner = Planner()
    reducer = Reducer()
    translator = Translator()

    planex = (
        planner.chain
        | (lambda output: {"input": output.content, "actions": reducer.actions})
        | reducer.chain
        | (lambda output: {"input": output.content})
        | translator.chain
    )
    print(Fore.GREEN + Style.BRIGHT + "[Planning...]" + Style.RESET_ALL)
    result = planex.invoke({"input": query})

    print(Fore.YELLOW + Style.BRIGHT + "Result:\n" + result.content + Style.RESET_ALL)
    print(Fore.GREEN + Style.BRIGHT + "[Executing...]" + Style.RESET_ALL)
    plan = result.content
    if plan.startswith("```python"):
        plan = extract_python_code(plan)

    task = parser.parse(target_str=plan)
    interpreter = RosaInterpreter()
    for pkg in interpreter._generate_packages_from_parsed_task(task):
        print(pkg.to_json())
    interpreter.run(task, callback="silent")
    interpreter.rosa.kill()
