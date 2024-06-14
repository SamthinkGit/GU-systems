from colorama import Fore
from colorama import Style
from langchain.tools import StructuredTool
from langchain.tools import tool as build_tool

from cognition_layer.planex.agents.reducer import Reducer
from cognition_layer.planex.utils.format import format_tool
from ecm.tools.registry import ItemRegistry


@ItemRegistry.add_function
def click(place: str, button: str) -> str:
    """Clicks at the settled place, with the specified button."""
    return "Click!"


if __name__ == "__main__":

    registry = ItemRegistry()
    reducer = Reducer()

    tools: list[StructuredTool] = [build_tool(t) for t in registry._functions.values()]
    tools_formatted = '\n'.join([format_tool(t) for t in tools])
    plan = """
    0. Click on the search bar located at the bottom left corner of the screen.
    1. Type "cmd" and press Enter to open the Command Prompt terminal.
    2. Type "echo Hello World" and press Enter to display "Hello World" in the terminal.
    """
    reducer = Reducer()

    print(Fore.GREEN + Style.BRIGHT + "[Reducer]" + Style.RESET_ALL)
    print(
        Fore.CYAN
        + str(reducer.reduce(plan, actions=tools_formatted).content)
        + Style.RESET_ALL
    )
