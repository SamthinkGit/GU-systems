from colorama import Fore
from colorama import Style
from langchain.globals import set_debug
from langchain.globals import set_verbose

from cognition_layer.planex.agents.planner import Planner
from cognition_layer.planex.agents.reducer import Reducer
from ecm.tools.registry import ItemRegistry

if __name__ == "__main__":

    set_verbose(True)
    set_debug(True)

    @ItemRegistry.register_function
    def click(place: str):
        """Clicks on the specified place"""
        ...

    @ItemRegistry.register_function
    def type(text: str):
        """Types the passed text."""
        ...

    planner = Planner()
    reducer = Reducer()
    print(reducer.actions)

    planex = (
        planner.chain
        | (lambda output: {"input": output.content, "actions": reducer.actions})
        | reducer.chain
    )
    result = planex.invoke({"input": "How can I open spotify?"})

    print(Fore.YELLOW + Style.BRIGHT + "Result: " + result.content + Style.RESET_ALL)
