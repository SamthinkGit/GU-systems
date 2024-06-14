from typing import Optional

from colorama import Fore
from colorama import Style
from langchain.prompts import ChatPromptTemplate
from langchain.tools import StructuredTool
from langchain.tools import tool as build_tool
from langchain_core.messages import BaseMessage
from langchain_openai import ChatOpenAI

from cognition_layer.planex.agents.prompts import PlanexPrompts
from cognition_layer.planex.constants import DEFAULT_MODEL
from cognition_layer.planex.utils.format import format_tool
from ecm.tools.registry import ItemRegistry


class Reducer:

    def __init__(
        self, model: str = DEFAULT_MODEL, temperature: float = 0, **kwargs
    ) -> None:
        self.llm = ChatOpenAI(model=model, temperature=temperature, **kwargs)
        sys_message = (
            "\nInstructions: \n"
            + PlanexPrompts.REDUCER_INSTRUCTIONS
            + "\nGuidelines: \n"
            + PlanexPrompts.REDUCER_GUIDELINES
            + "\nFunctions:\n{actions}\n"
            + "\nExample:\n"
            + PlanexPrompts.REDUCER_EXAMPLE
        )

        self.prompt = ChatPromptTemplate.from_messages(
            [("system", sys_message), ("user", "{input}")]
        )
        self.chain = self.prompt | self.llm
        self.auto_bind_actions()

    def auto_bind_actions(self):
        tools: list[StructuredTool] = [
            build_tool(t) for t in ItemRegistry._functions.values()
        ]
        tools_formatted = "\n".join([format_tool(t) for t in tools])
        self.actions = tools_formatted

    def reduce(self, input: str, actions: Optional[str] = None) -> BaseMessage:

        if actions is None:
            actions = self.actions

        return self.chain.invoke({"actions": actions, "input": input})


if __name__ == "__main__":

    @ItemRegistry.register_function
    def click(place: str):
        """Clicks on the specified place"""
        ...

    @ItemRegistry.register_function
    def write(text: str):
        """Types the passed text."""
        ...

    plan = """
    0. Click on the search bar located at the bottom left corner of the screen.
    1. Type "cmd" and press Enter to open the Command Prompt terminal.
    2. Type "echo Hello World" and press Enter to display "Hello World" in the terminal.
    """
    reducer = Reducer()
    print(Fore.GREEN + Style.BRIGHT + "[Reducer]" + Style.RESET_ALL)
    print(Fore.CYAN + str(msg := reducer.reduce(plan).content) + Style.RESET_ALL)
