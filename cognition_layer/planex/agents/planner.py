from colorama import Fore
from colorama import Style
from langchain_core.messages import BaseMessage
from langchain_core.prompts import ChatPromptTemplate
from langchain_openai import ChatOpenAI

from cognition_layer.planex.agents.prompts import PlanexPrompts
from cognition_layer.planex.constants import DEFAULT_MODEL


class Planner:

    def __init__(
        self, model: str = DEFAULT_MODEL, temperature: float = 0, **kwargs
    ) -> None:
        self.llm = ChatOpenAI(model=model, temperature=temperature, **kwargs)
        sys_message = (
            "\nInstructions: \n"
            + PlanexPrompts.PLANNER_INSTRUCTIONS
            + "\nGuidelines: \n"
            + PlanexPrompts.PLANNER_GUIDELINES
            + "\nExample: \n"
            + PlanexPrompts.PLANNER_EXAMPLE
        )

        self.prompt = ChatPromptTemplate.from_messages(
            [("system", sys_message), ("user", "{input}")]
        )
        self.chain = self.prompt | self.llm

    def plan(self, input: str) -> BaseMessage:
        return self.chain.invoke({"input": input})


if __name__ == "__main__":
    planner = Planner()
    print(Fore.GREEN + Style.BRIGHT + "[Planner]" + Style.RESET_ALL)
    print(Fore.CYAN + str(planner.plan("Write 'Hello World' in a terminal.").content) + Style.RESET_ALL)
