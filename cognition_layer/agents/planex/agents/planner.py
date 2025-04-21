"""
Planner Module
=================================

This module provides the `Planner` class for creating and executing planning tasks using
the `ChatOpenAI` model. The planner constructs prompts and processes user inputs to
generate responses based on the predefined instructions, guidelines, and examples.

The module is based on the "Planification" Step from this link:
https://github.com/SamthinkGit/GU-systems/wiki/ECM-Problem-Analysis

For using alone you can use the example below, if you want to chain this agent
to other langchain compatibles, you can check the example in /test/sandbox/planex
"""
import json
from typing import Optional

from colorama import Fore
from colorama import Style
from langchain_core.messages import BaseMessage
from langchain_core.prompts import ChatPromptTemplate
from langchain_openai import ChatOpenAI

from cognition_layer.constants import DEFAULT_MODEL
from cognition_layer.planex.agents.prompts import PlanexPrompts
from ecm.shared import get_logger
from ecm.tools.inspector import SystemInspector


class Planner:

    _logger = get_logger("Planner")

    def __init__(
        self, model: str = DEFAULT_MODEL, temperature: float = 0, **kwargs
    ) -> None:
        """Inits connection with OpenAI and loads prompt resources"""
        self.llm = ChatOpenAI(model=model, temperature=temperature, **kwargs)
        sys_message = (
            "\nInstructions: \n"
            + PlanexPrompts.PLANNER_INSTRUCTIONS
            + "\nGuidelines: \n"
            + PlanexPrompts.PLANNER_GUIDELINES
            + "\nExample: \n"
            + PlanexPrompts.PLANNER_EXAMPLE
            + "System Real Time Information: "
            + PlanexPrompts.SYSTEM_INFORMATION
            + "{sys_realtime_info}"
        )

        self.prompt = ChatPromptTemplate.from_messages(
            [("system", sys_message), ("user", "{input}")]
        )
        self.chain = self.prompt | self.llm
        self.sys_inspector = SystemInspector()

    def plan(self, input: str, verbose: Optional[bool] = False) -> BaseMessage:
        """From a given query, returns a plan to solve that query"""
        sys_realtime_info = json.dumps(self.sys_inspector.summary(), indent=2)
        result = self.chain.invoke({"input": input, "sys_realtime_info": sys_realtime_info})
        if verbose:
            Planner._logger.info("Received: \n" + input)
            Planner._logger.info("Generated: \n" + result.content)
        return result


if __name__ == "__main__":
    planner = Planner()
    print(Fore.GREEN + Style.BRIGHT + "[Planner]" + Style.RESET_ALL)
    print(
        Fore.CYAN
        + str(planner.plan("Write 'Hello World' in a terminal.").content)
        + Style.RESET_ALL
    )
