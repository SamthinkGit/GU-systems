"""
Translator Module
=================================

This module provides the `Translator` class for converting task plans into executable actions using
the `ChatOpenAI` model. The translator changes a given reduced plan into an Exelent task.

The module is based on the "Translation" Step from this link:
https://github.com/SamthinkGit/GU-systems/wiki/ECM-Problem-Analysis

For standalone use, refer to the example below. To integrate this agent with other LangChain-compatible
tools, check the example in /test/sandbox/planex.
"""
from typing import Optional

from colorama import Fore
from colorama import Style
from langchain_core.messages import BaseMessage
from langchain_core.prompts import ChatPromptTemplate
from langchain_openai import ChatOpenAI

from cognition_layer.constants import DEFAULT_MODEL
from cognition_layer.planex.agents.prompts import PlanexPrompts
from ecm.shared import get_logger


class Translator:

    _logger = get_logger("Translator")

    def __init__(
        self, model: str = DEFAULT_MODEL, temperature: float = 0, **kwargs
    ) -> None:
        """Inits connection with OpenAI and loads prompt resources"""

        self.llm = ChatOpenAI(model=model, temperature=temperature, **kwargs)
        # TODO: Add a structure for avoiding the type hard-coding
        exelent_supported_types = [
            (
                "Sequential(): Executes the following actions in order. If "
                "an action fails, it exits and returns an error."
            )
        ]

        sys_message = (
            "\nGuidelines: \n"
            + PlanexPrompts.TRANSLATOR_GUIDELINES
            + "\nExelent: \n"
            + PlanexPrompts.EXELENT_DESCRIPTION
            + "\nSupported Types: \n"
            + "\n".join(exelent_supported_types)
            + "\nExample: \n"
            + PlanexPrompts.TRANSLATOR_EXAMPLE
            + "\nInstructions: \n"
            + PlanexPrompts.TRANSLATOR_INSTRUCTIONS
        )

        self.prompt = ChatPromptTemplate.from_messages(
            [("system", sys_message), ("user", "{input}")]
        )
        self.chain = self.prompt | self.llm

    def translate(self, input: str, verbose: Optional[bool] = False) -> BaseMessage:
        """Translates a plan into a Exlent valid task"""
        result = self.chain.invoke({"input": input})

        if verbose:
            Translator._logger.info("Received: \n" + input)
            Translator._logger.info("Generated: \n" + result.content)

        return result


if __name__ == "__main__":
    translator = Translator()
    print(Fore.GREEN + Style.BRIGHT + "[Translator]" + Style.RESET_ALL)
    print(
        Fore.CYAN
        + str(
            translator.translate(
                "1. Use type('Ctrl+C') to exit.\n2. write('Hello World') in the notepad window"
            ).content
        )
        + Style.RESET_ALL
    )
