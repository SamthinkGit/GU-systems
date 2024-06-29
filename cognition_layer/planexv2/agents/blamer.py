import json
from typing import NamedTuple

from langchain.prompts import ChatPromptTemplate
from langchain.pydantic_v1 import BaseModel
from langchain.pydantic_v1 import Field
from langchain_openai import ChatOpenAI

from cognition_layer.constants import DEFAULT_MODEL
from cognition_layer.planexv2.agents.prompts import PlanexV2Prompts
from ecm.shared import get_logger


class ClassificationResponse(BaseModel):

    blame: str = Field(description="the exact name of the blamed agent.")
    explanation: str = Field(
        description="An explanation of why the agent has been blamed."
    )
    advise: str = Field(
        description="An advise for the blamed agent in order to avoid this exception. Be specific with the error."
    )


class BlameInfo(NamedTuple):
    query: str
    reducer_tools: str
    planner_output: str
    reducer_output: str
    translator_output: str
    additional_info: str


class Blamer:

    _logger = get_logger("Blamer")

    def __init__(
        self, model: str = DEFAULT_MODEL, temperature: float = 0, **kwargs
    ) -> None:
        """Inits connection with OpenAI and loads prompt resources"""
        self.llm = ChatOpenAI(
            model=model, temperature=temperature, **kwargs
        ).with_structured_output(ClassificationResponse)
        self.prompt = ChatPromptTemplate.from_messages(
            [
                ("system", f"Instructions:```{PlanexV2Prompts.BLAMER_INSTRUCTIONS}```"),
                ("system", f"Guidelines:```{PlanexV2Prompts.BLAMER_GUIDELINES}```"),
                ("system", f"EXAMPLE:```{PlanexV2Prompts.BLAMER_EXAMPLE}```"),
                ("user", "Query:```{query}```"),
                ("user", "Planner:```{planner_output}```"),
                ("user", "Tools:```{reducer_tools}```"),
                ("user", "Reducer:```{reducer_output}```"),
                ("user", "Translator:```{reducer_output}```"),
                ("user", "Additional_info:```{additional_info}```"),
            ]
        )
        self.chain = self.prompt | self.llm

    def run(self, blame_info: BlameInfo, verbose: bool = False) -> ClassificationResponse:
        """From a given information of an error (Planex) it selects the most probable failed
        agent and generates and explanation of the problem"""

        result: ClassificationResponse = self.chain.invoke(
            {
                "query": blame_info.query,
                "planner_output": blame_info.planner_output,
                "reducer_output": blame_info.reducer_output,
                "translator_output": blame_info.translator_output,
                "reducer_tools": blame_info.reducer_tools,
                "additional_info": blame_info.additional_info,
            }
        )
        if verbose:
            self._logger.info(
                "Received: \n" + json.dumps(blame_info._asdict(), indent=4)
            )
            self._logger.info("Generated: \n" + json.dumps(result.dict(), indent=4))
        return result
