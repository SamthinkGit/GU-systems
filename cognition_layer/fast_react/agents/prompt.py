# flake8: noqa
from typing import TypedDict

from langchain_core.output_parsers import JsonOutputParser
from pydantic import BaseModel
from pydantic import Field


class DictFastReactResponse(TypedDict):
    reasoning: str
    function: str
    all_tasks_completed: bool


class JsonFastReactResponse(BaseModel):
    previous_actions: str = Field(description="A small summary of previous AI actions (Look into functions used in AI messages)")
    reasoning: str = Field(description="A reasoning for solving the task")
    function: str = Field(
        description="The function with pythonic notation. E.g: myfunc(2, 3, 'foo')"
    )
    all_tasks_completed: bool = Field(
        description="(true/false) [with lowercase] true if and only if all tasks from the user have been completed"
    )


FastReactParser = JsonOutputParser(pydantic_object=JsonFastReactResponse)

FR_PROMPT = f"""
You are a ReAct agent. On each call you must observe the history and then generate the appropriate reasoning + action in json format.

The response should use the following json template:
{FastReactParser.get_format_instructions()}

Ensure the arguments use pythonic notation.
The system will call you with the observation from your action.
"""
