"""
Xplore: General Planner
========================
This agent is responsible for generating a general plan to satisfy a query. This
plan will be generated as a set of strings (plan) with a description of the
status of the system and a reasoning on how the plan could be solved.
"""
import action_space.experimental.screenshot.actions  # noqa
from action_space.tools.image import ImageMessage
from action_space.tools.image import load_image
from cognition_layer.constants import DEFAULT_IMAGE_MODEL
from ecm.tools.registry import ItemRegistry

from langchain.pydantic_v1 import BaseModel
from langchain.pydantic_v1 import Field
from langchain_core.messages import SystemMessage
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.prompts import MessagesPlaceholder
from langchain_openai import ChatOpenAI


class GeneralPlannerResponse(BaseModel):

    description: str = Field(
        description="A description of the current status of the system. Provide all relevant details such as "
        "already open windows, relevant options, operative system, etc."
    )
    reasoning: str = Field(
        description="A reasoning about what the user has requested and how that goal could be reached."
    )
    plan: list[str] = Field(
        description="A summarized, step by step set of sub-goals in order to complete the user request."
    )


class GeneralPlannerAgent:

    def __init__(self) -> None:
        llm = ChatOpenAI(model=DEFAULT_IMAGE_MODEL).with_structured_output(
            GeneralPlannerResponse
        )
        sys_message = """
        You are an Expert Planning Assistant. Your task is to provide a subgoal-based
        step-by-step plan to address the user's query.
        Note that you must follow these rules:
        1. You have full control over the user's computer/system, and any action is available.
        2. Only generate one plan for the given query
        3. If necessary specify the window which must be selected to do the action
        """
        prompt = ChatPromptTemplate(
            [SystemMessage(content=sys_message), MessagesPlaceholder("query")]
        )

        self.chain = prompt | llm

    def invoke(self, query: str) -> GeneralPlannerResponse:
        screenshot = load_image(ItemRegistry._utils["screenshot"]())
        response = self.chain.invoke(
            {
                "query": [
                    ImageMessage(
                        image=screenshot,
                        input=f"The user wants to complete the following task: ```{query}```",
                    ).as_human()
                ]
            }
        )
        return response
