"""
Xplore: Subgoal Planner
========================
This agent is responsible for generating an action or set of actions
in order to satisfy a subgoal.

Note: All the actions will be loaded from the ItemRegistry.
"""
from langchain.pydantic_v1 import BaseModel
from langchain.pydantic_v1 import Field
from langchain.tools import tool as build_tool
from langchain_core.messages import SystemMessage
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.prompts import MessagesPlaceholder
from langchain_core.tools import StructuredTool
from langchain_openai import ChatOpenAI

from action_space.tools.image import ImageMessage
from action_space.tools.image import load_image
from cognition_layer.constants import DEFAULT_IMAGE_MODEL
from cognition_layer.planex.utils.format import format_tool
from ecm.shared import get_logger
from ecm.tools.registry import ItemRegistry


class SubgoalResponse(BaseModel):

    reasoning: str = Field(
        description="A reasoning about what the user has requested and how that subgoal could be reached."
    )
    steps: list[str] = Field(
        description="The immediate set of pythonic functions (and arguments) that must be executed right now. Try "
        "to include multiple functions that can be executed without further analysis. Also ensure to wait for "
        "actions to complete if needed.",
        max_items=3,
    )


class SubgoalPlanner:

    _logger = get_logger("SubGoal Planner")

    def __init__(self) -> None:
        llm = ChatOpenAI(model=DEFAULT_IMAGE_MODEL).with_structured_output(
            SubgoalResponse
        )
        sys_message = """
        You are the First-Subgoal Planning Assistant. Your role is to create a function-based, step-by-step
        plan that effectively addresses the user's first subgoal, guiding them toward their final
        goal. Please adhere to the following guidelines:

        1. Generate plans exclusively using the functions provided previously in the system prompt.
        2. Structure any functions with arguments in a clear and Pythonic format (when arguments are necessary).
        3. Carefully review the original plan provided to fully grasp the intended outcome.
        4. Ensure that the new plan exclusively utilizes the provided functions and successfully replicates
        the results of the original plan.
        5. Ensure to only archieve the first subgoal.
        6. Use the image provided from the user as information of the current status of the user.
        """
        functions = "\n".join(self._load_functions())
        sys_tools = f"""
        The available functions for solving all subgoals are:
        {functions}
        """

        prompt = ChatPromptTemplate(
            [
                SystemMessage(content=sys_message),
                SystemMessage(content=sys_tools),
                MessagesPlaceholder("query"),
            ]
        )

        self.chain = prompt | llm

    def _load_functions(self):
        tools: list[StructuredTool] = [
            build_tool(t) for t in ItemRegistry._functions.values()
        ]
        return [format_tool(t) for t in tools]

    def invoke(
        self, goal: str, plan: list[str], additional_info: str
    ) -> SubgoalResponse:
        screenshot = load_image(ItemRegistry._utils["screenshot"]())
        plan_str = "\n".join([f"{idx+1}: {step}" for idx, step in enumerate(plan)])
        prompt = f"""
        I want to solve the following task:
        ```{goal}```
        For archieving it, this is the plan that I propose:
        ```
        {plan_str}
        ```
        {additional_info}

        How can i solve the first subgoal ```{plan[0]}``` by using the provided functions?
        """

        response = self.chain.invoke(
            {
                "query": [
                    ImageMessage(
                        image=screenshot,
                        input=prompt,
                    ).as_human()
                ]
            }
        )
        return response
