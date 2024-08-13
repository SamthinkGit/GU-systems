from langchain.pydantic_v1 import BaseModel
from langchain.pydantic_v1 import Field
from langchain_core.messages import SystemMessage
from langchain_core.prompts import ChatPromptTemplate
from langchain_core.prompts import MessagesPlaceholder
from langchain_openai import ChatOpenAI

import action_space.experimental.screenshot.actions  # noqa
from action_space.tools.image import ImageMessage
from action_space.tools.image import load_image
from cognition_layer.constants import DEFAULT_IMAGE_MODEL
from ecm.tools.registry import ItemRegistry

# ----- Required Utils ------


class SubgoalReviewerResponse(BaseModel):

    reasoning: str = Field(
        description="A reasoning about the user status and which subgoals have been reached."
    )
    verification: list[bool] = Field(
        description=(
            "A set of booleans for which goals have been completed ordered sequentially. Return "
            "for each subgoal: true if completed, else false."
        )
    )


class ReviewerAgent:

    def __init__(self) -> None:
        llm = ChatOpenAI(model=DEFAULT_IMAGE_MODEL).with_structured_output(
            SubgoalReviewerResponse
        )
        sys_message = """
        You are an Expert Reviewer Assistant. Your task is to provide a list of step-by-step
        reviews to find which subgoals have been completed.
        Note that you must follow these rules:
        1. Only focus on the current status of the user and that have been already reached and should be discarded.
        2. A subgoal cannot be completed if its previous requirements have not been reached (use deduction if needed).
        3. Use the image provided from the user to determine the current status of the user.
        """

        prompt = ChatPromptTemplate(
            [SystemMessage(content=sys_message), MessagesPlaceholder("query")]
        )

        self.chain = prompt | llm

    def invoke(self, plan: list[str], additional_info: str) -> SubgoalReviewerResponse:
        screenshot = load_image(ItemRegistry._utils["screenshot"]())
        prompt = f"""
        I want to reach the following set of subgoals:
        ```{plan}```

        {additional_info}

        Which of these subgoals have been already completed or reached (if any)?
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
