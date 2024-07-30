from operator import add
from typing import Annotated
from typing import Callable
from typing import Optional
from typing import TypedDict

from langchain.prompts import ChatPromptTemplate
from langchain.pydantic_v1 import BaseModel
from langchain.pydantic_v1 import Field
from langchain_core.runnables import Runnable
from langchain_openai import ChatOpenAI
from langgraph.graph import StateGraph
from langgraph.graph.graph import CompiledGraph

from cognition_layer.constants import DEFAULT_MODEL


class TryResponse(BaseModel):
    learning_retrieval: str = Field(
        description="A summary of the knowledge that you have obtained previously and should be considered in"
        "order to successfully complete the task."
    )
    reasoning: str = Field(
        description="A detailed step-by-step causal reasoning explaining which action should be taken and why "
        "it is appropriate. This should include an analysis of the potential effects and expected outcomes of "
        "the proposed action."
    )
    action: str = Field(
        description="The specific action to do in order to satisfy the user query."
    )
    expectation: str = Field(
        description="The expected effect and behavior due to the specified action."
    )


class ComparatorResponse(BaseModel):
    reasoning: str = Field(
        description="A detailed step-by-step reasoning on the main points that should be considered in order "
        "to define the match as true or false."
    )
    match: bool = Field(
        description="True if the expected outcome matches the effect, false otherwise."
    )


class LearnerResponse(BaseModel):
    failure: str = Field(
        description="A detailed step-by-step reasoning of the main errors or failures between the expected environment "
        "and the obtained effect. Focus on explaining the causality of the error and the failures in the original "
        "reasoning."
    )
    learnings: str = Field(
        description="A set of actionable advice and learnings on what can be improved in future actions. Include "
        "important considerations to take into account before defining the correct answer."
    )


class SpecializationState(TypedDict):
    query: str
    state: str
    actions: Annotated[list, add]
    effects: Annotated[list, add]
    expectation: str
    reasoning: str
    learnings: Annotated[list, add]
    failures: Annotated[list, add]
    difference: str
    match: bool


class SpecializationGraph:

    def __init__(
        self,
        action_prompt: str,
        test: Callable[[str], None],
        effect_descriptor: Callable[[], str],
        reset_state: Callable[[], None],
        model: Optional[str] = None,
    ) -> None:

        self.test = test
        self.effect_descriptor = effect_descriptor
        self.reset_state = reset_state
        if model is None:
            model = DEFAULT_MODEL

        llm = ChatOpenAI(temperature=0, model=model)
        action_prompt = ChatPromptTemplate(
            [
                ("system", action_prompt),
                ("ai", "{state}"),
                ("ai", "{previous_actions}"),
                ("ai", "{learnings}"),
                ("ai", "{failures}"),
                ("user", "{query}"),
            ]
        )
        self.try_chain: Runnable = action_prompt | llm.with_structured_output(
            TryResponse
        )
        comparator_prompt = ChatPromptTemplate(
            [
                (
                    "system",
                    "You are a Comparator Agent. Your goal is to determine if a given query has been executed "
                    "successfully as predicted in the expectation. For this task, you will first read user's "
                    "environment description, analyze the expected outcome and behavior, and then return true "
                    "if the actual effect matches the expected effect. Otherwise, return false.",
                ),
                (
                    "user",
                    "The original status of the environment can be described as: `{original_state}`",
                ),
                (
                    "user",
                    "The expected outcome of my action must be `{expected_effect}`",
                ),
                ("user", "The generated effect of my action is: `{effect}`"),
            ]
        )
        self.comparator_chain: Runnable = (
            comparator_prompt | llm.with_structured_output(ComparatorResponse)
        )

        learner_prompt = ChatPromptTemplate(
            [
                (
                    "system",
                    "You are an AI tutor/trainer. Your goal is to determine the"
                    "cause of the error in a user's generated action. To complete"
                    "this task, first read the user's main query, the reasoning"
                    "for achieving a subgoal, and the expected outcome. Then,"
                    "examine the actual effect of the action and identify the"
                    "differences between the expected and actual environments."
                    "Next, provide a step-by-step reasoning on why the action"
                    "failed, highlight any incorrect assumptions, and identify"
                    "invalid or inappropriate actions for the scenario. Finally,"
                    "provide a revised step-by-step learnings and advises that"
                    "leads to the correct action in future plannings, learning "
                    "from the original errors."
                ),
                (
                    "user",
                    "The original status of the environment can be described as: `{original_state}`",
                ),
                ("user", "The action used by the user is `{action}`"),
                ("user", "The reasoning for doing that action is `{reasoning}`"),
                (
                    "user",
                    "The expected outcome of my action must be `{expected_effect}`",
                ),
                ("user", "The generated effect of my action is: `{effect}`"),
                (
                    "user",
                    "The difference between those two scenarios is: `{difference}`",
                ),
            ]
        )
        self.learner_chain: Runnable = learner_prompt | llm.with_structured_output(
            LearnerResponse
        )

    def try_node(self, state: SpecializationState):
        previous_actions = [
            f"Previously tried with: `{action}`: {effect}."
            for action, effect in zip(state["actions"], state["effects"])
        ]
        learnings = [
            f"{idx}. {learning}" for idx, learning in enumerate(state["learnings"])
        ]
        failures = [
            f"{idx}. {failure}" for idx, failure in enumerate(state["failures"])
        ]

        descriptor_response = self.effect_descriptor()
        previous_actions = "\n".join(previous_actions)
        response: TryResponse = self.try_chain.invoke(
            {
                "query": state["query"],
                "state": f"The current state of the environment is {descriptor_response}",
                "previous_actions": (
                    "Some previous (and failed) trials for completing this task "
                    f"are: `{previous_actions}`"
                ),
                "learnings": "Knowledge previously obtained:\n" + "\n".join(learnings),
                "failures": "Failures previously reached:\n" + "\n".join(failures),
            }
        )

        return {
            "actions": [response.action],
            "expectation": [response.expectation],
            "reasoning": [response.reasoning],
            "state": descriptor_response,
        }

    def test_node(self, state: SpecializationState):
        self.test(state["actions"][-1])
        effect = self.effect_descriptor()
        comparator_response: ComparatorResponse = self.comparator_chain.invoke(
            {
                "original_state": state["state"],
                "expected_effect": state["expectation"],
                "effect": effect,
            }
        )

        return {
            "effects": [effect],
            "match": comparator_response.match,
            "difference": comparator_response.reasoning,
        }

    def should_retry_edge(self, state: SpecializationState):
        if state["match"]:
            return "__end__"
        return "learn"

    def learn_node(self, state: SpecializationState):
        learner_response: LearnerResponse = self.learner_chain.invoke(
            {
                "action": state["actions"][-1],
                "reasoning": state["reasoning"],
                "expected_effect": state["expectation"],
                "effect": state["effects"][-1],
                "original_state": state["state"],
                "difference": state["difference"],
            }
        )
        self.reset_state()

        return {
            "learnings": [learner_response.learnings],
            "failures": [learner_response.failure],
        }

    def compile(self) -> CompiledGraph:
        graph_builder = StateGraph(SpecializationState)
        graph_builder.add_node("try", self.try_node)
        graph_builder.add_node("test", self.test_node)
        graph_builder.add_node("learn", self.learn_node)
        graph_builder.set_entry_point("try")

        graph_builder.add_edge("try", "test")
        graph_builder.add_conditional_edges("test", self.should_retry_edge)
        graph_builder.add_edge("learn", "try")
        return graph_builder.compile()


if __name__ == "__main__":
    from colorama import Fore
    from langchain_core.prompts import MessagesPlaceholder
    from langchain_core.messages import AIMessage, HumanMessage
    import os
    import random
    import json

    os.environ["LANGCHAIN_PROJECT"] = "AGAIN Specialization Graph"

    llm = ChatOpenAI()
    history = []
    fake_env_prompt = ChatPromptTemplate(
        [
            (
                "system",
                "From now on you will emulate you are a windows 11 if the user asks you a question "
                "you will briefly describe the environment. Else, execute the actions the user tells (even if "
                "it fails just execute emulating the environment as realistic as you can, includin substeps needed.)",
            ),
            ("system", "Remember: VSCode is not installed at the moment."),
            MessagesPlaceholder("history"),
            ("user", "{query}"),
        ]
    )
    fake_env = fake_env_prompt | llm

    def test(query):

        response = fake_env.invoke({"query": query, "history": history}).content
        history.extend([HumanMessage(content=query), AIMessage(content=response)])
        return response

    def effect_descriptor():
        query = "Do a small description (1 line) of the status of the environment"
        response = fake_env.invoke({"query": query, "history": history}).content
        return response

    def reset():
        global history
        history = []

    graph = SpecializationGraph(
        action_prompt=(
            "You are an AI computer expert. The user will ask you for a task, and you must "
            "complete it doing action by action."
        ),
        test=test,
        effect_descriptor=effect_descriptor,  # -> str
        reset_state=reset,
    ).compile()

    user_input = "I want to open VS code."
    config = {"configurable": {"thread_id": str(random.randint(5000, 15000))}}

    for event in graph.stream({"query": user_input}, config, stream_mode="values"):
        print("=" * 30)
        print(Fore.YELLOW + json.dumps(event, indent=4) + Fore.RESET)
