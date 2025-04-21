"""
Xplore
====================
Xplore is a cognition layer agent that tries to generate the set
of actions in order to complete the given goal by using a loop of
subgoal-reaction.

All the information about this agent is explained in the Wiki.

An example of usage of this agent can be found at
    /tests/examples/host/xplore/xplore.py
"""
from typing import Literal
from typing import TypedDict

from langchain.pydantic_v1 import Field
from langgraph.graph import StateGraph

from cognition_layer.xplore.agents.general_planner import GeneralPlannerAgent
from cognition_layer.xplore.agents.general_planner import GeneralPlannerResponse
from cognition_layer.xplore.agents.subgoal_planner import SubgoalPlanner
from cognition_layer.xplore.agents.subgoal_planner import SubgoalResponse
from cognition_layer.xplore.agents.subgoal_reviewer import ReviewerAgent
from cognition_layer.xplore.agents.subgoal_reviewer import SubgoalReviewerResponse
from ecm.exelent.builder import ExelentBuilder
from ecm.mediator.Interpreter import Interpreter
from ecm.shared import get_logger


class XploreState(TypedDict):
    current_node: str = Field(description="The current working node.")

    query: str = Field(description="Original user query")
    problem_description: str = Field(
        description="Description on how the problem can be solved."
    )
    additional_info: str = Field(
        description="Additional Info that can be passed between llm's."
    )
    subgoals: list[str] = Field(
        description="Set of subgoals in order to solve the query."
    )
    functions: list[str] = Field(
        description="A set of pythonic functions that must be executed as next step."
    )
    step: int = Field(description="Current number of tasks sent.")


class Xplore:
    _logger = get_logger("Xplore")

    def __init__(self, interpreter: Interpreter) -> None:

        self.general_planner = GeneralPlannerAgent()
        self.subgoal_planner = SubgoalPlanner()
        self.reviewer_agent = ReviewerAgent()
        self.interpreter = interpreter
        self.graph = self._compile()

    def general_planner_node(self, status: XploreState):

        step = 0 if status.get("step") is None else status["step"]

        if not status["query"] or status["query"] == "":
            raise SystemError(
                "Xplore cannot start without an initial query from the user."
            )

        response: GeneralPlannerResponse = self.general_planner.invoke(
            query=status["query"]
        )
        additional_info = f"""Note that the current status of the computer can be described as:
        ```{response.description}```
        """
        return {
            "current_node": "General Planner",
            "problem_description": response.reasoning,
            "subgoals": response.plan,
            "additional_info": additional_info,
            "step": step,
        }

    def subgoal_planner_node(self, status: XploreState):
        response: SubgoalResponse = self.subgoal_planner.invoke(
            goal=status["query"],
            plan=status["subgoals"],
            additional_info=status["additional_info"],
        )
        return {
            "current_node": "Subgoal Planner",
            "additional_info": response.reasoning,
            "functions": response.steps,
        }

    def interpreter_node(self, status: XploreState):
        builder = ExelentBuilder()
        builder.add_task(task_name=f"xplore_subgoal_step_{status['step']}")
        builder.add_type("Sequential")

        for func in status["functions"]:
            builder.add_statement(func)

        task = builder.compile()
        self.interpreter.run(task=task, callback="silent")

    def review_completed_node(self, status: XploreState):
        response: SubgoalReviewerResponse = self.reviewer_agent.invoke(
            plan=status["subgoals"], additional_info=status["additional_info"]
        )
        subgoals = status["subgoals"]
        for check in response.verification:
            if check:
                try:
                    subgoals.pop(0)
                except IndexError:
                    self._logger.error(
                        "Reviewer returned more 'true' or completed subgoals than "
                        "defined in the status. Skipping."
                    )
        return {
            "step": status["step"] + 1,
            "current_node": "Reviewer Agent",
            "functions": [],
            "subgoals": subgoals,
            "additional_info": "",
        }

    def all_subgoals_completed_edge(
        self, status: XploreState
    ) -> Literal["__end__", "subgoal_planner"]:
        if len(status["subgoals"]) <= 0:
            self._logger.debug("All subgoals have been completed, exit.")
            return "__end__"
        return "subgoal_planner"

    def _compile(self):

        graph_builder = StateGraph(XploreState)
        graph_builder.add_node("general_planner", self.general_planner_node)
        graph_builder.add_node("subgoal_planner", self.subgoal_planner_node)
        graph_builder.add_node("interpreter", self.interpreter_node)
        graph_builder.add_node("review_completed", self.review_completed_node)

        graph_builder.set_entry_point("general_planner")
        graph_builder.add_edge("general_planner", "subgoal_planner")
        graph_builder.add_edge("subgoal_planner", "interpreter")
        graph_builder.add_edge("interpreter", "review_completed")
        graph_builder.add_conditional_edges(
            "review_completed", self.all_subgoals_completed_edge
        )

        return graph_builder.compile()
