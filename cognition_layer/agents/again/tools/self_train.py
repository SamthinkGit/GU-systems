from operator import add
from typing import Annotated
from typing import Any
from typing import Callable
from typing import Literal
from typing import TypedDict

from langchain.pydantic_v1 import BaseModel
from langchain.pydantic_v1 import Field
from langgraph.graph import END
from langgraph.graph import START
from langgraph.graph import StateGraph
from langgraph.graph.graph import CompiledGraph


class LearningResponse(BaseModel):
    failure: str = Field(
        description="A detailed step-by-step reasoning of the main errors or failures between the expected environment "
        "and the obtained effect. Focus on explaining the causality of the error and the failures in the original "
        "reasoning."
    )
    learnings: str = Field(
        description="A set of actionable advice and learnings on what can be improved in future actions. Include "
        "important considerations to take into account before defining the correct answer."
    )


class TrainingAgentInput(TypedDict):
    query: str
    failures: list[str]
    learnings: list[str]


class SelfTrainStatus(TypedDict):
    query: str
    response: Any
    execution_response: Any
    failures: Annotated[list, add]
    learnings: Annotated[list, add]
    current_iterations: int = 0
    success: bool = False


class SelfTrainGraph:

    def __init__(
        self,
        agent: Callable[[TrainingAgentInput], Any],
        response_executor: Callable[[Any], Any],
        test_approval: Callable[[Any], bool],
        learner: Callable[[SelfTrainStatus], LearningResponse],
        max_iterations: int = 6,
    ) -> None:
        self.agent = agent
        self.response_executor = response_executor
        self.test_approval = test_approval
        self.learner = learner
        self.max_iterations = max_iterations

    def try_node(self, status: SelfTrainStatus):

        response = self.agent(
            TrainingAgentInput(
                query=status["query"],
                failures=status["failures"],
                learnings=status["learnings"],
            )
        )
        return {"response": response}

    def execute_node(self, status: SelfTrainStatus):
        execution_response = self.response_executor(status["response"])
        return {
            "execution_response": execution_response,
            "current_iterations": status["current_iterations"] + 1,
        }

    def training_completed(
        self, status: SelfTrainStatus
    ) -> Literal["learn", "completed", "failure"]:
        if self.test_approval(status["execution_response"]):
            return "completed"
        if status["current_iterations"] >= self.max_iterations:
            return "failure"
        return "learn"

    def learn_node(self, status: SelfTrainStatus):
        response = self.learner(status)
        return {"failures": [response.failure], "learnings": [response.learnings]}

    def failure_node(self, status: SelfTrainStatus):
        return {"success": False}

    def success_node(self, status: SelfTrainStatus):
        return {"success": True}

    def compile(self) -> CompiledGraph:
        graph_builder = StateGraph(SelfTrainStatus)
        graph_builder.add_node("try", self.try_node)
        graph_builder.add_node("execute", self.execute_node)
        graph_builder.add_node("learn", self.learn_node)
        graph_builder.add_node("failure", self.failure_node)
        graph_builder.add_node("completed", self.success_node)

        graph_builder.add_edge(START, "try")
        graph_builder.add_edge("try", "execute")
        graph_builder.add_conditional_edges("execute", self.training_completed)
        graph_builder.add_edge("failure", END)
        graph_builder.add_edge("completed", END)
        graph_builder.add_edge("learn", "try")
        return graph_builder.compile()
