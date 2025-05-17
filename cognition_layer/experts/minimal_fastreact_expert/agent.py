import json

from langchain.tools import tool as build_tool
from langchain_core.messages import AIMessage
from langchain_core.tools import StructuredTool
from pydantic import BaseModel
from pydantic import Field

from action_space.meta.call_expert.sync import sync_experts
from cognition_layer.agents.minimal_vfr.agents.agent import _convert_response_to_exelent
from cognition_layer.agents.minimal_vfr.agents.agent import TaskSummary
from cognition_layer.agents.planex.utils.format import format_tool
from cognition_layer.deploy.types import DeployModel
from cognition_layer.deploy.types import DeploySchema
from cognition_layer.memory.simple import SimpleCognitiveMemory
from cognition_layer.routing.routers.simple_router.simple_router import SimpleRouter
from cognition_layer.tools.mutable_llm import MutableChatLLM
from ecm.mediator.feedback import ExecutionStatus
from ecm.mediator.feedback import Feedback
from ecm.mediator.Interpreter import Interpreter
from ecm.tools.registry import ItemRegistry


class MfreResponse(BaseModel):
    reasoning: str = Field(description="A reasoning for about the next steps")
    function: str = Field(
        description="The next function with pythonic notation. E.g: myfunc(2, 3, 'foo')."  # noqa
    )


class MinimalFastReactExpert:

    def __init__(
        self,
        interpreter: Interpreter = None,
        registry: ItemRegistry = ItemRegistry(),
        memory_capacity: int = 10,
        model: str = "gpt-4.1-nano",
        schema: DeploySchema = None,
    ):
        self.schema = schema
        if schema is not None:
            self.router = SimpleRouter(schema)
        self.llm = MutableChatLLM(model=model).with_structured_output(MfreResponse)
        self.registry = registry
        self.interpreter = interpreter
        self.memory_capacity = memory_capacity
        self.cognition_state = None
        self.formatted_tools = None
        self.memory = None

        actions = [action.content for action in self.registry.actions.values()]
        tools: list[StructuredTool] = [build_tool(a) for a in actions]
        self.formatted_tools = [format_tool(t) for t in tools]
        if self.schema is not None:
            self.formatted_experts = json.dumps(self.router.summary_dict(), indent=2)

    def _retrieve_task_result(self, feedback: Feedback):
        assert (
            self.latest_task is not None
        ), "No task has been initialized executed yet."
        self.latest_task: TaskSummary

        if feedback._exec_status == ExecutionStatus.SUCCESS:
            self.latest_task.success = True

        if feedback._exec_status == ExecutionStatus.RESULT:
            self.latest_task.result = feedback.object

    def restart(self):
        self.memory = SimpleCognitiveMemory(
            capacity=self.memory_capacity, keep_images=False
        )

    def execute_response(self, response: MfreResponse):
        if self.schema is not None:
            sync_experts(self.schema, self.interpreter)
        task = _convert_response_to_exelent(response)
        self.latest_task = TaskSummary()
        if task is not None:
            self.interpreter.run(task, callback=self._retrieve_task_result)
        else:
            self.latest_task = TaskSummary(success=True, result="Invalid syntax")

        self.memory.update(
            [
                AIMessage(content=str(response)),
                AIMessage(
                    content=f"`{response.function}` returned the following summary: `{self.latest_task}`"
                ),
            ]
        )


def custom_schema(models: list[DeployModel]):
    custom_schema = {
        "name": "ElementFinder",
        "type": "agent",
        "model": "None",
        "config": {},
        "packages": [],
        "workers": [
            {
                "name": model["name"],
                "type": "agent",
                "model": model["alias"][0],
                "config": model.get("config", {}),
                "packages": model.get("packages", []),
                "workers": [],
            }
            for model in models
        ],
    }
    return custom_schema
