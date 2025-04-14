import random
from dataclasses import dataclass
from typing import Generator

from cognition_layer.memory.simple import SimpleCognitiveMemory
from cognition_layer.tools.ocr.engine import OCR
from cognition_layer.visual_fast_react.agents.graph import compile_visual_fast_react_graph
from cognition_layer.visual_fast_react.agents.graph import VfrState
from ecm.mediator.Interpreter import Interpreter
from ecm.shared import get_logger
from ecm.tools.item_registry_v2 import ItemRegistry


# ======================= CLASSES ============================
@dataclass
class VisualFastReactStep:

    name: str
    content: str
    is_last: bool


class VisualFastReact:

    _logger = get_logger("VisualFastReact")

    def __init__(
        self,
        interpreter: Interpreter = None,
        registry: ItemRegistry = ItemRegistry(),
        memory_capacity: int = 10,
        ocr: OCR = OCR(),
    ):
        self.registry = registry
        self.memory_capacity = memory_capacity
        self.interpreter = interpreter
        self.graph = compile_visual_fast_react_graph()
        self.ocr = ocr

        self._logger.debug("Compilation succesful")

    def iter(self, query: str) -> Generator[VisualFastReactStep, None, None]:
        self._logger.debug(f"Starting graph for query: `{query}`")

        config = {"configurable": {"thread_id": str(random.randint(5000, 15000))}}
        initial_state = {
            "query": query,
            "registry": self.registry,
            "memory": SimpleCognitiveMemory(capacity=self.memory_capacity),
            "screen_focus": "fullscreen",
            "ocr": self.ocr,
            "fr_next_state": "undefined",
            "interpreter": self.interpreter,
            "is_last": False,
            "latest_summary": "",
        }
        step: VfrState
        for step in self.graph.stream(initial_state, config, stream_mode="values"):

            yield VisualFastReactStep(
                name="VFR", content=step["latest_summary"], is_last=step["is_last"]
            )
