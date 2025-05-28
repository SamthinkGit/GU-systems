from typing import Generator

from langchain_core.messages import HumanMessage
from langchain_core.messages import SystemMessage

from cognition_layer.protocols.fast_ap import FastAPStep
from cognition_layer.retrieval.tools.instructions_parsing import load_vectorstore
from cognition_layer.routing.routers.simple_router.simple_router import SimpleRouter
from cognition_layer.tools.mutable_llm import MutableChatLLM
from ecm.mediator.Interpreter import Interpreter
from ecm.shared import get_logger


PROMPT = """
Translate the user query to english if it is not in English.
Also aggregate keywords about the user query.
E.g: install, web search, file management...
"""


class MonoRAG:
    _logger = get_logger(__name__)

    def __init__(
        self,
        store: str,
        interpreter: Interpreter,
        model: str = "gpt-4.1-nano",
        k: int = 3,
        *args,
        **kwargs,
    ):
        self.interpreter = interpreter
        self.store = load_vectorstore(store)
        self.retriever = self.store.as_retriever(search_kwargs={"k": k})
        self.model = model
        self.llm = MutableChatLLM(model=self.model)
        self.router = SimpleRouter(*args, **kwargs)
        self.routered_agent = list(self.router.agents.keys())[0]
        assert len(self.router.agents) == 1, "MonoRAG supports only one worker."

    def invoke(self, query: str) -> Generator[FastAPStep, None, None]:

        prompt = [SystemMessage(content=PROMPT), HumanMessage(content=query)]
        response = self.llm.invoke(prompt)
        self._logger.debug(f"New enriched query:\n{response.content}")
        enriched_query = response.content

        docs = self.retriever.invoke(enriched_query)

        new_query = (
            query + "\n\nThe following instructions are related to the query:\n\n"
        )
        for doc in docs:
            new_query += f"{doc.metadata['title']}: {doc.page_content}\n\n"

        server = self.router.server(self.routered_agent, interpreter=self.interpreter)
        if server is None:
            raise ValueError(f"Agent {self.routered_agent} not found in the schema.")

        yield from server.send_task(new_query)
