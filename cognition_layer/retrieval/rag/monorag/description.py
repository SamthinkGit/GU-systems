# flake8: noqa


def server_loader():
    from cognition_layer.protocols.fast_ap import autoserver
    from cognition_layer.retrieval.rag.monorag.rag import MonoRAG

    return autoserver(MonoRAG, name="MonoRAG")


DEPLOY_MODEL = {
    "name": "MonoRAG",
    "alias": ["monorag", "mono-rag"],
    "agent_description": "A RAG that only uses one vectorstore and appends relevant instructions to the worker.",
    "response_description": "Original query + relevant instructions",
    "use_case": "Use me to retrieve instructions related to your query.",
    "welcome_message": "I should not be used by agents",
    "type": "router",
    "packages": [],
    "server": server_loader,
}
