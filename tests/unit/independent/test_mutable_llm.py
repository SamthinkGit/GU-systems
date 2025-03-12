from cognition_layer.tools.mutable_llm import MutableChatLLM
from ecm.shared import get_root_path


def test_mutable_llm_config_is_being_parsed_correctly():
    from langchain_openai import ChatOpenAI
    MutableChatLLM.config_path = (
        get_root_path() / "tests" / "resources" / "llm_config_example_1.yaml"
    )
    llm = MutableChatLLM()
    assert isinstance(llm, ChatOpenAI)
    assert llm.model_name == "gpt-4o-mini"


def test_local_llm():
    from langchain_ollama import ChatOllama
    MutableChatLLM.config_path = (
        get_root_path() / "tests" / "resources" / "llm_config_example_2.yaml"
    )
    llm = MutableChatLLM()
    assert isinstance(llm, ChatOllama)
    assert llm.model == "llama-3.2-vision:8b"
