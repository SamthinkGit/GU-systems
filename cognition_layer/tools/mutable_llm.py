"""
MutableChatLLM Module
==============================
This module defines the MutableChatLLM class, which extends the
BaseChatModel from the Langchain framework. It allows for the
creation of chat models based on configuration files and
provides methods for handling structured responses from
language models.

The configuration can be found at "cogntion_layer/llm_config.yaml"
"""
import re
from ecm.shared import get_root_path
from pathlib import Path
from typing import ClassVar
from typing import Optional

import yaml
from langchain_core.language_models import BaseChatModel
from langchain_core.messages import HumanMessage
from langchain_core.messages import SystemMessage
from pydantic import BaseModel


# ======================= CLASSES ============================
class MutableChatLLM(BaseChatModel):
    """
    A mutable chat LLM that can be instantiated based on
    configuration settings.
    """

    config_path: ClassVar[Path] = (
        get_root_path() / "cognition_layer" / "llm_config.yaml"
    )

    def __new__(cls, *args, enforce_structured_output: Optional[bool] = None, **kwargs):
        """
        - Creates a new instance of the chat model based on the llm_config.yaml
        - :params:
            - enforce_structured_output: Optional flag to enforce structured
              output based on configuration.
        - :return: An instance of the appropriate chat model based on
          configuration.
        - :raises: ValueError if the model is not supported
        """

        with open(cls.config_path) as file:
            config = yaml.safe_load(file)

        kwargs.setdefault("model", config["model"])
        if enforce_structured_output is None:
            enforce_structured_output = config["enforce_structured_output"]
        if config["local"]:
            from langchain_ollama import ChatOllama

            instance = ChatOllama(*args, **kwargs)

        else:
            from langchain_openai import ChatOpenAI

            instance = ChatOpenAI(*args, **kwargs)

        if enforce_structured_output:
            instance.__dict__["original_invoke"] = instance.invoke

            instance.__dict__["with_structured_output"] = (
                lambda schema: MutableChatLLM.with_structured_output(instance, schema)
            )
            instance.__dict__["invoke"] = (
                lambda schema, *args, **kwargs: MutableChatLLM.structured_invoke(
                    instance, schema, *args, **kwargs
                )
            )

        return instance

    def with_structured_output(self, schema: BaseModel):
        """
        - Initializes the model with a structured output schema.
        """
        properties: dict = schema.model_json_schema()["properties"]
        self.__dict__["schema_class"] = schema
        self.__dict__["structured_prompt"] = SystemMessage(
            content="From now your response must always contain the following labels: \n"
            + "\n".join(
                [
                    f"<{key}>{label['description']}</{key}>. Type: {label['type']}"
                    for key, label in properties.items()
                ]
            )
        )
        return self

    def structured_invoke(self, input, *args, **kwargs):
        """
        - Invokes the chat model with the provided input, enforcing
          structured output based on the defined schema.
        - :params:
            - input: The input to the model, which can be a string or
              a list of messages.
        - :return: A structured output based on the input and schema.
        - :raises: ValueError if the input type is unsupported.
        """

        start = SystemMessage(content="Start now answering with the provided labels")
        if isinstance(input, str):
            prompt = [self.structured_prompt, HumanMessage(content=input), start]
        elif isinstance(input, list):
            messages_copy = input.copy()
            messages_copy.insert(0, self.structured_prompt)
            messages_copy.append(start)
            prompt = messages_copy
        else:
            raise ValueError(
                f"{input.__class__} not supported for enforced structured output."
            )

        response = self.original_invoke(prompt, *args, **kwargs)
        labels = extract_labels(response.content)
        return self.schema_class(**labels)


def extract_labels(text: str) -> dict[str, str]:
    """
    - Extracts labeled content from the provided text using regular
      expressions.
    """
    matches = re.findall(r"<([a-zA-Z0-9_]+)>\s*(.*?)\s*</\1>", text, re.DOTALL)
    return {label: content.strip() for label, content in matches}
