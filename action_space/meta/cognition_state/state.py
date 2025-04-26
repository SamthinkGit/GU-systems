"""
CognitionState Module
==============================
This module provides the `CognitionState` class, a mutable state container
for managing LLM cognition using a pydantic schema. It allows for dynamic
state management, and integration with agentic actions. The module
also includes utility functions for interacting with the state and ensuring
schema compliance.
"""
# ======================= CLASSES ============================
import json
from typing import Any
from typing import Callable
from typing import Type

from pydantic import BaseModel
from pydantic import ValidationError

from ecm.tools.item_registry_v2 import Storage


class CognitionState:
    """
    Mutable state container for LLM cognition based on a pydantic schema.

    This class provides methods to dynamically set, get, and validate state
    fields defined in a pydantic schema. It also supports generating a
    JSON-formatted summary of the current state and ensures compatibility
    with required fields.
    """

    def __init__(self, schema: Type[BaseModel], storage_key: str = "default") -> None:
        """
        Initializes the CognitionState with a given pydantic schema.

        Args:
            schema (Type[BaseModel]): The pydantic schema defining the state.
            storage_key (str): The key used to store the state in the
                `Storage` registry. Defaults to "default".
        """
        self._schema_cls = schema
        try:
            self._model = schema()
        except ValidationError:
            self._model = schema.model_construct()
        Storage("CognitionState")[storage_key] = self

    def set(self, key: str, value: Any) -> None:
        """
        Sets a value for a specific field in the state.

        Args:
            key (str): The name of the field to set.
            value (Any): The value to assign to the field.

        Raises:
            AttributeError: If the field is not defined in the schema.
        """
        if key not in self._schema_cls.model_fields:
            raise AttributeError(
                f"Field '{key}' is not defined in {self._schema_cls.__name__}"
            )
        setattr(self._model, key, value)

    def get(self, key: str) -> Any:
        """
        Retrieves the value of a specific field in the state.

        Args:
            key (str): The name of the field to retrieve.

        Returns:
            Any: The value of the specified field.

        Raises:
            AttributeError: If the field is not defined in the schema.
        """
        if key not in self._schema_cls.model_fields:
            raise AttributeError(
                f"Field '{key}' is not defined in {self._schema_cls.__name__}"
            )
        return getattr(self._model, key)

    def supports(self, *args: str) -> bool:
        """
        Checks if the state is supporting the given keys.

        Args:
            *args (str): The keys to check for support.

        Returns:
            bool: True if all keys are supported, False otherwise.
        """
        for key in args:
            if key not in self._schema_cls.model_fields:
                return False
        return True

    def summary(self) -> str:
        """
        Produces a JSON-formatted summary string of the current state.

        The summary includes field types, descriptions, and current values
        as defined in the pydantic schema.

        Returns:
            str: A JSON-formatted string summarizing the state.
        Examples:
            >>> state = CognitionState(MySchema)
            >>> state.set("field1", "value1")
            >>> print(state.summary())
            {
                "field1": {
                    "type": "str",
                    "description": "Field 1 description",
                    "value": "value1"
                },
                ...
            }
        """
        summary_data: dict[str, dict[str, Any]] = {}
        for name, field in self._schema_cls.model_fields.items():
            summary_data[name] = {
                "type": str(field.annotation),
                "description": str(field.description),
                "value": str(getattr(self._model, name, False)) or "None",
            }
        return json.dumps(summary_data, indent=2)

    @staticmethod
    def require_supports(*supports: str, storage_key: str = "default") -> Callable:
        """
        Decorator to ensure that the state supports specific keys before
        executing a function.

        Args:
            *supports (str): The keys that must be supported.
            storage_key (str): The key used to retrieve the state from
                `Storage`. Defaults to "default".

        Returns:
            Callable: The decorated function.

        Raises:
            AttributeError: If the required keys are not supported.
        Examples:
            >>> @CognitionState.require_supports("key1", "key2")
            >>> def my_function(state: CognitionState):
            ...    # Function logic here
            ...    pass
            >>> state = CognitionState(MySchema)
            >>> state.set("key1", "value1")
            >>> state.set("key2", "value2")
            >>> my_function(state)
            >>> # Function executed successfully
        """

        def decorator(func: Callable) -> Callable:
            def wrapper(*args, **kwargs):
                state = Storage("CognitionState")[storage_key]
                if not state.supports(*supports):
                    raise AttributeError(
                        f"CognitionState does not support the following keys: {', '.join(args)}"
                    )
                return func(*args, **kwargs)

            return wrapper

        return decorator


def _cognition_state_setter(key: str, value: Any, state_key: str = "default") -> None:
    """
    Sets a value in the CognitionState for a specific key.

    Args:
        key (str): The name of the field to set.
        value (Any): The value to assign to the field.
        state_key (str): The key used to retrieve the state from `Storage`.
            Defaults to "default".

    Raises:
        AssertionError: If the state is not found in `Storage`.
    """
    state: CognitionState = Storage("CognitionState").get(state_key)
    assert (
        state is not None
    ), f"State '{state_key}' not found in Storage. Please initialize a CognitionState before using a setter."
    state.set(key, value)
