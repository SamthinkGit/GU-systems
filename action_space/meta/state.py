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
    """

    def __init__(self, schema: Type[BaseModel], storage_key: str = "default") -> None:
        self._schema_cls = schema
        try:
            self._model = schema()
        except ValidationError:
            self._model = schema.model_construct()
        Storage("CognitionState")[storage_key] = self

    def set(self, key: str, value: Any) -> None:
        if key not in self._schema_cls.model_fields:
            raise AttributeError(
                f"Field '{key}' is not defined in {self._schema_cls.__name__}"
            )
        setattr(self._model, key, value)

    def get(self, key: str) -> Any:
        if key not in self._schema_cls.model_fields:
            raise AttributeError(
                f"Field '{key}' is not defined in {self._schema_cls.__name__}"
            )
        return getattr(self._model, key)

    def supports(self, *args: str) -> bool:
        """
        Check if the state supports the given keys.
        """
        for key in args:
            if key not in self._schema_cls.model_fields:
                return False
        return True

    def summary(self) -> str:
        """
        Produce a JSON-formatted summary string of the current state,
        including field types and descriptions from the pydantic schema.
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
    state: CognitionState = Storage("CognitionState").get(state_key)
    assert (
        state is not None
    ), f"State '{state_key}' not found in Storage. Please initialize a CognitionState before using a setter."
    state.set(key, value)
