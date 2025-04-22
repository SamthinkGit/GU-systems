import pytest
from pydantic import BaseModel
from pydantic import Field

from action_space.meta.state import CognitionState
from ecm.tools.item_registry_v2 import Storage


class ExampleState(BaseModel):
    example_field: str = Field(description="An example field for testing purposes.")
    another_field: int = Field(
        description="Another example field for testing purposes."
    )


def test_cognition_state_building():
    state = CognitionState(ExampleState)
    state.summary()


def test_cognition_state_setter():
    state = CognitionState(ExampleState)
    state.set("example_field", "test_value")
    state.set("another_field", 0)
    assert state.get("example_field") == "test_value"
    assert state.get("another_field") == 0


def test_cognition_state_remaining_through_storage():
    state = CognitionState(ExampleState)
    state.set("example_field", "test_value")

    same_state = Storage("CognitionState").get("default")
    assert same_state.get("example_field") == "test_value"


def test_supports_are_valid():
    state = CognitionState(ExampleState)
    assert state.supports("example_field", "another_field") is True
    assert state.supports("example_field", "non_existent_field") is False


def test_support_decorator():

    @CognitionState.require_supports("non_existent_field")
    def example():
        pass

    state = CognitionState(ExampleState) # noqa
    with pytest.raises(AttributeError):
        example()
