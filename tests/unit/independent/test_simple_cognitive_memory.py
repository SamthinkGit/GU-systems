from action_space.tools.image import ImageMessage
from cognition_layer.memory.simple import SimpleCognitiveMemory

from langchain_core.messages import HumanMessage
from PIL import Image


def test_memory_is_not_overloading():
    memory = SimpleCognitiveMemory(capacity=3)

    memory.update(
        [
            HumanMessage(content="a"),
            HumanMessage(content="b"),
            HumanMessage(content="c"),
            HumanMessage(content="d"),
        ]
    )

    for msg, valid_content in zip(memory.messages, ["b", "c", "d"]):
        assert msg.content == valid_content


def test_memory_prunes_images():
    fake_img = Image.new("RGB", (200, 100), color="gray")

    memory = SimpleCognitiveMemory(capacity=3, keep_images=False)
    memory.update(
        [
            ImageMessage(image=fake_img, input="human").as_human(),
            ImageMessage(image=fake_img, input="ai").as_ai(),
            ImageMessage(image=fake_img, input="system").as_system(),
        ]
    )

    for msg in memory.messages:
        assert len(msg.content) == 1, "Image has not been extracted from the message?"


def test_memory_preservation():
    memory = SimpleCognitiveMemory(capacity=3, preserve=[HumanMessage("initial")])
    memory.update(
        [
            HumanMessage(content="a"),
            HumanMessage(content="b"),
            HumanMessage(content="c"),
            HumanMessage(content="d"),
        ]
    )

    for msg, valid_content in zip(memory.messages, ["initial", "c", "d"]):
        assert msg.content == valid_content
