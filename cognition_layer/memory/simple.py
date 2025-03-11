"""
SimpleCognitiveMemory
==============================
This module provides functionality to manage memory of AI, specifically
of messages, and includes a class to handle cognitive memory operations.
"""
from typing import Optional
from typing import Sequence

from langchain_core.messages import BaseMessage


# ======================= UTILITIES ============================
def prune_images_from_messages(messages: Sequence[BaseMessage]):
    """
    - Remove image URLs from messages.
    - :params:
        messages (Sequence[BaseMessage]): A sequence of messages to be pruned.
    - :return: None
    """
    for message in messages:
        if not isinstance(message.content, Sequence):
            continue
        message.content = [
            dict_msg for dict_msg in message.content if dict_msg["type"] != "image_url"
        ]


# ======================= CLASSES ============================
class SimpleCognitiveMemory:
    """
    A class to manage cognitive memory of messages.

    Purpose:
        This class allows storing messages in a limited capacity, with options
        to preserve certain messages and control the handling of images.
    """

    def __init__(
        self,
        capacity: int = 5,
        keep_images: bool = True,
        preserve: Optional[Sequence[BaseMessage]] = None,
    ):
        """
        - Initialize the cognitive memory with specified parameters.
        - :params:
            capacity (int): The maximum number of messages to store.
            keep_images (bool): Flag to indicate whether to keep image messages.
            preserve (Optional[Sequence[BaseMessage]]): Messages to preserve at
            initialization.
        - :return: None
        - :raises ValueError: If the number of preserved messages exceeds capacity.
        """
        self.capacity = capacity
        self.keep_images = keep_images

        self._messages = []
        self._preserve = None

        if preserve is not None:
            self.preserve(preserve)

    def update(self, messages: Sequence[BaseMessage]):
        """
        - Update the cognitive memory with new messages.
        - :params:
            messages (Sequence[BaseMessage]): A sequence of new messages to store.
        - :return: None
        """
        assert isinstance(messages, Sequence)
        for message in messages:
            self._add_message(message)

        if not self.keep_images:
            prune_images_from_messages(self._messages)

    def preserve(self, messages: Sequence[BaseMessage]):
        preserved_messages = len(messages)
        if preserved_messages > self.capacity:
            raise ValueError(
                f"Cannot preserve {preserved_messages} with only {self.capacity} slots of capacity."
            )
        self.capacity -= preserved_messages
        self._preserve = messages

        while len(self._messages) > self.capacity:
            self._messages.pop(0)

    def _add_message(self, message: BaseMessage):
        """
        - Add a message to the cognitive memory.
        - :params:
            message (BaseMessage): The message to be added.
        - :return: None
        """
        self._messages.append(message)

        while len(self._messages) > self.capacity:
            self._messages.pop(0)

    @property
    def messages(self) -> list[BaseMessage]:
        """
        - Retrieve stored messages.
        - :return: A list of messages, including preserved ones if any.
        """
        if self._preserve is None:
            return self._messages

        return self._preserve + self._messages
