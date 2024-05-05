"""
Server-Client Observer
==============================

This module facilitates communication between server and client
within a system by managing subscriptions and notifications. It
defines a singleton Publisher class and a Subscriber class to
handle event-based messaging.
"""
from typeguard import Callable


class Publisher:
    """
    [SINGLETON] class that manages a list of subscribers and broadcasts notifications to them.
    This class ensures that there is only one instance of the publisher in the application,
    managing a centralized list of subscribers to distribute notifications.
    """

    _instance = None

    def __new__(cls):
        """
        Ensures that only one instance of Publisher exists, following the singleton pattern.
        """
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        cls.subscribers = []
        return cls._instance

    @classmethod
    def add_subscriber(cls, subscriber):
        """
        Adds a new subscriber to the publisher's list if not already added.
        :param subscriber: The subscriber to add.
        """
        if subscriber not in cls.subscribers:
            cls.subscribers.append(subscriber)

    @classmethod
    def notify_subscribers(cls, task_id):
        """
        Notifies all subscribers about an event identified by the task_id.
        :param task_id: The identifier of the task to notify about.
        """
        for subscriber in cls.subscribers:
            subscriber.notify(task_id)


class Subscriber:
    """
    Represents a subscriber that can receive notifications from the Publisher.
    This class is designed to be instantiated with a specific callback
    function that gets called when a notification is received.
    """

    def __init__(self, target: Callable):
        """
        Initializes the Subscriber with a target callable to be invoked on notifications.
        :param target: The callable function that handles notifications.
        """
        self.target = target

    def notify(self, task_id):
        """
        Invokes the target callable, passing the task_id of the notification event.
        :param task_id: The identifier of the task associated with the notification.
        """
        self.target(task_id)
