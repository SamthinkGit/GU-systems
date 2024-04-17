from typeguard import Callable


class Publisher:

    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        cls.subscribers = []
        return cls._instance

    @classmethod
    def add_subscriber(cls, subscriber):
        if subscriber not in cls.subscribers:
            cls.subscribers.append(subscriber)

    @classmethod
    def notify_subscribers(cls, task_id):
        for subscriber in cls.subscribers:
            subscriber.notify(task_id)


class Subscriber:
    def __init__(self, target: Callable):
        self.target = target

    def notify(self, task_id):
        self.target(task_id)
