import atexit
import functools
import json
import os
import time
from copy import deepcopy

import pika

from ecm.shared import get_logger
from ecm.shared import load_env
from ecm.tools.item_registry_v2 import ItemRegistry as ItemRegistryV2

load_env()


class EcmServer:

    _instance = None
    _logger = get_logger("ECM Server")
    _inited = False

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self) -> None:

        user = os.getenv("ECM_USER")
        password = os.getenv("ECM_PASS")
        host = os.getenv("ECM_HOST")
        port = int(os.getenv("ECM_PORT"))

        credentials = pika.PlainCredentials(username=user, password=password)
        parameters = pika.ConnectionParameters(
            host=host, port=port, credentials=credentials
        )
        connection = pika.BlockingConnection(parameters=parameters)
        channel = connection.channel()

        channel.queue_declare(queue="task_queue")
        channel.queue_declare(queue="response_queue")
        channel.queue_purge(queue="task_queue")
        channel.queue_purge(queue="response_queue")
        EcmServer.channel = channel
        EcmServer.conection = connection
        atexit.register(EcmServer.cleanup)
        EcmServer._inited = True

    @classmethod
    def cleanup(cls):
        EcmServer.conection.close()

    @classmethod
    def send_task(cls, func_name, *args, **kwargs):
        if not EcmServer._inited:
            EcmServer()

        task = {"func_name": func_name, "args": args, "kwargs": kwargs}
        cls._logger.debug(f"Publishing task: {task} to client.")

        cls.channel.basic_publish(
            exchange="", routing_key="task_queue", body=json.dumps(task)
        )
        start = time.perf_counter()
        acknowledged = False

        for method_frame, _, body in cls.channel.consume(
            "response_queue", inactivity_timeout=1
        ):
            if body == b"ACK":
                acknowledged = True

            if not acknowledged and time.perf_counter() - start > 1:
                raise ConnectionAbortedError(
                    "[Timeout] Task not confirmed from the server. Maybe client is not connected yet?"
                )

            if body is not None and body != b"ACK":
                response = json.loads(body)
                cls._logger.debug("Reponse received from client.")
                cls.channel.basic_ack(method_frame.delivery_tag)
                break

        if response["exception"] is not None:
            cls._logger.error(
                "Error on client when execution a task:\n" + response["exception"]
            )
            raise SystemError(response["exception"])

        return response["result"]

    @classmethod
    def wrap_item_registry(cls, registry: ItemRegistryV2 = ItemRegistryV2()):

        cls._logger.warning(
            f"All functions within `{registry.name}` registry will be executed on the clients "
            "connected to the ECM Server."
        )

        new_items = []
        for item in list(registry.actions.values()) + list(registry.tools.values()):
            remote_function = functools.wraps(item.content)(
                lambda *args, name=item.name, **kwargs: EcmServer.send_task(
                    name, *args, **kwargs
                )
            )
            cp = deepcopy(item)
            cp.content = remote_function
            cp.labels.append("Remote")
            new_items.append(cp)

        registry.actions = {}
        registry.tools = {}

        for item in new_items:
            registry._load_to_correspondent(item, silent=True)
