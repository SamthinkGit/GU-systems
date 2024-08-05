import atexit
import json
import os
import traceback

import pika

from ecm.shared import get_logger
from ecm.shared import load_env
from ecm.tools.registry import ItemRegistry
load_env()


class EcmClient:

    _logger = get_logger("ECM Client")
    _instance = None

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
        channel.queue_purge("task_queue")
        channel.queue_purge("response_queue")
        channel.basic_consume(
            queue="task_queue", on_message_callback=EcmClient.on_request
        )
        EcmClient.connection = connection
        EcmClient.channel = channel
        atexit.register(EcmClient.cleanup)

        EcmClient._logger.info(
            f"Client successfully connected to {host}:{port}. Waiting for tasks..."
        )

    @classmethod
    def listen(cls):
        cls.channel.start_consuming()

    @classmethod
    def cleanup(cls):
        cls.conection.close()

    @classmethod
    def on_request(cls, ch, method, properties, body):

        EcmClient.channel.basic_publish(
            exchange="", routing_key="response_queue", body="ACK"
        )

        ch.basic_ack(delivery_tag=method.delivery_tag)
        task = json.loads(body)
        func_name = task["func_name"]
        args = task["args"]
        kwargs = task["kwargs"]

        cls._logger.debug(f"Received Task: {task}")
        exception = None

        try:
            func = ItemRegistry._names.get(func_name)
            if func is None:
                func = ItemRegistry._utils.get(func_name)
            if func is None:
                raise SystemError(f"Function {func_name} has not been found in the client ItemRegistry.")

            result = func(*args, **kwargs)

        except Exception:
            result = None
            exception = traceback.format_exc()

        response = {"result": result, "exception": exception}

        EcmClient.channel.basic_publish(
            exchange="", routing_key="response_queue", body=json.dumps(response)
        )
