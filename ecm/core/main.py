import asyncio
import multiprocessing
import time

import agent_protocol_client
import requests
from agent_protocol_client.models import TaskStepsListResponse
from agent_protocol_client.rest import ApiException

import tests.mocks.agent_actions  # noqa
from cognition_layer.constants import API_ADDRESS
from cognition_layer.constants import API_PORT
from cognition_layer.templates import ServerAPI
from ecm.mediator.Interpreter import Interpreter
from ecm.shared import get_logger
# TOOLS


async def main(cognition_layer: str = "PLANEX", execution_layer: str = "ROSA"):

    host = f"http://{API_ADDRESS}:{API_PORT}"
    configuration = agent_protocol_client.Configuration(host=host)
    logger = get_logger("main")

    logger.debug(f"Running {cognition_layer} as cognition_layer")
    match cognition_layer:
        case "PLANEX":

            from cognition_layer.planex.api.server import PlanexServer

            server = PlanexServer(verbose=True)

        case _:
            return ValueError(
                "Cognition Layer not valid, the unique supported values are: "
                + str(
                    [
                        "PLANEX",
                    ]
                )
            )

    logger.debug(f"Running {execution_layer} as execution_layer")
    match execution_layer:
        case "ROSA":
            from ecm.mediator.rosa_interpreter import RosaInterpreter

            interpreter = RosaInterpreter()

        case _:
            return ValueError(
                "Execution Layer not valid, the unique supported values are: "
                + str(
                    [
                        "ROSA",
                    ]
                )
            )

    server: ServerAPI
    interpreter: Interpreter

    server_process = multiprocessing.Process(target=server.start)
    server_process.start()

    info_shown = False
    status_code = 404

    while status_code != 200:

        try:
            status_code = requests.get(host + "/ap/v1/agent/tasks").status_code
        except Exception:
            pass

        if not info_shown:
            logger.info("Waiting for API server to be alive...")
            info_shown = True
        time.sleep(0.5)

    async with agent_protocol_client.ApiClient(configuration) as api_client:

        api_instance = agent_protocol_client.AgentApi(api_client)

        while True:

            task = agent_protocol_client.TaskRequestBody()
            task.input = input("Request a Task: ")

            # Request a task
            try:
                task_response = await api_instance.create_agent_task(task)
            except ApiException as e:
                print("Exception when calling AgentApi: %s\n" % e)

            # Obtain steps
            step_list: TaskStepsListResponse = await api_instance.list_agent_task_steps(
                task_id=task_response.task_id
            )
            logger.debug("Steps received from API")

            if len(step_list.steps) <= 0:
                logger.warn("Steps received are empty? Skip.")
                continue

            logger.info("Loading...")
            # Execute Cognition Layer
            step = step_list.steps[0]

            counter = 0
            result = None
            while step.is_last is False:

                step_list = await api_instance.list_agent_task_steps(
                    task_id=task_response.task_id
                )

                step = step_list.steps[counter]
                if result:
                    step.input = result.output

                result = await api_instance.execute_agent_task_step(
                    task_id=task_response.task_id, step_request_body=step
                )

                counter += 1

            logger.debug(result.output)
            interpreter.kill()


if __name__ == "__main__":
    asyncio.run(main(execution_layer="ROSA", cognition_layer="PLANEX"))
