import asyncio

import agent_protocol_client
from agent_protocol_client.models import TaskStepsListResponse
from agent_protocol_client.rest import ApiException
from colorama import Fore
from colorama import Style

import ecm.exelent.parser as parser
import tests.mocks.agent_actions  # noqa
from cognition_layer.planex.utils.format import extract_python_code
from ecm.mediator.rosa_interpreter import RosaInterpreter


configuration = agent_protocol_client.Configuration(host="http://0.0.0.0:8000")


async def main():
    # Enter a context with an instance of the API client
    async with agent_protocol_client.ApiClient(configuration) as api_client:
        # Create an instance of the API class
        api_instance = agent_protocol_client.AgentApi(api_client)
        task = agent_protocol_client.TaskRequestBody()
        task.input = input("Input task: ")
        try:
            # Creates a task for the agent.
            task_response = await api_instance.create_agent_task(task)
        except ApiException as e:
            print("Exception when calling AgentApi: %s\n" % e)

        step_list: TaskStepsListResponse = await api_instance.list_agent_task_steps(
            task_id=task_response.task_id
        )

        result = None
        for step in step_list.steps:

            if result:
                step.input = result.output
            result = await api_instance.execute_agent_task_step(
                task_id=task_response.task_id, step_request_body=step
            )
            print(
                Fore.YELLOW
                + Style.BRIGHT
                + "[Client] Step Response:\n"
                + result.output
                + "\n"
                + Style.RESET_ALL
            )

        print(Fore.GREEN + Style.BRIGHT + "[Parsing...]" + Style.RESET_ALL)
        plan = result.output

        if plan.startswith("```python"):
            plan = extract_python_code(plan)

        task = parser.parse(target_str=plan)
        interpreter = RosaInterpreter()

        print(Fore.YELLOW + Style.BRIGHT + "Generated Plan:\n" + Style.RESET_ALL)
        for pkg in interpreter._generate_packages_from_parsed_task(task):
            print(pkg.to_json())

        print(Fore.GREEN + Style.BRIGHT + "[Running...]" + Style.RESET_ALL)
        interpreter.run(task, callback="silent")
        interpreter.rosa.kill()


if __name__ == "__main__":
    asyncio.run(main())
