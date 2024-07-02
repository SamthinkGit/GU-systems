import asyncio

from colorama import Fore
from colorama import Style

import action_space.keyboard.pynput  # noqa
import action_space.window.focus  # noqa
from cognition_layer.RePlan.agents.async_replan import AsyncRePlan as RePlan
from cognition_layer.RePlan.agents.async_replan import ReplanResponse
from ecm.tools.async_interpreter import AsyncInterpreter
from ecm.tools.registry import ItemRegistry
from execution_layer.rosa.interpreter.rosa_interpreter import RosaFeedbackWrapper
from execution_layer.rosa.interpreter.rosa_interpreter import RosaInterpreter


async def main():
    ItemRegistry.invalidate_all_functions()

    rosa = RosaInterpreter()
    interpreter = AsyncInterpreter()
    interpreter.build(interpreter=rosa, feedback_class=RosaFeedbackWrapper)
    replan = RePlan(
        async_interpreter=interpreter,
        verbose=False,
    )

    step: ReplanResponse
    async for step in replan.iter(query="Open the terminal in linux"):
        print(Fore.GREEN + Style.BRIGHT + f"[{step.name}]" + Style.RESET_ALL)
        print(Fore.YELLOW + step.content + Style.RESET_ALL)

    rosa.kill()


if __name__ == "__main__":
    asyncio.run(main())
