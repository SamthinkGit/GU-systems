from colorama import Fore
from colorama import Style

import action_space.keyboard.pynput  # noqa
import action_space.window.focus  # noqa
from cognition_layer.planexv2.agents.planexv2 import PlanexV2


if __name__ == "__main__":

    query = input("Select Task (Press ENTER for deafault): ")
    if query == "":
        query = "Open a python http server"

    print(Fore.GREEN + Style.BRIGHT + "[Executing PlanexV2]" + Style.RESET_ALL)
    planex = PlanexV2()

    for step in planex.iter(input=query):
        print(Fore.GREEN + Style.BRIGHT + f"[{step.agent}]" + Style.RESET_ALL)
        print(Fore.YELLOW + Style.BRIGHT + f"{step.content}" + Style.RESET_ALL)
