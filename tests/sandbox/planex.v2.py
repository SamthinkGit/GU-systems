from colorama import Fore
from colorama import Style

import action_space.keyboard.pynput # noqa
import action_space.window.focus # noqa
from cognition_layer.planex.agents.planner import Planner
from cognition_layer.planex.agents.reducer import Reducer
from cognition_layer.planex.agents.translator import Translator
from cognition_layer.planexv2.agents.blamer import BlameInfo
from cognition_layer.planexv2.agents.blamer import Blamer

if __name__ == "__main__":

    query = input("Select Task (Press ENTER for deafault): ")
    if query == "":
        query = "Open a python http server"

    planner = Planner()
    reducer = Reducer()
    translator = Translator()
    blamer = Blamer()
    print(Fore.GREEN + Style.BRIGHT + "[Planning...]" + Style.RESET_ALL)

    planner_output = planner.plan(query, verbose=True).content
    reducer_output = reducer.reduce(planner_output, verbose=True).content
    translator_output = translator.translate(reducer_output, verbose=True).content

    blame = BlameInfo(
        query=query,
        planner_output=planner_output,
        reducer_output=reducer_output,
        translator_output=translator_output,
        reducer_tools=reducer.actions,
        additional_info=""
    )
    print(Fore.GREEN + Style.BRIGHT + "[Blaming...]" + Style.RESET_ALL)
    blamer.run(blame, verbose=True)
