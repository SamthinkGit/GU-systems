import action_space.experimental.mouse.actions # noqa
import action_space.keyboard.virtual_keyboard # noqa
from cognition_layer.xplore.agents.general_planner import GeneralPlannerAgent
from cognition_layer.xplore.agents.general_planner import GeneralPlannerResponse
from cognition_layer.xplore.agents.subgoal_planner import SubgoalPlanner
from cognition_layer.xplore.agents.subgoal_planner import SubgoalResponse
from ecm.tools.prettify import pretty_print
# ---- ACTIONS -----

if __name__ == "__main__":

    gen_agent = GeneralPlannerAgent()
    sub_agent = SubgoalPlanner()

    query = "Buy AAA batteries on Amazon."
    problem_description: GeneralPlannerResponse = gen_agent.invoke(query=query)
    additional_info = f"""Note that the current status of the computer can be described as:
        ```{problem_description.description}```
        """

    response: SubgoalResponse = sub_agent.invoke(
        goal=query, plan=problem_description.plan, additional_info=additional_info
    )
    pretty_print(problem_description, header="GeneralPlanner")
    pretty_print(response, header="SubgoalPlanner")
