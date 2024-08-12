from cognition_layer.xplore.agents.general_planner import GeneralPlannerAgent
from cognition_layer.xplore.agents.general_planner import GeneralPlannerResponse
from ecm.tools.prettify import pretty_print

if __name__ == "__main__":

    agent = GeneralPlannerAgent()
    plan: GeneralPlannerResponse = agent.generate_plan("Buy AAA batteries on Amazon.")
    pretty_print(plan)
