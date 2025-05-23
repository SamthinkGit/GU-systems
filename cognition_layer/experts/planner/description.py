# flake8: noqa


def server_loader():
    from cognition_layer.experts.planner.server import get_fast_ap_server

    return get_fast_ap_server


DEPLOY_MODEL = {
    "name": "PlannerExpert",
    "alias": ["planner", "planner-expert"],
    "agent_description": "A vision agent that can generate a plan to archieve a goal",
    "response_description": "A plan based on the current state to archieve your goal. I also usually can verify your plans and/or help you when something goes wrong. Also use me for starting a plan.",
    "use_case": "Use me always that you need to generate a plan with multiple steps",
    "welcome_message": "Please, inform me about other experts, additionally, send me the goal you want and some context about your preferences and the context you need.",
    "type": "agent",
    "packages": [
        "screenshot",
    ],
    "server": server_loader,
}
