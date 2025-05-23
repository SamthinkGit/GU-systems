# flake8: noqa


def server_loader():
    from cognition_layer.protocols.fast_ap import autoserver
    from cognition_layer.experts.code_interpreter.agent import CodeInterpreterExpert

    return autoserver(CodeInterpreterExpert, name="CodeInterpreter")


DEPLOY_MODEL = {
    "name": "CodeInterpreterExpert",
    "alias": ["code-interpreter"],
    "agent_description": "A chat based advanced command line agent with memory that can perform complex tasks and calculations with files, directories, images, code, and more. ",
    "response_description": "Summary of the tasks performed or queries needed",
    "use_case": "Use me to send command line codes or retrieve information that can be gathered faster via cmd, automatically generate complex codes, manage files, images, audios, etc.",
    "welcome_message": "Please retrieve the information you consider relevant for the task (directory, filenames, etc) if any and then send me the task. You can also send me <reset> to erase my memory. Otherwise, I will remember our conversation. Ensure to warn to the user this can take some minutes.",
    "type": "agent",
    "packages": [],
    "server": server_loader,
}
