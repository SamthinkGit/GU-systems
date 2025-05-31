# flake8: noqa


def server_loader():
    from cognition_layer.protocols.fast_ap import autoserver
    from cognition_layer.experts.code_interpreter.agent import CodeInterpreterExpert

    return autoserver(CodeInterpreterExpert, name="CodeInterpreter")


DEPLOY_MODEL = {
    "name": "CodeAndFileManagementInterpreterExpert",
    "alias": ["code-interpreter"],
    "agent_description": "An ubiquos chat based command line agent with memory that can perform complex tasks and calculations with files, directories, images, code, and more. Use it for creating files, finding them, editing them, visualizing directories, and more.",
    "response_description": "Summary of the tasks performed or queries needed",
    "use_case": "Use me to retrieve information that can be gathered faster via cmd, automatically generate complex codes, manage files, images, audios, etc.",
    "welcome_message": "Please ensure to warn to the user this can take some minutes and the execution of commands will be confirmated when necessary. You can also send me <reset> to erase my memory. Otherwise, I will remember our conversation. ",
    "type": "agent",
    "packages": [
        "_private_interpreter-tools",
    ],
    "server": server_loader,
}
