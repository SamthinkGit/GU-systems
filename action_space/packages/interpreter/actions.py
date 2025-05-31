from cognition_layer.experts.code_interpreter.core.interpreter import Interpreter
from cognition_layer.tools.temporary_environ import temporary_environ
from ecm.tools.registry import ItemRegistry


PKG_NAME = "_private_interpreter-tools"


@ItemRegistry.register(type="tool", package=PKG_NAME)
def _invoke_interpreter_with_animation(
    query: str,
    openai_api_key: str,
    auto_run: bool = False,
    model: str = "gpt-4o-mini",
) -> str:
    with temporary_environ({"OPENAI_API_KEY": openai_api_key}):
        interpreter = Interpreter(model=model, verbose=False)
        return interpreter.invoke(query, animation=True, auto_run=auto_run)
