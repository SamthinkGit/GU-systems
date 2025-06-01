from typing import Generator

from langchain_core.messages import HumanMessage
from langchain_core.messages import SystemMessage

from cognition_layer.deploy.types import DeploySchema
from cognition_layer.protocols.fast_ap import FastAPStep
from cognition_layer.routing.routers.simple_router.simple_router import SimpleRouter
from cognition_layer.tools.mutable_llm import MutableChatLLM
from cognition_layer.tools.voice.engine import play
from cognition_layer.tools.voice.engine import text2speech
from ecm.mediator.Interpreter import Interpreter
from ecm.shared import get_logger
from ecm.shared import get_root_path

LANGUAGE = "spanish"

START_PROMPT = f"""
Based on the user's command or request, generate a short, casual, and engaging
sentence in {LANGUAGE} that confirms you're starting the task. Follow these rules:

Rules:

- Use a natural and relaxed tone, as if you were speaking with confidence and
personality.

- The response must be not so long.
- Avoid generic or robotic replies.
- The response should contain a hint/confirmation and a small opinion/reaction.

Examples:
User: "Please open Spotify"
 Response: "Abrir Word, entendido. Vamos a ello sin perder tiempo."

User: "Muéstrame una imagen de un atardecer"
 Response: "Con tu permiso, empiezo a buscar el mejor atardecer del día."

User: "Quiero que escribas un texto inspirador"
 Response: "Esto pinta bien.  Activando modo creativo ahora mismo."

User: "Necesito que analices estos datos"
 Response: "¡Claro! Esto va a estar listo antes de que pestañees."
"""

END_PROMPT = f"""
After attempting the task, generate a short, casual, and friendly closing sentence in {LANGUAGE}.
Rules:
- Use a casual, confident, and slightly playful tone.
- Keep it always short (8-10 words max).
- Always suggest the task was attempted or completed — no guarantees.
- Include a subtle opinion or reaction.
- Do not overuse words as "Perfecto!" or "Listo", be more creative as in the example.

Behavior based on the type of query:
- If the user query was a question:
> Answer the question directly using the task results or context.
> Keep the tone conversational and human.
>
> Example:
> (Looking to the context)
> User: ¿Cuánto pesa la Luna?
> Response: Unos 7.300 kg... más o menos

- If the user query was a task or command:
> Do not confirm task success directly.
> Use natural language that feels friendly and relaxed.
>
> Examples:
> "Listo... o eso parece."
> "Deberías tenerlo ya. Tú me dirás."
> "Con suerte, misión cumplida."
> "Todo apunta a que está hecho."
"""


class StartEndVoiceRouter:

    _logger = get_logger("StartEndVoiceRouter")

    def __init__(
        self,
        schema: DeploySchema,
        interpreter: Interpreter,
        model: str = "gpt-4.1-nano",
        start_fx: str = "slow_button_trimmed.mp3",
        end_fx: str = "mellow_echo_success.mp3",
        disable_fx: bool = False,
        disable: bool = False,
        **kwargs,
    ):
        self.disabled = disable
        fx_path = get_root_path() / "cognition_layer" / "tools" / "voice" / "fx"
        self.interpreter = interpreter
        self.llm = MutableChatLLM(model=model, max_tokens=200, temperature=1)
        self.router = SimpleRouter(schema)
        self.start_fx = (fx_path / start_fx).as_posix()
        self.end_fx = (fx_path / end_fx).as_posix()
        self.disabled_fx = disable_fx

        assert (
            len(self.router.agents) == 1
        ), "StartEndVoiceRouter only supports one agent."

        self.routered_agent = list(self.router.agents.keys())[0]

    def invoke(self, query: str) -> Generator[FastAPStep, None, None]:
        if not self.disabled_fx:
            play(self.start_fx)
        server = self.router.server(self.routered_agent, self.interpreter)

        if server is None:
            yield FastAPStep(
                name="FeedbackVoiceRouter",
                content="Couldnt route to an expert agent.",
                is_last=True,
            )
            return

        if not self.disabled:
            prompt = [
                SystemMessage(START_PROMPT),
                HumanMessage(query),
            ]
            feedback = self.llm.invoke(prompt)
            message = feedback.content
            self._logger.debug(f"Start Speech: {message}")
            speech = text2speech(message)
            play(speech)

        last_step = None
        for step in server.send_task(query):
            last_step = step
            if step.is_last:
                break
            yield step

        if not self.disabled:
            prompt = [
                SystemMessage(END_PROMPT),
                SystemMessage("context: " + last_step.content),
                HumanMessage(query),
            ]
            feedback = self.llm.invoke(prompt)
            message = feedback.content
            self._logger.debug(f"End Speech: {message}")
            speech = text2speech(message)
            play(speech)
            if not self.disabled_fx:
                play(self.end_fx)
        yield last_step
