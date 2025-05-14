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

LANGUAGE = "spanish"

PROMPT = f"""
Based on the following text, generate one single sentence in Spanish that simulates performing the described action.
Follow these strict rules:
- The sentence must be no longer than 10 words.
- Use a natural, spoken, and direct tone, as if you are talking to the user.
- Do not include any variables, internal functions, technical terms, or code-related keywords.
- Use correct {LANGUAGE} accents, punctuation, question marks and upper/lowercases,
as the sentence will be read out loud.
- The actions in the reasoning are in the past. The function contains the immediate next action (future).
- Use first person.

Return only one sentence per input.

Input text (example):
"
Spotify is not currently opened, but I have found the paths to its shortcut and
its setup file. I will attempt to open the Spotify application using the
shortcut found. After opening, I will check if it is already running instead of
installing it again. function: open_spotify()
"

Expected output example:
"Vamos a abrir Spotify..."

Input text (example 2):
"
Having opened the Spotify application, my next step is to check the list of
opened windows to determine if Spotify has successfully launched...
function: check_if_spotify_opened()
"

Expected output example:
"Revisaré si Spotify se ha abierto"

Other example phrases:
- "Justo como esperaba, continuemos"
- "Acabo de encontrar la ruta a la aplicación, ahora la abriré"
- "Debería revisar donde hacer click"
- "Estoy intentando reproducir la canción"
...
"""


class FeedbackVoiceRouter:

    _logger = get_logger("FeedbackVoiceRouter")

    def __init__(
        self,
        schema: DeploySchema,
        interpreter: Interpreter,
        model: str = "gpt-4.1-nano",
        **kwargs,
    ):
        self.interpreter = interpreter
        self.llm = MutableChatLLM(model="gpt-4.1-nano")
        self.router = SimpleRouter(schema)

        assert (
            len(self.router.agents) == 1
        ), "FeedbackVoiceRouter only supports one agent."

        self.routered_agent = list(self.router.agents.keys())[0]

    def invoke(self, query: str) -> Generator[FastAPStep, None, None]:
        server = self.router.server(self.routered_agent, self.interpreter)

        if server is None:
            yield FastAPStep(
                name="FeedbackVoiceRouter",
                content="Couldnt route to an expert agent.",
                is_last=True,
            )
            return

        for step in server.send_task(query):
            if step.is_last:
                yield step
                break

            prompt = [
                SystemMessage(PROMPT),
                HumanMessage(step.content),
            ]
            feedback = self.llm.invoke(prompt)
            message = feedback.content
            self._logger.debug(f"Speech: {message}")
            speech = text2speech(message)
            play(speech)
            yield step
