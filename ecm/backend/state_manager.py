from enum import Enum
from typing import Callable


class NovaState(Enum):
    HOME = "<home>"
    LISTENING = "<listening>"
    INITIAL_LOADING = "<initial_loading>"
    LOADING = "<loading>"
    SPEECH = "<speech>"
    PAUSED = "<paused>"
    INVALID = "<invalid>"


class NovaAction(Enum):
    NONE = "<none>"
    START_STT = "<start_stt>"
    FINISH_STT = "<finish_stt>"
    START_TTS = "<start_tts>"
    PAUSE = "<pause>"
    RESUME = "<resume>"
    SOFT_STOP = "<soft_stop>"
    HARD_STOP = "<hard_stop>"


class NovaEvent(Enum):
    NOVA_FEEDBACK = "<nova_feedback>"
    NOVA_FINISHED = "<nova_finished>"
    LOADING_FINISHED = "<loading_finished>"
    FINISHED_TTS = "<finished_tts>"


class Button(Enum):
    NOVA = "<nova_button>"
    RESUME = "<resume_button>"
    EXIT = "<exit_button>"


def autoinvalidate(func: Callable):
    def wrapper(*args, **kwargs):
        result = func(*args, **kwargs)
        if result is None:
            return NovaState.INVALID, NovaAction.NONE
        return result

    return wrapper


_states = {}


def state(state: NovaState):

    @autoinvalidate
    def decorator(func):
        _states[state] = func
        return func

    return decorator


class StateManager:

    def __init__(self, initial_state: NovaState = NovaState.INITIAL_LOADING):
        self.state = initial_state

    def next_state(
        self, button: Button = None, event: NovaEvent = None
    ) -> tuple[NovaState, NovaAction]:

        if self.state not in _states:
            return NovaState.INVALID, NovaAction.NONE
        next_state_func = _states[self.state]

        return next_state_func(button, event)

    @state(NovaState.INITIAL_LOADING)
    def _next_state_from_initial_loading(
        self, button: Button = None, event: NovaEvent = None
    ) -> tuple[NovaState, NovaAction]:
        if event == NovaEvent.LOADING_FINISHED:
            return NovaState.HOME, NovaAction.NONE

        return NovaState.INVALID, NovaAction.NONE

    @state(NovaState.HOME)
    def _next_state_from_home(
        self, button: Button = None, event: NovaEvent = None
    ) -> tuple[NovaState, NovaAction]:
        if button == Button.NOVA:
            return NovaState.LISTENING, NovaAction.START_STT

    @state(NovaState.LISTENING)
    def _next_state_from_listening(
        self, button: Button = None, event: NovaEvent = None
    ) -> tuple[NovaState, NovaAction]:

        if button == Button.NOVA:
            return NovaState.LOADING, NovaAction.FINISH_STT

    @state(NovaState.LOADING)
    def _next_state_from_loading(
        self, button: Button = None, event: NovaEvent = None
    ) -> tuple[NovaState, NovaAction]:
        if button == Button.NOVA:
            return NovaState.PAUSED, NovaAction.PAUSE

        if event == NovaEvent.NOVA_FEEDBACK:
            return NovaState.SPEECH, NovaAction.NONE

        if event == NovaEvent.NOVA_FINISHED:
            return NovaState.HOME, NovaAction.NONE

    @state(NovaState.SPEECH)
    def _next_state_from_speech(
        self, button: Button = None, event: NovaEvent = None
    ) -> tuple[NovaState, NovaAction]:
        if event == NovaEvent.FINISHED_TTS:
            return NovaState.LOADING, NovaAction.NONE

        if button == Button.NOVA:
            return NovaState.PAUSED, NovaAction.PAUSE

    @state(NovaState.PAUSED)
    def _next_state_from_paused(
        self, button: Button = None, event: NovaEvent = None
    ) -> tuple[NovaState, NovaAction]:
        if button == Button.RESUME:
            return NovaState.LOADING, NovaAction.RESUME

        if button == Button.EXIT:
            return NovaState.HOME, NovaAction.SOFT_STOP


# Endpoints
# /next_state
# /button
# /config
