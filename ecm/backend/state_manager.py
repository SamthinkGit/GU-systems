from enum import Enum
from typing import Callable


class NovaState(Enum):

    INITIAL_LOADING = "<initial_loading>"

    HOME = "<home>"
    LISTENING = "<listening>"
    WAITING_FOR_AUDIO = "<waiting_for_audio>"
    LOADING = "<loading>"
    SPEECH = "<speech>"
    PAUSED = "<paused>"
    INVALID = "<invalid>"


class NovaAction(Enum):
    NONE = "<none>"
    START_ECM = "<start_ecm>"
    PAUSE = "<pause>"
    RESUME = "<resume>"
    SOFT_STOP = "<soft_stop>"
    HARD_STOP = "<hard_stop>"


class NovaEvent(Enum):
    NOVA_FEEDBACK = "<nova_feedback>"
    NOVA_FINISHED = "<nova_finished>"
    LOADING_FINISHED = "<loading_finished>"


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

    def decorator(func):
        _states[state] = func
        return func

    return decorator


class StateManager:

    def __init__(self, initial_state: NovaState = NovaState.HOME):
        self.state = initial_state

    def next_state(
        self, button: Button = None, event: NovaEvent = None, extra: dict = None
    ) -> tuple[NovaState, NovaAction]:

        next_state_func = _states[self.state]
        return next_state_func(button, event, extra)


@state(NovaState.INITIAL_LOADING)
@autoinvalidate
def _next_state_from_initial_loading(
    button: Button = None, event: NovaEvent = None, extra: dict = None
) -> tuple[NovaState, NovaAction]:
    if event == NovaEvent.LOADING_FINISHED:
        return NovaState.HOME, NovaAction.NONE

    return NovaState.INVALID, NovaAction.NONE


@state(NovaState.HOME)
@autoinvalidate
def _next_state_from_home(
    button: Button = None, event: NovaEvent = None, extra: dict = None
) -> tuple[NovaState, NovaAction]:
    if button == Button.NOVA:
        return NovaState.LISTENING, NovaAction.NONE


@state(NovaState.LISTENING)
@autoinvalidate
def _next_state_from_listening(
    button: Button = None, event: NovaEvent = None, extra: dict = None
) -> tuple[NovaState, NovaAction]:

    if button == Button.NOVA:
        return NovaState.WAITING_FOR_AUDIO, NovaAction.NONE


@state(NovaState.WAITING_FOR_AUDIO)
@autoinvalidate
def _next_state_from_waiting_for_audio(
    button: Button = None, event: NovaEvent = None, extra: dict = None
) -> tuple[NovaState, NovaAction]:
    return NovaState.LOADING, NovaAction.START_ECM


@state(NovaState.LOADING)
@autoinvalidate
def _next_state_from_loading(
    button: Button = None, event: NovaEvent = None, extra: dict = None
) -> tuple[NovaState, NovaAction]:
    if button == Button.NOVA:
        return NovaState.PAUSED, NovaAction.PAUSE

    if event == NovaEvent.NOVA_FEEDBACK:
        return NovaState.SPEECH, NovaAction.NONE

    if event == NovaEvent.NOVA_FINISHED:
        return NovaState.HOME, NovaAction.NONE


@state(NovaState.SPEECH)
@autoinvalidate
def _next_state_from_speech(
    button: Button = None, event: NovaEvent = None, extra: dict = None
) -> tuple[NovaState, NovaAction]:

    if button == Button.NOVA:
        return NovaState.PAUSED, NovaAction.PAUSE

    return NovaState.LOADING, NovaAction.NONE


@state(NovaState.PAUSED)
@autoinvalidate
def _next_state_from_paused(
    button: Button = None, event: NovaEvent = None, extra: dict = None
) -> tuple[NovaState, NovaAction]:
    if button == Button.RESUME:
        return NovaState.LOADING, NovaAction.RESUME

    if button == Button.EXIT:
        return NovaState.HOME, NovaAction.SOFT_STOP


# Endpoints
# /next_state
# /button
# /config
