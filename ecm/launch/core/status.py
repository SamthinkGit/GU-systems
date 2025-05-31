from enum import Enum


class CoreStatus(Enum):
    INPUT_REQUIRED = "<input_required>"
    OBTAINING_INPUT = "<obtaining_input>"
    STARTING_CHAIN = "<starting_chain>"
    STEP_STARTED = "<step_started>"
    STEP_COMPLETED = "<step_completed>"
    EXITING_CHAIN = "<exiting_chain>"
