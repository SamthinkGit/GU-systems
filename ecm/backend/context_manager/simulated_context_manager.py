import threading
import time
from typing import Dict
from typing import Optional

from ecm.backend.state_manager import Button
from ecm.backend.state_manager import NovaAction
from ecm.backend.state_manager import NovaEvent
from ecm.backend.state_manager import NovaState
from ecm.backend.state_manager import StateManager


class SimulatedContextManager:
    def __init__(self, initial_state: NovaState = NovaState.HOME):
        self.state_manager = StateManager(initial_state=initial_state)
        self.extra: Dict[str, str] = {}
        self._state_changed = threading.Event()

    def update(
        self,
        button: Optional[str] = None,
        event: Optional[str] = None,
        extra_input: Optional[Dict[str, str]] = None,
        _extra_output: Optional[Dict[str, str]] = None,
    ) -> None:
        button_enum = None
        event_enum = None

        if button is not None:
            try:
                button_enum = Button(button)
            except ValueError:
                raise ValueError(f"Botón inválido: {button}")

        if event is not None:
            try:
                event_enum = NovaEvent(event)
            except ValueError:
                raise ValueError(f"Evento inválido: {event}")

        new_state, action = self.state_manager.next_state(
            button=button_enum, event=event_enum, extra=extra_input
        )

        match action:
            case NovaAction.START_ECM:
                print("ECM is started")
                self.thread = threading.Thread(target=self.work)
                self.thread.start()
            case NovaAction.PAUSE:
                print("ECM is paused (not implemented in mock)")
            case NovaAction.RESUME:
                print("ECM is resumed (not implemented in mock)")
            case NovaAction.SOFT_STOP:
                print("ECM is stopped")

        if new_state != NovaState.INVALID:
            self.state_manager.state = new_state
            self.extra = _extra_output if _extra_output is not None else {}
            self._state_changed.set()

    def wait_for_new_state(self) -> None:
        self._state_changed.wait()
        self._state_changed.clear()

    def work(self) -> None:
        for _ in range(3):
            time.sleep(1)
            print("[MOCK] Ecm is working...")

        self.update(
            event=NovaEvent.NOVA_FEEDBACK,
            _extra_output={"audio": "thisisnotarealfile.wav"},
        )

        for _ in range(3):
            time.sleep(1)
            print("[MOCK] Ecm is working...")

        self.update(
            event=NovaEvent.NOVA_FEEDBACK,
            _extra_output={"audio": "thisisnotarealfile.wav"},
        )

        for _ in range(3):
            time.sleep(1)
            print("[MOCK] Ecm is working...")

        self.update(
            event=NovaEvent.NOVA_FINISHED,
            _extra_output={"audio": "thisisnotarealfile.wav"},
        )
