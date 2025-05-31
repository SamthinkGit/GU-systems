from typing import Dict

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from ecm.backend.context_manager.simulated_context_manager import NovaState
from ecm.backend.context_manager.simulated_context_manager import SimulatedContextManager

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

context_manager = SimulatedContextManager(initial_state=NovaState.HOME)


class ConfigRequest(BaseModel):
    config: Dict[str, str]


class ExtraRequest(BaseModel):
    extra: Dict[str, str]


class NextStateRequest(BaseModel):
    state: str


class ButtonRequest(BaseModel):
    button: str


# === Endpoints ===
@app.post("/config")
def config_endpoint(payload: ConfigRequest) -> bool:
    for k, v in payload.config.items():
        print(f"[API] Config received: {k} = {v}")
    return True


@app.post("/next_state")
def next_state(payload: NextStateRequest) -> dict[str, object]:
    is_match = context_manager.state_manager.state.value == payload.state
    if not is_match:
        print(
            f"[!WARNING!] Expected state {payload.state}, but current state is {context_manager.state_manager.state.value}"  # noqa
        )

    context_manager.wait_for_new_state()
    return {
        "next_state": context_manager.state_manager.state,
        "extra": context_manager.extra,
    }


@app.post("/extra")
def extra(payload: ExtraRequest) -> bool:
    print(f"[API] Extra received: {payload.extra}")
    context_manager.update(extra_input=payload.extra)
    return True


@app.post("/button")
def button(payload: ButtonRequest) -> bool:
    print(f"[API] Button received: {payload.button}")
    context_manager.update(button=payload.button)
    return True
