from typing import Dict

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from ecm.launch.core.engine import CoreConfig
from ecm.launch.core.engine import NovaCore
from ecm.shared import get_logger
from ecm.tools.persistent_config import PersistentConfig

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

config = CoreConfig(
    schema="nova",
    tts=True,
    stt=False,
    language="spanish",
    voice_api_enabled=True,
)

context_manager = NovaCore(config)
logger = get_logger("NovaAPI")


class ConfigRequest(BaseModel):
    config: Dict[str, str]


class ExtraRequest(BaseModel):
    extra: Dict[str, str]


class Request(BaseModel):
    request: str


# === Endpoints ===
@app.post("/config")
def config_endpoint(payload: ConfigRequest) -> bool:
    for k, v in payload.config.items():
        logger.debug(f"[API] Config received: {k} = {v}")
        PersistentConfig.set(k, v)
    return True


@app.post("/update")
def next_state() -> dict[str, object]:
    context_manager.wait_for_new_state()
    return {
        "update": dict(context_manager.extra),
    }


@app.post("/extra")
def extra(payload: ExtraRequest) -> bool:
    logger.debug(f"[API] Extra received: {payload.extra}")
    context_manager.update(extra_input=payload.extra)
    return True


@app.post("/request")
def request(payload: Request) -> bool:
    logger.debug(f"[API] Request received: {payload.request}")
    context_manager.update(
        input_request=payload.request,
    )
    return True
