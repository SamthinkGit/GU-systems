import asyncio
import json
from typing import List

from fastapi import FastAPI
from fastapi import HTTPException
from fastapi import Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from fastapi.responses import StreamingResponse

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

topic_queues: List[asyncio.Queue] = []


@app.post("/steps")
async def publish_step(request: Request):
    """
    Endpoint to publish a FastAPStep to all subscribers.
    Expects JSON with fields: name, step_name, content, is_last
    """
    try:
        data = await request.json()
        for field in ("name", "step_name", "content", "is_last"):
            if field not in data:
                raise ValueError(f"Missing field: {field}")
        message = json.dumps(data)

    except (ValueError, json.JSONDecodeError) as e:
        raise HTTPException(status_code=400, detail=str(e))

    for queue in topic_queues:
        try:
            queue.put_nowait(message)

        except asyncio.QueueFull:
            pass

    return JSONResponse({"status": "published"})


@app.get("/steps/stream")
async def stream_steps():
    """
    Endpoint for clients to stream FastAPStep events as Server-Sent Events (SSE).
    """
    queue: asyncio.Queue = asyncio.Queue(maxsize=100)
    topic_queues.append(queue)

    async def event_generator():
        try:
            while True:
                message = await queue.get()
                yield f"data: {message}\n\n"
                queue.task_done()
        except asyncio.CancelledError:
            return

    return StreamingResponse(event_generator(), media_type="text/event-stream")


@app.get("/health")
async def health_check():
    return {"status": "ok"}
