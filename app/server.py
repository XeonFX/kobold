"""Kobold web app — the robot's only user interface.

Your phone supplies the microphone, speaker and screen. Nothing robot-specific
runs on it; it is a browser pointed at this server. That is how you get voice
before owning a mic, and it is why buying the mic array later changes nothing
above the `voice` container — the audio source moves, the pipeline does not.

Serves:
  /                     the PWA (static, no build step)
  /healthz              deploy health check
  /api/status           robot + deploy state
  /ws/telemetry         live robot state, ~10 Hz
  /ws/audio             mic in / speech out
  /api/targets          photo upload for object search
  /api/estop            the big red button
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

from fastapi import FastAPI, File, Form, HTTPException, UploadFile, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger("app")

STATIC_DIR = Path(__file__).parent / "static"
TARGETS_DIR = Path(os.environ.get("TARGETS_DIR", "/data/targets"))
DEPLOY_DIR = Path(os.environ.get("DEPLOY_DIR", "/data/deploy"))
PORT = int(os.environ.get("PORT", "8000"))


class RobotState:
    """Latest known robot state, refreshed from ROS 2.

    Deliberately a plain snapshot rather than a full ROS graph in this process:
    the app should keep serving the UI — including the e-stop — even when the
    ROS side is unhealthy. An interface that dies with the thing it monitors is
    worse than useless.
    """

    def __init__(self) -> None:
        self.connected = False
        self.battery_pct: float | None = None
        self.battery_v: float | None = None
        self.faults: str = "unknown"
        self.estopped = False
        self.ranges: dict[str, float] = {}
        self.mode = "floor"
        self.last_update = 0.0

    def snapshot(self) -> dict[str, Any]:
        return {
            "connected": self.connected,
            "battery_pct": self.battery_pct,
            "battery_v": self.battery_v,
            "faults": self.faults,
            "estopped": self.estopped,
            "ranges": self.ranges,
            "mode": self.mode,
            "stale": (time.time() - self.last_update) > 3.0,
        }


state = RobotState()


def deploy_info() -> dict[str, Any]:
    """What is actually running — read from the files deploy.sh writes."""
    def read(name: str) -> str | None:
        try:
            return (DEPLOY_DIR / name).read_text().strip()
        except OSError:
            return None

    return {
        "sha": read("current") or os.environ.get("KOBOLD_SHA", "dev"),
        "previous": read("previous"),
        "deployed_at": read("deployed_at"),
        "rolled_back_from": read("rolled_back_from"),
        # Written by the kobold-update-check timer, which NOTIFIES and never
        # installs. Surfacing it here is the whole point of that split: you
        # find out an update exists, and you decide when to apply it.
        "available": read("available"),
    }


@asynccontextmanager
async def lifespan(app: FastAPI):
    TARGETS_DIR.mkdir(parents=True, exist_ok=True)
    task = asyncio.create_task(ros_poller())
    log.info("kobold app on :%d  sha=%s", PORT, deploy_info()["sha"])
    yield
    task.cancel()


app = FastAPI(title="Kobold", lifespan=lifespan)


async def ros_poller() -> None:
    """Placeholder for the rclpy subscription loop (Phase 3+).

    Kept as its own task so the HTTP/WebSocket layer never blocks on ROS.
    """
    while True:
        await asyncio.sleep(1.0)


# ------------------------------------------------------------------ routes --

@app.get("/healthz")
async def healthz():
    """Used by deploy.sh. 'The container started' is not 'it works'."""
    return {"ok": True, "sha": deploy_info()["sha"]}


@app.get("/api/status")
async def status():
    return {"robot": state.snapshot(), "deploy": deploy_info()}


@app.post("/api/estop")
async def estop(engage: bool = Form(True)):
    """Software e-stop.

    Note this is the SLOW path — it goes through ROS 2 to the drive board. The
    fast paths are the cliff ISR and the sense board's hardware safety line,
    both under a millisecond and neither involving this process. If you need to
    stop the robot right now, the motor-rail kill switch is the real answer.
    """
    state.estopped = engage
    log.warning("e-stop %s via app", "ENGAGED" if engage else "cleared")
    return {"estopped": engage}


@app.post("/api/targets")
async def add_target(
    file: UploadFile = File(...),
    label: str = Form(""),
):
    """Register a photo as a search target.

    The image is embedded once by the perception container (SigLIP) and stored;
    finding it later is cosine similarity against YOLO crops, with the VLM
    called only to verify candidates.
    """
    if not (file.content_type or "").startswith("image/"):
        raise HTTPException(400, "expected an image")

    safe = "".join(c for c in (label or "target") if c.isalnum() or c in "-_")[:40] or "target"
    dest = TARGETS_DIR / f"{int(time.time())}_{safe}.jpg"
    dest.write_bytes(await file.read())

    log.info("target registered: %s (%s)", dest.name, label)
    return {"id": dest.stem, "label": label, "path": str(dest)}


@app.get("/api/targets")
async def list_targets():
    items = sorted(TARGETS_DIR.glob("*.jpg"), reverse=True)
    return [{"id": p.stem, "url": f"/api/targets/{p.name}"} for p in items[:50]]


@app.get("/api/targets/{name}")
async def get_target(name: str):
    path = (TARGETS_DIR / name).resolve()
    if not str(path).startswith(str(TARGETS_DIR.resolve())) or not path.exists():
        raise HTTPException(404)
    return FileResponse(path)


@app.websocket("/ws/telemetry")
async def ws_telemetry(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            d = deploy_info()
            await ws.send_json({
                "robot": state.snapshot(),
                "deploy": d,
                "update": d.get("available"),
            })
            await asyncio.sleep(0.1)          # 10 Hz is plenty for a human
    except WebSocketDisconnect:
        pass


@app.websocket("/ws/audio")
async def ws_audio(ws: WebSocket):
    """Browser mic in, synthesised speech out.

    Binary frames are Opus from MediaRecorder; JSON frames are control. The
    voice container does VAD -> whisper -> agent -> Piper and returns audio.
    """
    await ws.accept()
    try:
        while True:
            msg = await ws.receive()
            if "bytes" in msg and msg["bytes"]:
                pass          # -> voice container (Phase 6)
            elif "text" in msg and msg["text"]:
                data = json.loads(msg["text"])
                if data.get("type") == "ping":
                    await ws.send_json({"type": "pong"})
    except WebSocketDisconnect:
        pass


@app.websocket("/ws/cmd")
async def ws_cmd(ws: WebSocket):
    """Teleop joystick.

    The firmware stops the motors after 300 ms without a command, so a dropped
    WebSocket means the robot stops on its own. That is deliberate: the failure
    mode of a lost connection should be 'stationary', never 'last command
    forever'.
    """
    await ws.accept()
    try:
        while True:
            data = json.loads(await ws.receive_text())
            if data.get("type") == "cmd_vel":
                pass          # -> bridge (Phase 1)
    except WebSocketDisconnect:
        log.info("teleop disconnected — firmware watchdog will stop the motors")


if STATIC_DIR.exists():
    app.mount("/", StaticFiles(directory=STATIC_DIR, html=True), name="static")
else:
    @app.get("/")
    async def missing():
        return JSONResponse({"error": "static/ not mounted"}, status_code=500)


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=PORT, log_level="info")
