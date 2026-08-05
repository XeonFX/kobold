"""MJPEG-over-HTTP view of what the camera sees.

Lives with the camera node rather than in the app container because only one
process can hold /dev/video11, and the camera node already does. The app's UI
points an <img> at this port; with network_mode: host they share an address.

WHY MJPEG AND NOT H.264
-----------------------
The MPP encoder does both and H.264 is dramatically smaller — measured on 60
distinct real frames at 1280x960:

    MJPEG q80    2.11 ms cpu/frame   56.9 kB   5.59 Mbit/s @12fps
    H.264 2Mbit  0.95 ms cpu/frame    7.1 kB   0.70 Mbit/s @12fps

But MJPEG displays in an <img> tag with no client machinery at all, whereas
H.264 needs WebRTC or MSE plus fragmented MP4. On a LAN, 5.59 Mbit/s costs
nothing and 2.5% of a core is noise. Switch to H.264 when the link becomes the
constraint -- remote access over a WAN -- not before.

WHY THE HTTP HANDLER NEVER TOUCHES A FRAME
------------------------------------------
An earlier attempt polled a shared buffer at 200 Hz from the request handler.
That churned the GIL badly enough to starve the capture thread, and with
drop=true every frame it missed was discarded: 2.8 fps against a 12 fps camera.
Here the handler blocks on a Condition until there genuinely is a new frame, so
it wakes exactly once per frame and does nothing in between.
"""

from __future__ import annotations

import socketserver
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

PAGE = b"""<!doctype html><meta charset=utf-8><title>kobold camera</title>
<style>
 body{margin:0;background:#0b0d10;color:#c9d1d9;font:14px/1.5 system-ui,sans-serif;
      display:grid;place-items:center;min-height:100vh}
 img{max-width:100%;height:auto;border-radius:8px;box-shadow:0 8px 32px #0008}
 p{opacity:.6;margin:.75rem 0 0}
</style>
<img src="/stream" alt="camera">
<p>kobold &middot; <span id=s>live</span></p>
"""


class _Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.0"
    server_version = "kobold-stream"

    def log_message(self, *args):
        pass                                    # never spam the ROS log

    def do_GET(self):
        src = self.server.source
        if self.path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(PAGE)))
            self.end_headers()
            self.wfile.write(PAGE)
            return

        if self.path == "/stats":
            body = src.stats().encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/plain")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path != "/stream":
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()

        seq = 0
        try:
            while True:
                jpeg, seq = src.wait_for_frame(seq, timeout=2.0)
                if jpeg is None:
                    continue                    # timed out; check the socket and retry
                self.wfile.write(
                    b"--frame\r\nContent-Type: image/jpeg\r\n"
                    + b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n"
                    + jpeg + b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass                                # browser navigated away


class _Server(socketserver.ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class FrameSource:
    """Latest-frame holder. Producers publish, consumers block until there is one."""

    def __init__(self):
        self._cond = threading.Condition()
        self._jpeg: bytes | None = None
        self._seq = 0
        self._count = 0
        self._bytes = 0

    def publish(self, jpeg: bytes) -> None:
        with self._cond:
            self._jpeg = jpeg
            self._seq += 1
            self._count += 1
            self._bytes += len(jpeg)
            self._cond.notify_all()

    def wait_for_frame(self, last_seq: int, timeout: float = 2.0):
        with self._cond:
            if self._seq == last_seq:
                self._cond.wait(timeout=timeout)
            if self._seq == last_seq:
                return None, last_seq
            return self._jpeg, self._seq

    def stats(self) -> str:
        with self._cond:
            return (f"frames={self._count}\nbytes={self._bytes}\n"
                    f"kb_per_frame={self._bytes / max(self._count, 1) / 1024:.1f}\n")


class StreamServer:
    """Owns the HTTP thread. Call publish() from the capture loop."""

    def __init__(self, port: int, bind: str = "0.0.0.0"):
        self.source = FrameSource()
        self._httpd = _Server((bind, port), _Handler)
        self._httpd.source = self.source
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()
        self.port = self._httpd.server_address[1]

    def publish(self, jpeg: bytes) -> None:
        self.source.publish(jpeg)

    def shutdown(self) -> None:
        self._httpd.shutdown()
        self._httpd.server_close()
