# camera-agent

A tiny, standalone streamer. It captures a camera (USB, RTSP, or a video file) and pushes
JPEG frames to a monitoring server over a WebSocket. It has no dependency on the rest of
this repo, and only needs OpenCV and `websockets`, so it runs on whatever device the
camera happens to be plugged into.

```
 any device (laptop, mini PC, Pi)                          the server machine
 ┌──────────────────────────┐   wss, one JPEG per message  ┌───────────────────────┐
 │ camera ─► camera-agent   ├─────────────────────────────►│ Overwatch /api/ingest │
 └──────────────────────────┘   (connects OUT: no port     │  or UthaLe relay      │
                                 forwarding; ngrok works)  └───────────────────────┘
```

Because the agent connects **out** to the server, the device can be behind any NAT. To
reach a server on another network, give it that server's ngrok URL.

## Run

```bash
cd camera-agent
uv sync
uv run camera-agent --list-cameras                  # what can it see?
uv run camera-agent --source "name:HD camera" --dry-run 10   # check size/bandwidth, sends nothing

# same machine as the server
uv run camera-agent --server http://127.0.0.1:8010 --token <token> --source "name:HD camera"

# server reached through ngrok (ngrok http 8010)
uv run camera-agent --server https://<your>.ngrok-free.app --token <token> --source "name:HD camera"

# server reached through the UthaLe backend (ngrok http 8000, the same tunnel the dashboard uses)
uv run camera-agent --server https://<your>.ngrok-free.app --token <token> --via-uthale --source "name:HD camera"
```

`--server` / `--token` / `--source` can also come from `CAMERA_AGENT_SERVER`,
`CAMERA_AGENT_TOKEN`, `CAMERA_AGENT_SOURCE`.

| Option | Default | |
|---|---|---|
| `--source` | *(required)* | `name:<exact device name>`, `index:N`, `rtsp://…`, or a video file (looped). No default, so the laptop's own camera is never streamed by accident |
| `--fps` | 5 | frames sent per second |
| `--max-width` | 960 | downscale before sending (0 = keep). Detection works at 320 px, so more is wasted bandwidth |
| `--quality` | 75 | JPEG quality |
| `--rotate` | 0 | 90 / 180 / 270 for a mounted camera |
| `--path` | `/api/ingest` | WebSocket path; `--via-uthale` = `/api/monitor/ingest` |

## Behaviour

- **Always the newest frame.** The camera is read continuously and only the latest frame
  is sent, so a slow network never makes the server analyse old footage.
- **Selects the camera by exact name**, not index. On macOS the numbering differs between tools and
  changes between runs (a test run once opened the built-in camera by index). A name that matches
  nothing or two cameras is refused.
- **Reconnects by itself** (1, 2, 4, 8, then every 10 s), and reopens the camera if it
  was unplugged.
- **Logs bandwidth every 10 s**, including MB/hour, so you can check it against your
  tunnel's data limits before relying on it. `--dry-run N` measures it without a server.
- **Keep the settings stable.** The server stores its reference image at the size it
  receives. If you change `--max-width`/`--width`, capture a new reference.
- **One sender at a time.** A second agent is refused while one is connected.
- **Security.** The token is mandatory; the server refuses unauthenticated handshakes
  before accepting. Anyone who has the ngrok URL can still reach the server's pages, so
  do not share it publicly. The video is not end-to-end encrypted beyond TLS to the tunnel.

## Troubleshooting

| Log says | Fix |
|---|---|
| `server refused the connection (HTTP 403)` | Token mismatch, or the server is not in remote mode (`OVERWATCH_SOURCE=remote` and `OVERWATCH_INGEST_TOKEN` set) |
| `HTTP 404` | Wrong path or the tunnel points at the wrong port (Overwatch 8010 vs UthaLe 8000); try `--via-uthale` |
| `could not open …` | Cable, another app holding the camera, or macOS camera permission for this terminal |
| `No camera matching …` / `matches several cameras` | Run `--list-cameras` and use the full exact name. A missing camera is refused, never replaced by another |
