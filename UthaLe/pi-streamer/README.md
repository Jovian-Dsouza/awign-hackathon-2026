# UthaLe Pi streamer

Streams a Raspberry Pi camera to the UthaLe dashboard running on your PC, through an ngrok tunnel.

```
Raspberry Pi                          internet                 Your PC
stream_camera.py ── wss (JPEG frames) ──► ngrok ──► UthaLe backend :8000 ──► dashboard
```

The Pi connects *out* to the PC's public ngrok URL, so the Pi needs no port forwarding and can be on any network. Frames go over one WebSocket instead of one HTTP request per frame. That keeps latency low and stays inside the request limits of ngrok's free plan.

## 1. On the PC

1. Pick a long random token, e.g. `python -c "import secrets; print(secrets.token_urlsafe(24))"`.
2. Add these lines to `backend/.env` (or `UthaLe/.env`):
   ```
   CAMERA_SOURCE=remote
   CAMERA_INGEST_TOKEN=<your token>
   ```
3. Start the backend as usual (`python -m app` from `backend/`, port 8000).
4. Start the tunnel: `ngrok http 8000`. Copy the `https://….ngrok-free.app` forwarding URL.
   The free URL changes every time ngrok restarts. To keep one address, claim the free static domain in the ngrok dashboard and run `ngrok http --url=<your-domain> 8000`.

The Live Robot camera panel shows **Connecting** until the Pi connects.

## 2. On the Raspberry Pi

Tested target: Raspberry Pi OS Bookworm with a Camera Module (CSI) or a USB webcam.

```bash
sudo apt update && sudo apt install -y python3-picamera2 python3-opencv
python3 -m venv --system-site-packages ~/uthale-venv   # sees apt's picamera2 + OpenCV
~/uthale-venv/bin/pip install -r requirements.txt

~/uthale-venv/bin/python stream_camera.py \
  --server https://<your-ngrok-domain> \
  --token <your token>
```

The script logs its frame rate and bandwidth every 10 s. Ctrl+C stops it.

| Option | Default | |
|---|---|---|
| `--camera` | `auto` | `picamera` (Camera Module), `usb`, or `auto` (Camera Module if found, else USB) |
| `--usb-index` | `0` | `/dev/videoN` for a USB webcam |
| `--width` / `--height` | `1280` / `720` | Requested resolution |
| `--fps` | `10` | Frames sent per second |
| `--quality` | `70` | JPEG quality. Lower it if the feed lags |
| `--rotate` | `0` | `90`, `180` or `270` for a mounted camera |
| `--label` | `UthaLe PI-CAM` | Text burned into the frame with a timestamp (`--no-overlay` to disable) |

`--server` and `--token` can also come from the `UTHALE_SERVER` and `UTHALE_INGEST_TOKEN` environment variables.

### Run on boot (optional)

`/etc/systemd/system/uthale-cam.service`:

```ini
[Unit]
Description=UthaLe Pi camera streamer
After=network-online.target
Wants=network-online.target

[Service]
User=pi
Environment=UTHALE_SERVER=https://<your-ngrok-domain>
Environment=UTHALE_INGEST_TOKEN=<your token>
ExecStart=/home/pi/uthale-venv/bin/python /home/pi/pi-streamer/stream_camera.py
Restart=always

[Install]
WantedBy=multi-user.target
```

Then run `sudo systemctl enable --now uthale-cam` and watch the logs with `journalctl -u uthale-cam -f`. Change `User` and the paths to match your Pi.

## Behaviour

- **Reconnects by itself.** If Wi-Fi drops, ngrok restarts or the backend restarts, the script retries with backoff (1, 2, 4, 8, then every 10 s). The camera stays open in the meantime.
- **Dashboard Stop/Start.** Stopping the camera on the dashboard disconnects the Pi. The Pi keeps retrying and resumes once you press Start.
- **One sender at a time.** A second Pi is refused while one is connected.
- **Security.** The token is required. The backend refuses remote frames if `CAMERA_INGEST_TOKEN` is empty or wrong, and it decodes every frame so only real JPEG images reach viewers and QC evidence. Anyone with the ngrok URL can still open the dashboard itself, so don't share the URL publicly.
- **Latency.** Expect roughly 0.3–1 s through ngrok, depending on the Pi's upload speed. If it grows, lower `--fps`, `--quality` or the resolution.

## Troubleshooting

| Pi log says | Fix |
|---|---|
| `HTTP 403` | Token mismatch, or the backend isn't running with `CAMERA_SOURCE=remote`. Restart the backend after editing `.env`. |
| `HTTP 404` / `Could not reach backend` | ngrok isn't running, the URL changed, or ngrok points at the wrong port. |
| `Camera capture is stopped on the dashboard` | Press **Start** on the Live Robot camera panel. |
| `No Pi camera module detected` | Reseat the ribbon cable and check it with `rpicam-hello`. Or use `--camera usb`. |
| `Could not open USB camera` | List devices with `v4l2-ctl --list-devices` and pass the right `--usb-index`. |
