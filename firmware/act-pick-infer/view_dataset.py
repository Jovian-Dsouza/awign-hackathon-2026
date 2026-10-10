# -*- coding: utf-8 -*-
"""Browse recorded LeRobot wrist videos in the browser.

Official one-episode tool (Rerun / Foxglove)::

    lerobot-dataset-viz \\
      --repo-id Jovian-Dsouza/so101_pick_wrist_finetune-2 \\
      --episode-index 0

This script lists *all* episodes and plays the wrist MP4 segment for each
one. Videos live under ``$HF_LEROBOT_HOME/<repo_id>/videos/`` (chunked files;
each episode is a time range inside a chunk).

    conda activate lerobot
    python act-pick-infer/view_dataset.py \\
      --repo-id Jovian-Dsouza/so101_pick_wrist_finetune-2

Open http://127.0.0.1:8006
"""
from __future__ import annotations

import argparse
import json
import mimetypes
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, unquote, urlparse

import pyarrow.parquet as pq

from lerobot.utils.constants import HF_LEROBOT_HOME

VIEW_PORT = 8006
VIDEO_KEY = "observation.images.wrist"


def _urls(port: int) -> list[str]:
    urls = [f"http://127.0.0.1:{port}"]
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if not ip.startswith("127."):
                urls.append(f"http://{ip}:{port}")
    except OSError:
        pass
    seen: set[str] = set()
    out: list[str] = []
    for url in urls:
        if url not in seen:
            seen.add(url)
            out.append(url)
    return out


def resolve_root(repo_id: str, root: str | None) -> Path:
    if root:
        path = Path(root).expanduser().resolve()
    else:
        path = Path(HF_LEROBOT_HOME) / repo_id
    if not (path / "meta" / "info.json").is_file():
        raise SystemExit(f"No dataset at {path} (missing meta/info.json)")
    return path


def load_episodes(root: Path) -> list[dict]:
    info = json.loads((root / "meta" / "info.json").read_text())
    fps = float(info.get("fps") or 30)
    tasks_path = root / "meta" / "tasks.parquet"
    task_by_index: dict[int, str] = {}
    if tasks_path.is_file():
        tdf = pq.read_table(tasks_path).to_pandas()
        # index is the task string when written as in record.py
        if "task_index" in tdf.columns and tdf.index.name == "task":
            for task_str, row in tdf.iterrows():
                task_by_index[int(row["task_index"])] = str(task_str)
        elif "task" in tdf.columns and "task_index" in tdf.columns:
            for _, row in tdf.iterrows():
                task_by_index[int(row["task_index"])] = str(row["task"])

    rows: list[dict] = []
    for parquet in sorted((root / "meta" / "episodes").rglob("*.parquet")):
        table = pq.read_table(parquet)
        df = table.to_pandas()
        for _, ep in df.iterrows():
            ep_idx = int(ep["episode_index"])
            length = int(ep["length"])
            chunk = int(ep[f"videos/{VIDEO_KEY}/chunk_index"])
            file_i = int(ep[f"videos/{VIDEO_KEY}/file_index"])
            t0 = float(ep[f"videos/{VIDEO_KEY}/from_timestamp"])
            t1 = float(ep[f"videos/{VIDEO_KEY}/to_timestamp"])
            rel = f"videos/{VIDEO_KEY}/chunk-{chunk:03d}/file-{file_i:03d}.mp4"
            abs_path = root / rel
            task = ""
            if "tasks" in ep and ep["tasks"] is not None:
                raw = ep["tasks"]
                if hasattr(raw, "__iter__") and not isinstance(raw, (str, bytes)):
                    task = ", ".join(str(x) for x in raw)
                else:
                    task = str(raw)
            if not task and task_by_index:
                task = next(iter(task_by_index.values()), "")
            rows.append(
                {
                    "episode_index": ep_idx,
                    "length": length,
                    "seconds": round(length / fps, 1),
                    "from_timestamp": t0,
                    "to_timestamp": t1,
                    "video_rel": rel,
                    "video_exists": abs_path.is_file(),
                    "task": task,
                }
            )
    rows.sort(key=lambda r: r["episode_index"])
    return rows


HTML = """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>Dataset videos</title>
<style>
body{font-family:sans-serif;background:#1e1e2e;color:#cdd6f4;margin:0}
main{display:flex;gap:20px;padding:20px;flex-wrap:wrap;align-items:flex-start}
h1{font-size:1.15em;margin:0 0 8px}
.dim{color:#a6adc8;font-size:13px;line-height:1.4}
#list{width:280px;max-height:80vh;overflow:auto}
.ep{display:block;width:100%;text-align:left;padding:10px 12px;margin:0 0 6px;
  border:none;border-radius:6px;background:#313244;color:#cdd6f4;cursor:pointer}
.ep:hover{background:#45475a}
.ep.on{background:#89b4fa;color:#1e1e2e}
.ep .meta{font-family:monospace;font-size:12px;opacity:.85}
video{width:640px;max-width:100%;background:#111;border-radius:8px}
#info{font-family:monospace;margin:10px 0;min-height:1.4em}
</style></head><body>
<main>
<div>
<h1 id="title">Dataset</h1>
<p class="dim" id="summary"></p>
<div id="list"></div>
</div>
<div>
<video id="player" controls playsinline></video>
<div id="info">Select an episode</div>
<p class="dim">Playback seeks to each episode’s time range inside the chunked MP4.
Official per-episode tool: <code>lerobot-dataset-viz --repo-id … --episode-index N</code></p>
</div>
</main>
<script>
const episodes=__EPISODES__;
const repo=__REPO__;
document.getElementById('title').textContent=repo;
document.getElementById('summary').textContent=
  episodes.length+' episodes · wrist camera';
const list=document.getElementById('list');
const player=document.getElementById('player');
const info=document.getElementById('info');
let active=-1;
let endT=null;

episodes.forEach((ep,i)=>{
  const b=document.createElement('button');
  b.className='ep';
  b.innerHTML=`<div>episode ${ep.episode_index}</div>
    <div class="meta">${ep.seconds}s · ${ep.length} frames</div>
    <div class="meta">${ep.task||''}</div>`;
  b.onclick=()=>play(i);
  list.appendChild(b);
});

function play(i){
  const ep=episodes[i];
  active=i;
  [...list.children].forEach((el,j)=>el.classList.toggle('on',j===i));
  if(!ep.video_exists){
    info.textContent='Missing video file: '+ep.video_rel;
    player.removeAttribute('src');
    return;
  }
  endT=ep.to_timestamp;
  const url='/video?path='+encodeURIComponent(ep.video_rel)+'#t='+ep.from_timestamp;
  player.src=url;
  player.load();
  player.play().catch(()=>{});
  info.textContent=
    `episode ${ep.episode_index}  ·  ${ep.seconds}s  ·  t=${ep.from_timestamp.toFixed(2)}–${ep.to_timestamp.toFixed(2)}s  ·  ${ep.video_rel}`;
}
player.addEventListener('timeupdate',()=>{
  if(endT!=null && player.currentTime>=endT-0.04){
    player.pause();
  }
});
player.addEventListener('loadedmetadata',()=>{
  if(active>=0){
    const ep=episodes[active];
    if(Math.abs(player.currentTime-ep.from_timestamp)>0.15){
      player.currentTime=ep.from_timestamp;
    }
  }
});
if(episodes.length) play(0);
</script>
</body></html>"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-id", required=True, help="e.g. Jovian-Dsouza/so101_pick_wrist_finetune-2")
    parser.add_argument("--root", default=None, help="Local dataset root (default: $HF_LEROBOT_HOME/repo_id)")
    parser.add_argument("--port", type=int, default=VIEW_PORT)
    args = parser.parse_args()

    root = resolve_root(args.repo_id, args.root)
    episodes = load_episodes(root)
    print(f"Dataset {args.repo_id}")
    print(f"  root {root}")
    print(f"  episodes {len(episodes)}")
    for ep in episodes:
        mark = "ok" if ep["video_exists"] else "MISSING"
        print(f"  [{mark}] ep {ep['episode_index']}: {ep['seconds']}s  {ep['video_rel']}")

    page = (
        HTML.replace("__EPISODES__", json.dumps(episodes))
        .replace("__REPO__", json.dumps(args.repo_id))
        .encode()
    )

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            parsed = urlparse(self.path)
            if parsed.path == "/api/episodes":
                body = json.dumps(episodes).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(body)
                return
            if parsed.path == "/video":
                qs = parse_qs(parsed.query)
                rel = unquote((qs.get("path") or [""])[0])
                if ".." in rel or rel.startswith("/"):
                    self.send_error(400)
                    return
                path = (root / rel).resolve()
                if not str(path).startswith(str(root.resolve())) or not path.is_file():
                    self.send_error(404)
                    return
                size = path.stat().st_size
                ctype = mimetypes.guess_type(str(path))[0] or "video/mp4"
                range_hdr = self.headers.get("Range")
                if range_hdr and range_hdr.startswith("bytes="):
                    start_s, _, end_s = range_hdr.replace("bytes=", "", 1).partition("-")
                    start = int(start_s) if start_s else 0
                    end = int(end_s) if end_s else size - 1
                    end = min(end, size - 1)
                    length = end - start + 1
                    self.send_response(206)
                    self.send_header("Content-Type", ctype)
                    self.send_header("Content-Length", str(length))
                    self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
                    self.send_header("Accept-Ranges", "bytes")
                    self.end_headers()
                    with path.open("rb") as f:
                        f.seek(start)
                        remaining = length
                        while remaining > 0:
                            chunk = f.read(min(64 * 1024, remaining))
                            if not chunk:
                                break
                            self.wfile.write(chunk)
                            remaining -= len(chunk)
                    return
                self.send_response(200)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(size))
                self.send_header("Accept-Ranges", "bytes")
                self.end_headers()
                with path.open("rb") as f:
                    while True:
                        chunk = f.read(64 * 1024)
                        if not chunk:
                            break
                        self.wfile.write(chunk)
                return
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(page)

    class Server(ThreadingHTTPServer):
        allow_reuse_address = True

    try:
        server = Server(("0.0.0.0", args.port), Handler)
    except OSError as e:
        if getattr(e, "errno", None) == 98:
            print(f"Port {args.port} in use. Pass --port N")
        raise
    print("Video browser:")
    for url in _urls(args.port):
        print(f"  {url}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
