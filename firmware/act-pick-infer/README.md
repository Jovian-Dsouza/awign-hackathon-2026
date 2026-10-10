# ACT pick — record, fine-tune, infer (SO101 wrist)

Local tooling for running and fine-tuning the Hugging Face ACT policy
[TakuyaHiraoka/act_so101_pick_diverse_objects](https://huggingface.co/TakuyaHiraoka/act_so101_pick_diverse_objects)
on this Raspberry Pi + SO101 follower setup.

```text
record.py (Pi)  →  private HF dataset  →  finetune_colab.py (GPU)
                                              ↓
                                    private HF policy
                                              ↓
                                    infer.py --policy (Pi)
```

| Script | Role | Default port |
|--------|------|--------------|
| [`infer.py`](infer.py) | Run a policy on the arm (base or fine-tuned) | **8004** |
| [`record.py`](record.py) | Collect demos → LeRobot dataset → private Hub | **8005** |
| [`view_dataset.py`](view_dataset.py) | Browse recorded wrist episode videos | **8006** |
| [`finetune_colab.py`](finetune_colab.py) | Fine-tune ACT on Colab / a CUDA machine | — |

---

## Hardware (this Pi)

| Device | Path / id | Notes |
|--------|-----------|--------|
| SO101 follower | `/dev/ttyACM1`, id `so_follower` | Same calibration as `so101-arm/web_arm.py` |
| Mecanum Arduino | `/dev/ttyACM0` | Not used by these scripts |
| Wrist camera | LG Smart Cam → `/dev/video0` | 640×480 @ 30 fps, MJPG |
| Compute | CPU only | No CUDA on the Pi; inference is slow on the first ACT chunk |

Policy I/O (must match training data):

- `observation.images.wrist` — RGB 480×640×3
- `observation.state` / `action` — 6 joints in order:  
  `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`, `gripper`

---

## Environment

```bash
conda activate lerobot
cd /path/to/awign-hackathon-2026/firmware   # or wherever this folder lives
hf auth login   # once; write token for private push
```

Only one process should own `/dev/ttyACM1` and `/dev/video0` at a time (stop `web_arm`, `web_combined`, other infer/record sessions first).

---

## 1. Inference — `infer.py`

Loads ACT on CPU, streams the wrist view, and optionally drives the arm.

```bash
# Preview only (arm does not move). Open http://127.0.0.1:8004
python act-pick-infer/infer.py --check

# Drive the arm until Ctrl-C
python act-pick-infer/infer.py

# Timed run
python act-pick-infer/infer.py --seconds 20

# Fine-tuned Hub checkpoint (after Colab)
python act-pick-infer/infer.py --check --policy YOUR_HF_USER/act_so101_pick_finetune
python act-pick-infer/infer.py --policy YOUR_HF_USER/act_so101_pick_finetune
```

Useful flags:

| Flag | Default | Meaning |
|------|---------|---------|
| `--check` | off | Live view + predictions; no motion |
| `--policy` | `TakuyaHiraoka/act_so101_pick_diverse_objects` | Hub repo or local path |
| `--seconds` | `0` (forever) | Auto-stop |
| `--max-delta` | `10` | Max joint step per tick (safety) |

The page shows the 640×480 frame fed to the policy, measured pose, and commanded joints. First forward pass is ~0.7–1 s; later ticks mostly pop the 100-step action queue (~few ms).

---

## 2. Record demos — `record.py`

Collects LeRobot episodes for fine-tuning. Each frame stores **measured** joint pose as both `observation.state` and `action` (what the arm actually did).

```bash
python act-pick-infer/record.py \
  --repo-id YOUR_HF_USER/so101_pick_wrist_finetune \
  --task "Pick up the object and lift it"

# Append more episodes later
python act-pick-infer/record.py \
  --repo-id YOUR_HF_USER/so101_pick_wrist_finetune \
  --task "Pick up the object and lift it" \
  --resume
```

Open **http://127.0.0.1:8005** (or `http://<pi-ip>:8005` from a laptop).

### Control modes (page toggle)

**Kinesthetic**

- Arm joint torque **off** — move the arm by hand.
- Gripper powered: web slider, or hold **S** (close) / **D** (open).
- **grip speed** slider = units/sec for S/D.

**Joystick teleop**

- Gamepad stays on the **client** (e.g. MacBook). The browser Gamepad API sends targets to the Pi; nothing is plugged into the Pi for teleop.
- Torque **on** for all joints while this mode is active.
- Mapping (typical Xbox / dual-stick layout):

| Input | Joint |
|--------|--------|
| Left stick X / Y | shoulder_pan / shoulder_lift |
| Right stick X / Y | wrist_flex / elbow_flex |
| LB / RB | wrist_roll |
| LT / RT | close / open gripper |
| **arm speed** | °/s for sticks |

Press a button on the pad once so the browser sees it, then click **Joystick teleop**.

### Recording UI

1. Pose the object (bright, non-black items on a white table work best).
2. **Record** → demonstrate a pick/lift.
3. **Stop and save** (or **Discard**).
4. Repeat (~20–50 short episodes).
5. **Push** → finalize and upload a **private** Hub dataset.
6. Ctrl-C in the terminal when done.

Local data lands under `$HF_LEROBOT_HOME/<repo_id>/` (usually `~/.cache/huggingface/lerobot/<repo_id>/`).

---

## 3. View recorded videos — `view_dataset.py`

```bash
python act-pick-infer/view_dataset.py \
  --repo-id YOUR_HF_USER/so101_pick_wrist_finetune
```

Open **http://127.0.0.1:8006** — episode list + wrist clip player.

Official LeRobot one-episode viewer (Rerun / Foxglove):

```bash
lerobot-dataset-viz \
  --repo-id YOUR_HF_USER/so101_pick_wrist_finetune \
  --episode-index 0
```

---

## 4. Fine-tune on Colab — `finetune_colab.py`

Run on a **GPU** machine (not the Pi). Installs/uses LeRobot’s `lerobot-train`, starting from the base ACT weights (`--policy.path`).

```bash
# After login() / huggingface-cli login on Colab:
python finetune_colab.py \
  --dataset.repo_id YOUR_HF_USER/so101_pick_wrist_finetune \
  --policy.path TakuyaHiraoka/act_so101_pick_diverse_objects \
  --policy.repo_id YOUR_HF_USER/act_so101_pick_finetune \
  --policy.device cuda \
  --policy.push_to_hub true \
  --policy.private true \
  --batch_size 8 \
  --steps 10000 \
  --save_freq 2000 \
  --num_workers 2 \
  --wandb.enable false \
  --output_dir outputs/train/act_so101_pick_finetune \
  --job_name act_so101_pick_finetune
```

- `--dry-run` prints the `lerobot-train` command without running it.
- If the T4 OOMs, use `--batch_size 4`.
- Extra unknown flags are forwarded to `lerobot-train`.

Then on the Pi:

```bash
python act-pick-infer/infer.py --policy YOUR_HF_USER/act_so101_pick_finetune
```

---

## Object tips (grocery / household)

Wrist cam + black gripper + white table: prefer **bright, matte, rigid** objects (yellow sponge, red cup, green foam apple, blue scrubber, small orange carton). Avoid black, clear glass, shiny metal, and floppy bags for the first fine-tune.

---

## Ports cheat sheet

| Port | Service |
|------|---------|
| 8004 | Infer live view |
| 8005 | Record panel |
| 8006 | Dataset video browser |

All bind `0.0.0.0` so you can open them from another machine on the LAN.

---

## Troubleshooting

| Symptom | What to try |
|---------|-------------|
| `Port … already in use` | Stop the other `infer` / `record` / viewer process |
| Camera open fails | Nothing else using `/dev/video0`; unplug/replug LG Smart Cam |
| Arm connect / Lock write errors | Power cycle follower; close other clients on `/dev/ttyACM1` |
| Joystick does nothing | Open the page **from the MacBook**, press a pad button, select **Joystick teleop** |
| Push fails | `hf auth login` with a write token; repo id `USER/NAME` |
| Infer very slow every ~3 s | Expected on CPU when the ACT action queue refills (~100 steps) |
