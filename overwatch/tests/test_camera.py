import sys

import pytest

from overwatch import camera
from overwatch.camera import FfmpegName, resolve_source, split_jpegs

DEVICES = [(0, "FaceTime HD Camera"), (1, "HD camera")]


def test_split_jpegs_returns_complete_frames_and_keeps_the_partial_tail():
    a, b = b"\xff\xd8AAAA\xff\xd9", b"\xff\xd8BB\xff\xd9"
    buf = bytearray(b"junk" + a + b + b"\xff\xd8CC")
    assert split_jpegs(buf) == [a, b]
    assert bytes(buf) == b"\xff\xd8CC"            # incomplete frame waits for more bytes
    buf += b"DD\xff\xd9"
    assert split_jpegs(buf) == [b"\xff\xd8CCDD\xff\xd9"] and not buf


def test_name_is_exact_and_never_falls_back_to_the_builtin_camera(monkeypatch):
    monkeypatch.setattr(camera, "list_video_devices", lambda: DEVICES)
    monkeypatch.setattr(camera.sys, "platform", "darwin")
    monkeypatch.setattr(camera.shutil, "which", lambda _: "/usr/bin/ffmpeg")
    target, label = resolve_source("name:HD camera")
    assert isinstance(target, FfmpegName) and str(target) == "HD camera"      # not "FaceTime HD Camera"
    with pytest.raises(LookupError, match="several cameras"):                  # ambiguous substring
        resolve_source("name:HD")
    with pytest.raises(LookupError, match="No video device"):                  # unplugged: refuse, do not substitute
        resolve_source("name:USB 2.0 Camera")


def test_name_lookup_on_linux_uses_the_device_path(monkeypatch):
    monkeypatch.setattr(camera, "list_video_devices", lambda: [(0, "Integrated"), (2, "HD camera")])
    monkeypatch.setattr(camera.sys, "platform", "linux")
    assert resolve_source("name:HD camera")[0] == "/dev/video2"


def test_urls_and_files_pass_through():
    assert resolve_source("rtsp://10.0.0.5/s1")[0] == "rtsp://10.0.0.5/s1"
    assert resolve_source("/tmp/x.mp4")[0] == "/tmp/x.mp4"


def test_rotation_180_is_upright_for_an_upside_down_camera():
    import numpy as np
    from overwatch.camera import apply_rotation
    img = np.arange(2 * 3 * 3, dtype=np.uint8).reshape(2, 3, 3)
    assert (apply_rotation(img, 0) == img).all()
    assert (apply_rotation(img, 180) == img[::-1, ::-1]).all()
    assert apply_rotation(img, 90).shape == (3, 2, 3)
    with pytest.raises(ValueError):
        apply_rotation(img, 45)


def test_bad_rotation_is_refused_at_startup(tmp_path):
    import dataclasses
    from overwatch.app import create_app
    from overwatch.config import Settings
    s = dataclasses.replace(Settings(), data_dir=tmp_path, source="index:0", rotate=45)
    with pytest.raises(SystemExit, match="ROTATE"):
        create_app(s)
