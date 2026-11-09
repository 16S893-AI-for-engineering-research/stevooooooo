"""AA-30 pre-flight: the --live paths of scripts/calibrate_scanner.py.

The live paths were written against camera APIs that do not exist
(``CameraArbiter(config)``, ``CameraCapture(transport)``, ``read_gray``), so a
first live run would have crashed. These tests drive the live code with a
scripted camera transport (real JPEG round-trip through ``CameraCapture``), a
mock relay, and no hardware. They also check the chessboard PDF generator.
"""

from __future__ import annotations

import importlib.util
import subprocess
from pathlib import Path

import cv2
import numpy as np
import pytest

from inchworm.backends import MockBackend, MockMjpegBackend, MockMjpegCameraTransport
from inchworm.config import load_config

_ROBOT = Path(__file__).resolve().parents[1]


def _load(name: str):
    spec = importlib.util.spec_from_file_location(name, _ROBOT / "scripts" / f"{name}.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope="module")
def cal():
    return _load("calibrate_scanner")


@pytest.fixture(scope="module")
def config():
    return load_config()


K = np.array([[800.0, 0.0, 640.0], [0.0, 800.0, 400.0], [0.0, 0.0, 1.0]])


def render_boards(n, seed=3):
    """Detectable chessboard images: a 10x7-square board (25 mm) warped through K."""
    rng = np.random.default_rng(seed)
    px = 20  # board texture pixels per mm-square edge
    tex = np.full((7 * px + 2 * px, 10 * px + 2 * px), 255, np.uint8)  # white border
    for r in range(7):
        for c in range(10):
            if (r + c) % 2 == 0:
                tex[px + r * px: px + (r + 1) * px, px + c * px: px + (c + 1) * px] = 0
    # texture pixel -> board mm (board origin at inner top-left of squares, -25 mm border)
    out = []
    while len(out) < n:
        rvec = rng.uniform(-0.35, 0.35, 3)
        # board centre ~ (112.5, 75) mm from its corner; put it ~330 mm in front
        R, _ = cv2.Rodrigues(rvec)
        centre = np.array([112.5, 87.5 - 12.5, 0.0])
        t = np.array([rng.uniform(-30, 30), rng.uniform(-30, 30), rng.uniform(300, 420)]) - R @ centre
        src_mm = np.array([[-25, -25, 0], [275 - 25, -25, 0], [275 - 25, 200 - 25, 0], [-25, 200 - 25, 0]], np.float64)
        proj, _ = cv2.projectPoints(src_mm, rvec, t, K, np.zeros(4))
        tex_pts = np.array([[0, 0], [tex.shape[1], 0], [tex.shape[1], tex.shape[0]], [0, tex.shape[0]]], np.float32)
        H = cv2.getPerspectiveTransform(tex_pts, proj.reshape(4, 2).astype(np.float32))
        img = cv2.warpPerspective(tex, H, (1280, 800), borderValue=128)
        if cv2.findChessboardCorners(img, (9, 6), None)[0]:
            out.append(img)
    return out


class ScriptedCamera(MockMjpegCameraTransport):
    """Serves the given grayscale frames (JPEG-encoded) in order, cycling."""

    def __init__(self, device, frames, corrupt_every=0):
        super().__init__(device)
        self._frames = frames
        self._i = 0
        self._n = 0
        self._corrupt_every = corrupt_every

    def read_frame_with_counter(self):
        self._transport_counter += 1
        self._n += 1
        if self._corrupt_every and self._n % self._corrupt_every == 0:
            return b"\xff\xd8\xff\x00NOT_A_JPEG", self._transport_counter
        frame = self._frames[self._i % len(self._frames)]
        self._i += 1
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
        assert ok
        return buf.tobytes(), self._transport_counter


def _backend(config, frames, corrupt_every=0):
    class _B(MockMjpegBackend):
        def _open_camera(self, device, identity):
            t = ScriptedCamera(device, frames, corrupt_every)
            self.cameras[device] = t
            return t

    return _B(config)


def test_intrinsics_live_path_runs_and_skips_damaged_frames(cal, config, tmp_path):
    good = render_boards(8)
    prompts = []
    result = cal.run_intrinsics(
        config, MockBackend(config), live=True, n_frames=5,
        camera_backend=_backend(config, good, corrupt_every=3),
        prompt=prompts.append, frames_dir=tmp_path / "frames",
    )
    assert result.n_images_used == 5
    assert result.camera_matrix.shape == (3, 3)
    assert result.rms_reprojection_error_px < 2.0
    assert len(prompts) == 5
    assert len(list((tmp_path / "frames").glob("*.png"))) == 5


def test_grab_gray_raises_when_all_frames_damaged(cal, config):
    from inchworm.camera_arbiter import CameraArbiter
    from inchworm.camera_capture import open_capture

    be = _backend(config, [np.zeros((800, 1280), np.uint8)], corrupt_every=1)
    with CameraArbiter(be).acquire("camera_scanner") as t:
        cap = open_capture(t, role="camera_scanner")
        with pytest.raises(RuntimeError, match="damaged"):
            cal._grab_gray(cap)


def test_background_subtraction_does_not_wrap(cal):
    laser = np.array([[10, 200]], np.uint8)
    bg = np.array([[50, 100]], np.uint8)
    assert cal._subtract_background(laser, bg).tolist() == [[0, 100]]


def test_laser_plane_live_path_runs_and_laser_ends_off(cal, config):
    from inchworm.scanner_calibration import IntrinsicsResult

    board = render_boards(1)[0]
    lit = board.copy()
    lit[300:303, :] = 255  # a bright horizontal line across all columns
    intr = IntrinsicsResult(
        camera_matrix=K, dist_coeffs=np.zeros(5), rms_reprojection_error_px=0.1,
        per_image_reprojection_error_px=[], n_images_used=1,
        target_description="test", unverified=True,
    )
    backend = MockBackend(config)
    # Per standoff: background frame (laser off), then laser frame.
    result = cal.run_laser_plane(
        config, backend, intr, live=True, standoffs=[70.0, 80.0],
        camera_backend=_backend(config, [board, lit]), prompt=lambda s: None,
    )
    assert result.n_points > 10
    relay = backend.serial["relay"]
    # Last command written to the relay must be the OFF frame.
    assert relay.written[-1] == bytes([0xA0, 0x01, 0x00, 0xA1])


def test_chessboard_pdf_is_valid_and_has_right_geometry(tmp_path):
    gen = _load("make_chessboard_pdf")
    pdf = tmp_path / "board.pdf"
    pdf.write_bytes(gen.build_pdf())
    data = pdf.read_bytes()
    assert data.startswith(b"%PDF-1.4") and data.rstrip().endswith(b"%%EOF")
    committed = _ROBOT / "assets" / "chessboard_9x6_25mm.pdf"
    assert committed.read_bytes() == data, "regenerate: scripts/make_chessboard_pdf.py"
    # Render (ghostscript, if available) and check OpenCV finds 9x6 at 25 mm / square.
    try:
        png = tmp_path / "b.png"
        subprocess.run(
            ["gs", "-q", "-dNOPAUSE", "-dBATCH", "-sDEVICE=pnggray", "-r200",
             f"-sOutputFile={png}", str(pdf)], check=True, timeout=30)
    except (FileNotFoundError, subprocess.CalledProcessError):
        pytest.skip("ghostscript not available")
    im = cv2.imread(str(png), 0)
    ok, c = cv2.findChessboardCorners(im, (9, 6), None)
    assert ok
    c = c.reshape(-1, 2)
    mm = np.linalg.norm(c[1] - c[0]) / 200.0 * 25.4
    assert mm == pytest.approx(25.0, abs=0.3)
