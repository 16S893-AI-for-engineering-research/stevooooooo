"""Tests for inchworm.camera_capture — AA-14 ACs #1–#3, #5–#7.

AC#4 (measured FPS/latency) requires real cameras; use scripts/camera_probe.py --live.
All tests run against MockMjpegCameraTransport; no real hardware is accessed.

Coverage:
  AC#1 — arbiter enforcement: CameraCapture constructible only inside arbiter context
  AC#2 — settings read-back: resolution, fps, exposure, gain, auto_exposure, auto_gain
  AC#3 — frame metadata: single-channel grayscale, monotonic timestamp, counter,
          drop detection (counter gap), corrupt detection (failed decode)
  AC#5 — snapshot and record CLI exist and write to sessions/
  AC#6 — sequential switch: close before open (verified)
  AC#7 — mock backend round-trips, settings read-back, frame metadata, sequential switch
"""

from __future__ import annotations

import time
from pathlib import Path

import pytest

from inchworm.backends import (
    BackendError,
    MockBackend,
    MockMjpegCameraTransport,
    _make_synthetic_mjpeg,
    create_backend,
)
from inchworm.camera_arbiter import CameraArbiter, CameraArbiterTimeoutError
from inchworm.camera_capture import (
    ArbiterLockRequiredError,
    CameraCapture,
    CameraSettings,
    CaptureError,
    FrameResult,
    _decode_to_gray,
    open_capture,
)
from inchworm.config import load_config


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture()
def config():
    return load_config()


@pytest.fixture()
def backend(config):
    return MockBackend(config)


@pytest.fixture()
def lock_dir(tmp_path):
    d = tmp_path / "cam_locks"
    d.mkdir()
    return d


@pytest.fixture()
def arbiter(backend, lock_dir):
    return CameraArbiter(backend, lock_dir=lock_dir)


@pytest.fixture()
def mjpeg_transport():
    """Standalone MockMjpegCameraTransport for direct unit tests."""
    return MockMjpegCameraTransport(
        "camera_left_arm",
        width=64,
        height=48,
        fps=30.0,
        exposure=100,
        gain=1,
        auto_exposure=False,
        auto_gain=False,
    )


# ---------------------------------------------------------------------------
# AC#1 — Arbiter enforcement: CameraCapture constructible only inside context
# ---------------------------------------------------------------------------


class TestAc1ArbiterEnforcement:
    def test_open_capture_inside_context_succeeds(self, arbiter):
        """CameraCapture created inside arbiter.acquire() must succeed."""
        with arbiter.acquire("camera_left_arm") as transport:
            capture = open_capture(transport, role="camera_left_arm")
            assert capture.is_open

    def test_open_capture_with_closed_transport_raises(self):
        """CameraCapture with a closed transport must raise ArbiterLockRequiredError."""
        transport = MockMjpegCameraTransport("camera_left_arm")
        transport.close()
        with pytest.raises(ArbiterLockRequiredError, match="camera_left_arm"):
            open_capture(transport, role="camera_left_arm")

    def test_capture_inaccessible_after_context_exits(self, arbiter):
        """After the arbiter context exits, the transport is closed and grab() must raise."""
        with arbiter.acquire("camera_left_arm") as transport:
            capture = open_capture(transport, role="camera_left_arm")
        # Transport is now closed (arbiter released it).
        assert not capture.is_open
        with pytest.raises(CaptureError, match="closed"):
            capture.grab()

    def test_streaming_blocked_while_another_holds_lock(self, arbiter):
        """A second acquire() of any camera while one is active must fail."""
        session = arbiter.acquire("camera_left_arm")
        session.__enter__()
        try:
            with pytest.raises(CameraArbiterTimeoutError):
                arbiter.acquire("camera_right_arm", timeout_s=0.05).__enter__()
        finally:
            session.__exit__(None, None, None)


# ---------------------------------------------------------------------------
# AC#2 — Settings read-back
# ---------------------------------------------------------------------------


class TestAc2SettingsReadback:
    def test_set_width_height_read_back(self, mjpeg_transport):
        """Width and height set on transport must be read back correctly."""
        capture = open_capture(mjpeg_transport, role="camera_left_arm")
        applied = capture.apply_settings(CameraSettings(width=640, height=480))
        assert applied.width == 640
        assert applied.height == 480

    def test_set_fps_read_back(self, mjpeg_transport):
        """FPS set on transport must be read back correctly."""
        capture = open_capture(mjpeg_transport, role="camera_left_arm")
        applied = capture.apply_settings(CameraSettings(fps=15.0))
        assert applied.fps == pytest.approx(15.0)

    def test_set_exposure_read_back(self, mjpeg_transport):
        """Exposure set on transport must be read back correctly."""
        capture = open_capture(mjpeg_transport, role="camera_left_arm")
        applied = capture.apply_settings(CameraSettings(exposure=200))
        assert applied.exposure == 200

    def test_set_gain_read_back(self, mjpeg_transport):
        """Gain set on transport must be read back correctly."""
        capture = open_capture(mjpeg_transport, role="camera_left_arm")
        applied = capture.apply_settings(CameraSettings(gain=4))
        assert applied.gain == 4

    def test_disable_auto_exposure_read_back(self, mjpeg_transport):
        """auto_exposure=False must be read back as False."""
        capture = open_capture(mjpeg_transport, role="camera_left_arm")
        applied = capture.apply_settings(CameraSettings(auto_exposure=False))
        assert applied.auto_exposure is False

    def test_enable_auto_exposure_read_back(self, mjpeg_transport):
        """auto_exposure=True must be read back as True."""
        capture = open_capture(mjpeg_transport, role="camera_left_arm")
        applied = capture.apply_settings(CameraSettings(auto_exposure=True))
        assert applied.auto_exposure is True

    def test_disable_auto_gain_read_back(self, mjpeg_transport):
        """auto_gain=False must be read back as False."""
        capture = open_capture(mjpeg_transport, role="camera_left_arm")
        applied = capture.apply_settings(CameraSettings(auto_gain=False))
        assert applied.auto_gain is False

    def test_multiple_settings_in_one_call(self, mjpeg_transport):
        """All settings applied in one call must all be read back."""
        capture = open_capture(mjpeg_transport, role="camera_left_arm")
        applied = capture.apply_settings(
            CameraSettings(
                width=800,
                height=600,
                fps=25.0,
                exposure=150,
                gain=2,
                auto_exposure=False,
                auto_gain=False,
            )
        )
        assert applied.width == 800
        assert applied.height == 600
        assert applied.fps == pytest.approx(25.0)
        assert applied.exposure == 150
        assert applied.gain == 2
        assert applied.auto_exposure is False
        assert applied.auto_gain is False

    def test_settings_via_kwargs(self, mjpeg_transport):
        """apply_settings(**kwargs) must work the same as CameraSettings."""
        capture = open_capture(mjpeg_transport, role="camera_left_arm")
        applied = capture.apply_settings(width=320, height=240)
        assert applied.width == 320
        assert applied.height == 240

    def test_settings_logged(self, mjpeg_transport, tmp_path):
        """Settings application must emit a telemetry event via the logger."""
        from inchworm.telemetry import SessionLogger

        log_path = tmp_path / "events.jsonl"
        logger = SessionLogger(log_path)
        capture = open_capture(mjpeg_transport, role="camera_left_arm", logger=logger)
        capture.apply_settings(CameraSettings(width=640, height=480, auto_exposure=False))
        logger.close()

        import json
        lines = log_path.read_text().strip().splitlines()
        events = [json.loads(l) for l in lines if l.strip()]
        types = [e["event_type"] for e in events]
        assert "camera_settings_applied" in types

        setting_event = next(e for e in events if e["event_type"] == "camera_settings_applied")
        assert setting_event["payload"]["applied"]["width"] == 640
        assert setting_event["payload"]["applied"]["auto_exposure"] is False


# ---------------------------------------------------------------------------
# AC#3 — Frame metadata: grayscale, monotonic timestamps, counter, drop/corrupt
# ---------------------------------------------------------------------------


class TestAc3FrameMetadata:
    def test_frame_is_2d_uint8_grayscale(self, mjpeg_transport):
        """Frame data must be a 2D uint8 numpy array (single channel)."""
        import numpy as np
        capture = open_capture(mjpeg_transport, role="camera_left_arm")
        result = capture.grab()
        assert isinstance(result, FrameResult)
        assert result.data.ndim == 2
        assert result.data.dtype == np.uint8

    def test_frame_shape_matches_settings(self, mjpeg_transport):
        """Frame shape must match the requested resolution."""
        capture = open_capture(mjpeg_transport, role="camera_left_arm")
        capture.apply_settings(CameraSettings(width=64, height=48))
        result = capture.grab()
        assert result.data.shape == (48, 64)

    def test_frame_counter_starts_at_1(self, mjpeg_transport):
        """First frame counter must be 1."""
        capture = open_capture(mjpeg_transport, role="camera_left_arm")
        result = capture.grab()
        assert result.counter == 1

    def test_frame_counter_increments(self, mjpeg_transport):
        """Successive frame counters must increment by 1."""
        capture = open_capture(mjpeg_transport, role="camera_left_arm")
        results = [capture.grab() for _ in range(5)]
        for i, r in enumerate(results, start=1):
            assert r.counter == i

    def test_monotonic_timestamps_increase(self, mjpeg_transport):
        """Successive frame timestamps must be non-decreasing."""
        capture = open_capture(mjpeg_transport, role="camera_left_arm")
        results = [capture.grab() for _ in range(5)]
        for a, b in zip(results, results[1:]):
            assert b.ts_monotonic >= a.ts_monotonic

    def test_timestamp_is_float(self, mjpeg_transport):
        """Timestamp must be a float (seconds, monotonic)."""
        capture = open_capture(mjpeg_transport, role="camera_left_arm")
        result = capture.grab()
        assert isinstance(result.ts_monotonic, float)
        assert result.ts_monotonic > 0

    def test_no_drop_flag_on_consecutive_frames(self, mjpeg_transport):
        """Normal consecutive frames must not set dropped=True."""
        capture = open_capture(mjpeg_transport, role="camera_left_arm")
        results = [capture.grab() for _ in range(5)]
        assert all(not r.dropped for r in results)

    def test_drop_detected_on_counter_gap(self, mjpeg_transport):
        """A gap in transport counter must set dropped=True on the next frame."""
        capture = open_capture(mjpeg_transport, role="camera_left_arm")
        # Grab one frame first to establish baseline.
        r0 = capture.grab()
        assert not r0.dropped
        # Simulate 2 dropped frames.
        mjpeg_transport.skip_next_n = 2
        r1 = capture.grab()
        assert r1.dropped

    def test_drop_not_flagged_after_recovery(self, mjpeg_transport):
        """After a drop event, subsequent consecutive frames must not flag dropped."""
        capture = open_capture(mjpeg_transport, role="camera_left_arm")
        capture.grab()
        mjpeg_transport.skip_next_n = 1
        capture.grab()  # dropped frame
        r = capture.grab()
        assert not r.dropped

    def test_corrupt_flagged_on_bad_decode(self, mjpeg_transport):
        """A frame that fails JPEG decode must set corrupt=True."""
        capture = open_capture(mjpeg_transport, role="camera_left_arm")
        mjpeg_transport.corrupt_next_n = 1
        result = capture.grab()
        assert result.corrupt

    def test_corrupt_frame_data_is_empty(self, mjpeg_transport):
        """Corrupt frame data must be shape (0, 0)."""
        import numpy as np
        capture = open_capture(mjpeg_transport, role="camera_left_arm")
        mjpeg_transport.corrupt_next_n = 1
        result = capture.grab()
        assert result.data.shape == (0, 0)

    def test_corrupt_not_flagged_on_normal_frame(self, mjpeg_transport):
        """Normal frames must not be flagged corrupt."""
        capture = open_capture(mjpeg_transport, role="camera_left_arm")
        result = capture.grab()
        assert not result.corrupt

    def test_drop_event_logged(self, mjpeg_transport, tmp_path):
        """Drop events must emit a telemetry event."""
        from inchworm.telemetry import SessionLogger
        import json

        log_path = tmp_path / "events.jsonl"
        logger = SessionLogger(log_path)
        capture = open_capture(mjpeg_transport, role="camera_left_arm", logger=logger)
        capture.grab()  # baseline
        mjpeg_transport.skip_next_n = 1
        capture.grab()
        logger.close()

        lines = log_path.read_text().strip().splitlines()
        events = [json.loads(l) for l in lines if l.strip()]
        types = [e["event_type"] for e in events]
        assert "camera_frame_dropped" in types

    def test_corrupt_event_logged(self, mjpeg_transport, tmp_path):
        """Corrupt events must emit a telemetry event."""
        from inchworm.telemetry import SessionLogger
        import json

        log_path = tmp_path / "events.jsonl"
        logger = SessionLogger(log_path)
        capture = open_capture(mjpeg_transport, role="camera_left_arm", logger=logger)
        mjpeg_transport.corrupt_next_n = 1
        capture.grab()
        logger.close()

        lines = log_path.read_text().strip().splitlines()
        events = [json.loads(l) for l in lines if l.strip()]
        types = [e["event_type"] for e in events]
        assert "camera_frame_corrupt" in types


# ---------------------------------------------------------------------------
# AC#5 — Snapshot and record CLI write to sessions/
# ---------------------------------------------------------------------------


class TestAc5Cli:
    def test_snapshot_creates_artifact(self, tmp_path, config):
        """inchworm-camera snapshot must write at least one artifact file."""
        from inchworm.camera_cli import main

        session_root = tmp_path / "sessions"
        ret = main([
            "snapshot",
            "--role", "camera_left_arm",
            "--session-root", str(session_root),
        ])
        assert ret == 0
        # Find artifact files created.
        artifacts = list(session_root.rglob("camera_left_arm_*"))
        assert len(artifacts) >= 1, f"No artifacts found under {session_root}"

    def test_record_creates_multiple_artifacts(self, tmp_path, config):
        """inchworm-camera record --frames N must write N artifacts."""
        from inchworm.camera_cli import main

        session_root = tmp_path / "sessions"
        ret = main([
            "record",
            "--role", "camera_right_arm",
            "--frames", "3",
            "--session-root", str(session_root),
        ])
        assert ret == 0
        artifacts = list(session_root.rglob("camera_right_arm_*"))
        assert len(artifacts) == 3, f"Expected 3 artifacts, got {len(artifacts)}"

    def test_snapshot_session_dir_under_sessions(self, tmp_path):
        """Artifacts must land inside robot/sessions/ (or the overridden root)."""
        from inchworm.camera_cli import main

        session_root = tmp_path / "sessions"
        main(["snapshot", "--role", "camera_scanner", "--session-root", str(session_root)])
        assert session_root.exists()
        # Session sub-directory must have been created.
        subdirs = [d for d in session_root.iterdir() if d.is_dir()]
        assert len(subdirs) == 1

    def test_record_with_settings(self, tmp_path):
        """record with --width/--height/--exposure must apply settings (smoke test)."""
        from inchworm.camera_cli import main

        session_root = tmp_path / "sessions"
        ret = main([
            "record",
            "--role", "camera_left_arm",
            "--frames", "2",
            "--width", "64",
            "--height", "48",
            "--exposure", "150",
            "--no-auto-exposure",
            "--session-root", str(session_root),
        ])
        assert ret == 0


# ---------------------------------------------------------------------------
# AC#6 — Sequential switch: close first device before opening second
# ---------------------------------------------------------------------------


class TestAc6SequentialSwitch:
    def test_first_transport_closed_before_second_opens(self, backend, lock_dir):
        """The first camera must be fully closed before the second one opens."""
        arbiter = CameraArbiter(backend, lock_dir=lock_dir)

        first_transport = None
        with arbiter.acquire("camera_left_arm") as transport:
            first_transport = transport
            assert transport.is_open

        # After exiting: first transport must be closed.
        assert not first_transport.is_open

        second_transport = None
        with arbiter.acquire("camera_right_arm") as transport:
            second_transport = transport
            assert transport.is_open
            # First is still closed while second is open.
            assert not first_transport.is_open

        assert not second_transport.is_open

    def test_sequential_switch_different_roles_all_roles(self, backend, lock_dir):
        """All three camera roles can be opened sequentially without errors."""
        arbiter = CameraArbiter(backend, lock_dir=lock_dir)
        roles = ["camera_left_arm", "camera_right_arm", "camera_scanner"]
        closed_transports = []
        for role in roles:
            with arbiter.acquire(role) as transport:
                assert transport.is_open
                # All previously used transports must be closed.
                for prev in closed_transports:
                    assert not prev.is_open
            closed_transports.append(transport)

    def test_grab_after_sequential_switch(self, lock_dir, config):
        """After sequential switch, the new camera must deliver valid frames."""
        from inchworm.backends import MockMjpegBackend
        mjpeg_backend = MockMjpegBackend(config)
        arbiter = CameraArbiter(mjpeg_backend, lock_dir=lock_dir)
        with arbiter.acquire("camera_left_arm") as t1:
            c1 = open_capture(t1, role="camera_left_arm")
            r1 = c1.grab()
            assert not r1.corrupt
        # First is closed now.
        with arbiter.acquire("camera_right_arm") as t2:
            c2 = open_capture(t2, role="camera_right_arm")
            r2 = c2.grab()
            assert not r2.corrupt
        assert not t1.is_open
        assert not t2.is_open


# ---------------------------------------------------------------------------
# AC#7 — Mock backend round-trips, settings read-back, frame metadata
# ---------------------------------------------------------------------------


class TestAc7MockBackend:
    def test_mjpeg_transport_frame_decodes_to_grayscale(self):
        """MockMjpegCameraTransport frames must decode to valid grayscale arrays."""
        import numpy as np
        transport = MockMjpegCameraTransport("camera_left_arm", width=64, height=48)
        raw, counter = transport.read_frame_with_counter()
        data, corrupt = _decode_to_gray(raw)
        assert not corrupt
        assert data.ndim == 2
        assert data.dtype == np.uint8
        assert data.shape == (48, 64)

    def test_mjpeg_transport_counter_increments(self):
        """Transport counter must increment by 1 per frame."""
        transport = MockMjpegCameraTransport("camera_left_arm")
        for expected in range(1, 6):
            _, counter = transport.read_frame_with_counter()
            assert counter == expected

    def test_mjpeg_transport_skip_produces_gap(self):
        """skip_next_n must produce a gap in transport counter."""
        transport = MockMjpegCameraTransport("camera_left_arm")
        _, c1 = transport.read_frame_with_counter()
        assert c1 == 1
        transport.skip_next_n = 2
        _, c2 = transport.read_frame_with_counter()
        assert c2 == 4  # skipped 2 → counter jumps 1+2+1=4

    def test_mjpeg_transport_corrupt_bytes(self):
        """corrupt_next_n must return undecodable bytes."""
        transport = MockMjpegCameraTransport("camera_left_arm", width=64, height=48)
        transport.corrupt_next_n = 1
        raw, _ = transport.read_frame_with_counter()
        data, corrupt = _decode_to_gray(raw)
        assert corrupt

    def test_mjpeg_transport_settings_read_back(self):
        """Settings applied to transport must all round-trip correctly."""
        transport = MockMjpegCameraTransport("camera_left_arm")
        from inchworm.camera_capture import CameraSettings
        applied = transport.apply_settings(
            CameraSettings(
                width=1280, height=720, fps=60.0,
                exposure=50, gain=8,
                auto_exposure=True, auto_gain=True,
            )
        )
        assert applied.width == 1280
        assert applied.height == 720
        assert applied.fps == pytest.approx(60.0)
        assert applied.exposure == 50
        assert applied.gain == 8
        assert applied.auto_exposure is True
        assert applied.auto_gain is True

    def test_mjpeg_transport_closed_raises_on_read(self):
        """Reading from a closed transport must raise BackendError."""
        transport = MockMjpegCameraTransport("camera_left_arm")
        transport.close()
        with pytest.raises(BackendError, match="closed"):
            transport.read_frame_with_counter()

    def test_arbiter_mock_backend_enforces_lock(self, arbiter):
        """Arbiter with MockBackend must enforce single-camera invariant."""
        s = arbiter.acquire("camera_left_arm")
        s.__enter__()
        try:
            with pytest.raises(CameraArbiterTimeoutError):
                arbiter.acquire("camera_left_arm", timeout_s=0.05).__enter__()
        finally:
            s.__exit__(None, None, None)

    def test_full_capture_pipeline_with_mock_mjpeg(self, tmp_path, config):
        """Full pipeline (arbiter → transport → capture → grab) with mock MJPEG."""
        import numpy as np
        from inchworm.backends import MockMjpegBackend

        lock_dir = tmp_path / "locks"
        lock_dir.mkdir()
        backend = MockMjpegBackend(config)
        arbiter = CameraArbiter(backend, lock_dir=lock_dir)

        with arbiter.acquire("camera_left_arm") as transport:
            capture = open_capture(transport, role="camera_left_arm")
            capture.apply_settings(CameraSettings(width=64, height=48, auto_exposure=False))
            frames = [capture.grab() for _ in range(3)]

        # All frames valid.
        for i, f in enumerate(frames, start=1):
            assert f.counter == i
            assert not f.corrupt
            assert not f.dropped
            assert f.data.ndim == 2
            assert f.data.dtype == np.uint8


# ---------------------------------------------------------------------------
# Synthetic MJPEG helper tests
# ---------------------------------------------------------------------------


class TestSyntheticMjpeg:
    def test_synthetic_frame_decodes(self):
        """Synthetic MJPEG bytes must decode to grayscale without error."""
        import numpy as np
        raw = _make_synthetic_mjpeg(64, 48)
        data, corrupt = _decode_to_gray(raw)
        assert not corrupt
        assert data.shape == (48, 64)
        assert data.dtype == np.uint8

    def test_synthetic_frame_pixel_values_vary(self):
        """Synthetic frame must have non-uniform pixels (not blank)."""
        import numpy as np
        raw = _make_synthetic_mjpeg(64, 48)
        data, corrupt = _decode_to_gray(raw)
        if not corrupt:
            # The gray ramp image has varying pixel values.
            assert data.max() > data.min() or data.size == 0


# ---------------------------------------------------------------------------
# Decode helper unit tests
# ---------------------------------------------------------------------------


class TestDecodeHelper:
    def test_empty_bytes_is_corrupt(self):
        """Empty bytes must decode as corrupt."""
        import numpy as np
        data, corrupt = _decode_to_gray(b"")
        assert corrupt
        assert data.shape == (0, 0)

    def test_invalid_bytes_is_corrupt(self):
        """Garbage bytes must decode as corrupt."""
        import numpy as np
        data, corrupt = _decode_to_gray(b"NOTJPEG_GARBAGE_DATA")
        assert corrupt

    def test_valid_mjpeg_decodes_correctly(self):
        """Real MJPEG bytes must decode to a 2D grayscale array."""
        import numpy as np
        try:
            import cv2
        except ImportError:
            pytest.skip("cv2 not available")
        raw = _make_synthetic_mjpeg(32, 32)
        if raw == b"\xff\xd8\xff\xd9":
            pytest.skip("cv2 not available for encoding")
        data, corrupt = _decode_to_gray(raw)
        assert not corrupt
        assert data.ndim == 2
        assert data.dtype == np.uint8
        assert data.shape == (32, 32)


# --- Orchestrator hardening test 2026-10-24 ---
def test_v4l2_transport_refuses_open_without_global_lock():
    """Direct V4L2CameraTransport.open() outside CameraArbiter.acquire() must fail
    before touching any device, so streaming without the lock is impossible."""
    import pytest
    from inchworm.backends import BackendError, V4L2CameraTransport
    with pytest.raises(BackendError, match="global camera lock"):
        V4L2CameraTransport.open("camera_left_arm", identity=None)


# --- Orchestrator 2026-10-25: truncated MJPEG detection (hardware finding) ---
def test_truncated_jpeg_is_reported_corrupt():
    import cv2
    import numpy as np
    from inchworm.camera_capture import _decode_to_gray
    img = (np.arange(64 * 48, dtype=np.uint8).reshape(48, 64))
    ok, buf = cv2.imencode(".jpg", img)
    full = buf.tobytes()
    data, corrupt = _decode_to_gray(full)
    assert not corrupt and data.shape == (48, 64)
    # Driver zero padding after EOI is still a complete frame.
    _, corrupt = _decode_to_gray(full + b"\x00" * 32)
    assert not corrupt
    # Truncated bitstream (no EOI) decodes in libjpeg but must be flagged.
    _, corrupt = _decode_to_gray(full[: len(full) * 2 // 3])
    assert corrupt
