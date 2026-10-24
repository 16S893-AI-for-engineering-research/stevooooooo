"""Tests for AA-22: InchwormLeRobotAdapter (main suite, no lerobot required).

All tests run against a fake minimal Robot base when lerobot is absent,
so the full main test suite (``uv run pytest``) always passes.

ACs verified here (software-only):
  AC#3 — camera frame goes through injected frame_source (arbiter path verified
          structurally: adapter never opens a camera internally).
  AC#4 — gripper key is structurally absent from action_features; send_action
          raises GripperCommandBlocked for any non-JOINT key.
  AC#5 — construction-time contract check raises AdapterContractMismatch on
          N_JOINTS / JOINT_ORDER mismatch.
  AC#9 — camera_wh resolved from caller (AA-5 path); adapter accepts (w, h) at
          construction time and reflects it in observation_features shape.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Dict, Optional, Tuple
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

# ---------------------------------------------------------------------------
# Fake lerobot stub — used when lerobot is absent
# ---------------------------------------------------------------------------

def _install_lerobot_stub() -> None:
    """Inject a minimal lerobot stub into sys.modules so the adapter loads."""
    import types  # noqa: PLC0415
    import dataclasses  # noqa: PLC0415
    import abc  # noqa: PLC0415

    # --- lerobot.robots.config -------------------------------------------
    lerobot_robots_config = types.ModuleType("lerobot.robots.config")

    class _ChoiceRegistry:
        """Minimal draccus ChoiceRegistry stub."""
        @classmethod
        def get_choice_name(cls, klass: type) -> str:
            return getattr(klass, "__name__", "unknown")

    @dataclasses.dataclass(kw_only=True)
    class _RobotConfig(_ChoiceRegistry, abc.ABC):
        id: Optional[str] = None
        calibration_dir: Optional[Path] = None

        @property
        def type(self) -> str:
            return type(self).__name__

    lerobot_robots_config.RobotConfig = _RobotConfig  # type: ignore[attr-defined]

    # --- lerobot.robots.robot -------------------------------------------
    lerobot_robots_robot = types.ModuleType("lerobot.robots.robot")

    class _Robot(abc.ABC):
        config_class: type
        name: str

        def __init__(self, config: Any) -> None:
            self.robot_type = self.name
            self.id = config.id
            self.calibration_dir = config.calibration_dir or Path("/tmp/inchworm_test_cal")
            self.calibration_dir.mkdir(parents=True, exist_ok=True)
            self.calibration_fpath = self.calibration_dir / f"{self.id or 'test'}.json"
            self.calibration: dict = {}

        @property
        @abc.abstractmethod
        def observation_features(self) -> dict: ...
        @property
        @abc.abstractmethod
        def action_features(self) -> dict: ...
        @property
        @abc.abstractmethod
        def is_connected(self) -> bool: ...
        @abc.abstractmethod
        def connect(self, calibrate: bool = True) -> None: ...
        @property
        @abc.abstractmethod
        def is_calibrated(self) -> bool: ...
        @abc.abstractmethod
        def calibrate(self) -> None: ...
        @abc.abstractmethod
        def configure(self) -> None: ...
        @abc.abstractmethod
        def get_observation(self) -> dict: ...
        @abc.abstractmethod
        def send_action(self, action: dict) -> dict: ...
        @abc.abstractmethod
        def disconnect(self) -> None: ...

    lerobot_robots_robot.Robot = _Robot  # type: ignore[attr-defined]

    # --- lerobot.lerobot_types -------------------------------------------
    lerobot_types = types.ModuleType("lerobot.lerobot_types")
    lerobot_types.RobotAction = Dict[str, Any]  # type: ignore[attr-defined]
    lerobot_types.RobotObservation = Dict[str, Any]  # type: ignore[attr-defined]

    # --- parent modules --------------------------------------------------
    lerobot_pkg = types.ModuleType("lerobot")
    lerobot_pkg.__version__ = "0.0.0-stub"  # type: ignore[attr-defined]
    lerobot_robots = types.ModuleType("lerobot.robots")

    sys.modules.setdefault("lerobot", lerobot_pkg)
    sys.modules.setdefault("lerobot.robots", lerobot_robots)
    sys.modules.setdefault("lerobot.robots.robot", lerobot_robots_robot)
    sys.modules.setdefault("lerobot.robots.config", lerobot_robots_config)
    sys.modules.setdefault("lerobot.lerobot_types", lerobot_types)


# Install stub before importing adapter, but only if lerobot is not present.
try:
    import lerobot  # noqa: F401
    _LEROBOT_REAL = True
except ImportError:
    _install_lerobot_stub()
    _LEROBOT_REAL = False


from inchworm.lerobot_adapter import (  # noqa: E402
    ADAPTER_JOINT_ORDER,
    ACTION_JOINT_KEYS,
    AdapterContractMismatch,
    CAMERA_KEY,
    CAMERA_SHAPE,
    OBSERVATION_JOINT_KEYS,
    _check_contract,
    _make_adapter_class,
    _make_config_class,
)
from inchworm.policy_actions import GripperCommandBlocked  # noqa: E402


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_guarded_arm_mock(joint_positions: Optional[Dict[str, float]] = None) -> MagicMock:
    """Return a mock GuardedArm with read_joint_deg and apply_joint_command."""
    mock = MagicMock(name="GuardedArm")
    positions = joint_positions or {j: 0.0 for j in ADAPTER_JOINT_ORDER}
    mock.read_joint_deg.side_effect = lambda joint: positions.get(joint, 0.0)
    mock.apply_joint_command.return_value = None
    return mock


def _make_frame_source(h: int = 480, w: int = 640) -> MagicMock:
    """Return a mock frame_source returning a (H, W) uint8 mono frame."""
    mock = MagicMock(name="frame_source")
    mock.return_value = np.zeros((h, w), dtype=np.uint8)
    return mock


def _build_adapter(
    arm_role: str = "arm_left",
    frame_h: int = 480,
    frame_w: int = 640,
    camera_wh: Optional[Tuple[int, int]] = None,
    joint_positions: Optional[Dict[str, float]] = None,
    tmp_dir: Optional[Path] = None,
) -> Any:
    """Build an InchwormLeRobotAdapter with mock dependencies."""
    Config = _make_config_class()
    config = Config(arm_role=arm_role, frame_height=frame_h, frame_width=frame_w)

    guarded_arm = _make_guarded_arm_mock(joint_positions)
    frame_source = _make_frame_source(frame_h, frame_w)

    AdapterCls = _make_adapter_class()
    adapter = AdapterCls(
        config,
        guarded_arm=guarded_arm,
        frame_source=frame_source,
        camera_wh=camera_wh,
        calibration_dir=tmp_dir,
    )
    return adapter, guarded_arm, frame_source


# ---------------------------------------------------------------------------
# Tests: module-level constants
# ---------------------------------------------------------------------------

class TestConstants:
    def test_joint_order_is_five_proximal_to_distal(self):
        """ADAPTER_JOINT_ORDER must be the 5-joint proximal-to-distal order."""
        assert ADAPTER_JOINT_ORDER == (
            "shoulder_pan",
            "shoulder_lift",
            "elbow_flex",
            "wrist_flex",
            "wrist_roll",
        )

    def test_no_gripper_in_joint_keys(self):
        """gripper must NOT appear in observation or action keys (AC#4)."""
        for key in OBSERVATION_JOINT_KEYS:
            assert "gripper" not in key
        for key in ACTION_JOINT_KEYS:
            assert "gripper" not in key

    def test_camera_shape_is_hwc(self):
        """CAMERA_SHAPE must be (H, W, 1)."""
        assert len(CAMERA_SHAPE) == 3
        assert CAMERA_SHAPE[2] == 1  # mono channel


# ---------------------------------------------------------------------------
# Tests: AC#5 — construction-time contract check
# ---------------------------------------------------------------------------

class TestContractCheck:
    def test_valid_contract_does_not_raise(self):
        """_check_contract() should not raise with the real policy_contract."""
        _check_contract()  # no exception

    def test_mismatch_n_joints_raises(self):
        """Patching N_JOINTS to wrong value must raise AdapterContractMismatch."""
        with patch("inchworm.lerobot_adapter.N_JOINTS", 4):
            with pytest.raises(AdapterContractMismatch, match="N_JOINTS"):
                _check_contract()

    def test_mismatch_joint_order_raises(self):
        """Patching JOINT_ORDER to wrong value must raise AdapterContractMismatch."""
        wrong_order = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrong")
        with patch("inchworm.lerobot_adapter.JOINT_ORDER", wrong_order):
            with pytest.raises(AdapterContractMismatch, match="JOINT_ORDER"):
                _check_contract()


# ---------------------------------------------------------------------------
# Tests: observation_features / action_features
# ---------------------------------------------------------------------------

class TestFeatures:
    def test_observation_features_has_five_joint_floats(self, tmp_path):
        adapter, _, _ = _build_adapter(tmp_dir=tmp_path)
        obs_ft = adapter.observation_features
        for key in OBSERVATION_JOINT_KEYS:
            assert key in obs_ft
            assert obs_ft[key] is float

    def test_observation_features_has_camera_key(self, tmp_path):
        adapter, _, _ = _build_adapter(tmp_dir=tmp_path)
        obs_ft = adapter.observation_features
        assert CAMERA_KEY in obs_ft
        h, w, c = obs_ft[CAMERA_KEY]
        assert c == 1  # mono

    def test_observation_camera_shape_from_camera_wh(self, tmp_path):
        """AC#9: camera shape must reflect camera_wh from AA-5 discovery."""
        adapter, _, _ = _build_adapter(camera_wh=(800, 600), tmp_dir=tmp_path)
        obs_ft = adapter.observation_features
        h, w, c = obs_ft[CAMERA_KEY]
        assert (h, w) == (600, 800)  # camera_wh=(width, height)

    def test_action_features_has_five_joint_floats(self, tmp_path):
        adapter, _, _ = _build_adapter(tmp_dir=tmp_path)
        act_ft = adapter.action_features
        assert set(act_ft.keys()) == set(ACTION_JOINT_KEYS)
        for key in ACTION_JOINT_KEYS:
            assert act_ft[key] is float

    def test_gripper_absent_from_action_features(self, tmp_path):
        """AC#4: gripper must be structurally absent from action_features."""
        adapter, _, _ = _build_adapter(tmp_dir=tmp_path)
        for key in adapter.action_features:
            assert "gripper" not in key, f"gripper found in action key: {key!r}"


# ---------------------------------------------------------------------------
# Tests: connect / disconnect lifecycle
# ---------------------------------------------------------------------------

class TestLifecycle:
    def test_not_connected_initially(self, tmp_path):
        adapter, _, _ = _build_adapter(tmp_dir=tmp_path)
        assert not adapter.is_connected

    def test_connect_sets_connected(self, tmp_path):
        adapter, _, _ = _build_adapter(tmp_dir=tmp_path)
        adapter.connect()
        assert adapter.is_connected

    def test_disconnect_clears_connected(self, tmp_path):
        adapter, _, _ = _build_adapter(tmp_dir=tmp_path)
        adapter.connect()
        adapter.disconnect()
        assert not adapter.is_connected

    def test_is_calibrated_always_true(self, tmp_path):
        adapter, _, _ = _build_adapter(tmp_dir=tmp_path)
        assert adapter.is_calibrated


# ---------------------------------------------------------------------------
# Tests: AC#3 — get_observation routes camera through injected frame_source
# ---------------------------------------------------------------------------

class TestGetObservation:
    def test_frame_source_called_once_per_observation(self, tmp_path):
        """AC#3: frame_source is called; adapter never opens camera itself."""
        adapter, _, frame_source = _build_adapter(tmp_dir=tmp_path)
        adapter.connect()
        adapter.get_observation()
        frame_source.assert_called_once()

    def test_observation_returns_correct_keys(self, tmp_path):
        adapter, _, _ = _build_adapter(tmp_dir=tmp_path)
        adapter.connect()
        obs = adapter.get_observation()
        for key in OBSERVATION_JOINT_KEYS:
            assert key in obs
        assert CAMERA_KEY in obs

    def test_observation_camera_shape_is_hwc1(self, tmp_path):
        """Camera observation must be (H, W, 1) uint8."""
        adapter, _, _ = _build_adapter(frame_h=480, frame_w=640, tmp_dir=tmp_path)
        adapter.connect()
        obs = adapter.get_observation()
        cam = obs[CAMERA_KEY]
        assert cam.shape == (480, 640, 1)
        assert cam.dtype == np.uint8

    def test_observation_joint_positions_come_from_guarded_arm(self, tmp_path):
        """Joints must come from GuardedArm.read_joint_deg (not hard-coded)."""
        positions = {
            "shoulder_pan": 10.0,
            "shoulder_lift": -5.0,
            "elbow_flex": 20.0,
            "wrist_flex": 3.0,
            "wrist_roll": -1.0,
        }
        adapter, guarded_arm, _ = _build_adapter(joint_positions=positions, tmp_dir=tmp_path)
        adapter.connect()
        obs = adapter.get_observation()
        for joint, pos in positions.items():
            assert obs[f"{joint}.pos"] == pytest.approx(pos)

    def test_get_observation_raises_when_not_connected(self, tmp_path):
        adapter, _, _ = _build_adapter(tmp_dir=tmp_path)
        with pytest.raises(RuntimeError, match="not connected"):
            adapter.get_observation()

    def test_2d_frame_reshaped_to_hwc1(self, tmp_path):
        """A (H, W) frame from frame_source must be reshaped to (H, W, 1)."""
        Config = _make_config_class()
        config = Config(arm_role="arm_left", frame_height=120, frame_width=160)
        guarded_arm = _make_guarded_arm_mock()
        frame_source = MagicMock(return_value=np.zeros((120, 160), dtype=np.uint8))

        AdapterCls = _make_adapter_class()
        adapter = AdapterCls(
            config,
            guarded_arm=guarded_arm,
            frame_source=frame_source,
            calibration_dir=tmp_path,
        )
        adapter.connect()
        obs = adapter.get_observation()
        assert obs[CAMERA_KEY].shape == (120, 160, 1)


# ---------------------------------------------------------------------------
# Tests: AC#4 — send_action gripper exclusion
# ---------------------------------------------------------------------------

class TestSendAction:
    def test_valid_action_dispatched_to_guarded_arm(self, tmp_path):
        """Valid 5-joint action must call apply_joint_command for each joint."""
        adapter, guarded_arm, _ = _build_adapter(tmp_dir=tmp_path)
        adapter.connect()
        action = {f"{j}.pos": 5.0 for j in ADAPTER_JOINT_ORDER}
        result = adapter.send_action(action)
        assert guarded_arm.apply_joint_command.call_count == 5
        assert result == action

    def test_gripper_key_raises_gripper_command_blocked(self, tmp_path):
        """AC#4: 'gripper.pos' in action must raise GripperCommandBlocked."""
        adapter, _, _ = _build_adapter(tmp_dir=tmp_path)
        adapter.connect()
        bad_action = {"gripper.pos": 50.0}
        with pytest.raises(GripperCommandBlocked):
            adapter.send_action(bad_action)

    def test_unknown_key_raises_gripper_command_blocked(self, tmp_path):
        """Any key not in ACTION_JOINT_KEYS must raise GripperCommandBlocked."""
        adapter, _, _ = _build_adapter(tmp_dir=tmp_path)
        adapter.connect()
        bad_action = {"unknown_joint.pos": 0.0}
        with pytest.raises(GripperCommandBlocked):
            adapter.send_action(bad_action)

    def test_send_action_raises_when_not_connected(self, tmp_path):
        adapter, _, _ = _build_adapter(tmp_dir=tmp_path)
        action = {f"{j}.pos": 0.0 for j in ADAPTER_JOINT_ORDER}
        with pytest.raises(RuntimeError, match="not connected"):
            adapter.send_action(action)

    def test_partial_action_dispatches_only_given_joints(self, tmp_path):
        """Only the joints present in action are dispatched."""
        adapter, guarded_arm, _ = _build_adapter(tmp_dir=tmp_path)
        adapter.connect()
        action = {"shoulder_pan.pos": 10.0, "shoulder_lift.pos": -5.0}
        adapter.send_action(action)
        assert guarded_arm.apply_joint_command.call_count == 2


# --- AA-22 ---
# Tests for make_arbiter_frame_source and build_adapter_from_config.
# AC#3: frames come through the arbiter lock; a second acquire while streaming
#       times out with CameraArbiterTimeoutError.
# AC#9: arm_role maps to cam_left/cam_right via discovery naming;
#       build_adapter_from_config never produces an adapter that uses an OpenCV index.

from inchworm.backends import MockMjpegBackend  # noqa: E402
from inchworm.camera_arbiter import CameraArbiter, CameraArbiterTimeoutError  # noqa: E402
from inchworm.config import load_config  # noqa: E402
from inchworm.lerobot_adapter import (  # noqa: E402
    _ARM_ROLE_TO_BACKEND_CAMERA,
    _BACKEND_CAMERA_TO_DISCOVERY_ROLE,
    make_arbiter_frame_source,
    build_adapter_from_config,
)


@pytest.fixture()
def hw_config():
    """Real hardware config (mock backend)."""
    return load_config()


@pytest.fixture()
def mock_mjpeg_backend(hw_config):
    """MockMjpegBackend instance for camera tests."""
    return MockMjpegBackend(hw_config)


@pytest.fixture()
def arbiter(mock_mjpeg_backend, tmp_path):
    """CameraArbiter backed by MockMjpegBackend."""
    lock_dir = tmp_path / "cam_locks"
    lock_dir.mkdir()
    return CameraArbiter(mock_mjpeg_backend, lock_dir=lock_dir)


class TestMakeArbiterFrameSource:
    """AC#3 — frames come through the arbiter lock."""

    def test_frames_delivered_through_arbiter_context(self, arbiter):
        """Frames are delivered inside the arbiter context; each call returns (H, W) array."""
        with make_arbiter_frame_source(
            arbiter,
            "camera_left_arm",
            cam_h=48,
            cam_w=64,
        ) as grab:
            frame = grab()
        # MockMjpegBackend produces 64×48 synthetic MJPEG → decoded to (48, 64).
        assert isinstance(frame, np.ndarray)
        assert frame.dtype == np.uint8
        assert frame.ndim == 2  # (H, W) mono

    def test_multiple_grabs_in_single_context(self, arbiter):
        """Multiple grabs within the same arbiter context all succeed."""
        with make_arbiter_frame_source(
            arbiter,
            "camera_left_arm",
            cam_h=48,
            cam_w=64,
        ) as grab:
            for _ in range(5):
                frame = grab()
                assert frame.ndim == 2

    def test_ac3_second_acquire_times_out_while_streaming(self, hw_config, tmp_path):
        """AC#3: a second CameraArbiter.acquire() for any device while the first
        lock is held must raise CameraArbiterTimeoutError (exclusive global lock).

        This verifies that the arbiter truly enforces the single-camera-at-a-time
        invariant during an active make_arbiter_frame_source episode.
        """
        lock_dir = tmp_path / "cam_locks2"
        lock_dir.mkdir()

        # Use two independent arbiters sharing the same lock directory.
        backend1 = MockMjpegBackend(hw_config)
        backend2 = MockMjpegBackend(hw_config)
        arb1 = CameraArbiter(backend1, lock_dir=lock_dir)
        arb2 = CameraArbiter(backend2, lock_dir=lock_dir)

        with make_arbiter_frame_source(arb1, "camera_left_arm", cam_h=48, cam_w=64):
            # The global lock is held; any second acquire must time out quickly.
            with pytest.raises(CameraArbiterTimeoutError):
                with arb2.acquire("camera_right_arm", timeout_s=0.1):
                    pass  # must not reach here

    def test_ac3_no_direct_opencv_call_in_frame_source(self, arbiter):
        """AC#3 structural: make_arbiter_frame_source never calls cv2.VideoCapture
        with an integer index.  The mock backend intercepts all camera opens, so
        if an integer were used it would not match any device name in the backend
        table and BackendError would be raised.  The fact that frames arrive at all
        proves the path goes through the named-device backend, not an integer.
        """
        with make_arbiter_frame_source(
            arbiter,
            "camera_left_arm",
            cam_h=48,
            cam_w=64,
        ) as grab:
            # If an integer OpenCV index were used, the backend would raise
            # BackendError("unknown device ...") on open_camera().  Reaching here
            # proves the named-device path was used.
            frame = grab()
            assert frame is not None


class TestDeviceRoleMapping:
    """AC#9 — arm_role maps to cam_left/cam_right via discovery naming."""

    def test_arm_left_maps_to_camera_left_arm(self):
        """arm_left -> camera_left_arm (backend) -> cam_left (discovery)."""
        assert _ARM_ROLE_TO_BACKEND_CAMERA["arm_left"] == "camera_left_arm"

    def test_arm_right_maps_to_camera_right_arm(self):
        """arm_right -> camera_right_arm (backend) -> cam_right (discovery)."""
        assert _ARM_ROLE_TO_BACKEND_CAMERA["arm_right"] == "camera_right_arm"

    def test_camera_left_arm_maps_to_cam_left_discovery(self):
        """backend camera_left_arm -> AA-5 discovery role 'cam_left'."""
        assert _BACKEND_CAMERA_TO_DISCOVERY_ROLE["camera_left_arm"] == "cam_left"

    def test_camera_right_arm_maps_to_cam_right_discovery(self):
        """backend camera_right_arm -> AA-5 discovery role 'cam_right'."""
        assert _BACKEND_CAMERA_TO_DISCOVERY_ROLE["camera_right_arm"] == "cam_right"

    def test_ac9_both_ov9281_arms_resolved_via_discovery_naming(self):
        """Both OV9281 cameras (which share USB serial 5423534724) must be resolved
        via usbip_busid (discovery naming), never by OpenCV integer index.
        The _ARM_ROLE_TO_BACKEND_CAMERA and _BACKEND_CAMERA_TO_DISCOVERY_ROLE maps
        together form the complete resolution chain:
          arm_left  -> camera_left_arm  -> cam_left  (usbip_busid in config)
          arm_right -> camera_right_arm -> cam_right (usbip_busid in config)
        This test verifies that chain is complete for both arms.
        """
        for arm_role in ("arm_left", "arm_right"):
            backend_role = _ARM_ROLE_TO_BACKEND_CAMERA[arm_role]
            assert backend_role in _BACKEND_CAMERA_TO_DISCOVERY_ROLE, (
                f"arm_role {arm_role!r} -> backend_role {backend_role!r} has no "
                "entry in _BACKEND_CAMERA_TO_DISCOVERY_ROLE"
            )
            discovery_role = _BACKEND_CAMERA_TO_DISCOVERY_ROLE[backend_role]
            assert discovery_role in ("cam_left", "cam_right"), (
                f"Expected discovery role cam_left/cam_right, got {discovery_role!r}"
            )


class TestBuildAdapterFromConfig:
    """AC#9 — build_adapter_from_config never uses an OpenCV index."""

    def test_returns_adapter_arbiter_and_backend_role(self, hw_config, tmp_path):
        """build_adapter_from_config returns (adapter, arbiter, backend_device_role)."""
        guarded_arm = _make_guarded_arm_mock()
        adapter, arbiter, backend_device_role = build_adapter_from_config(
            hw_config,
            "arm_left",
            guarded_arm=guarded_arm,
            calibration_dir=tmp_path,
        )
        assert backend_device_role == "camera_left_arm"
        assert isinstance(arbiter, CameraArbiter)

    def test_ac9_arm_right_backend_role_is_camera_right_arm(self, hw_config, tmp_path):
        """AC#9: arm_right -> camera_right_arm, never an integer index."""
        guarded_arm = _make_guarded_arm_mock()
        adapter, arbiter, backend_device_role = build_adapter_from_config(
            hw_config,
            "arm_right",
            guarded_arm=guarded_arm,
            calibration_dir=tmp_path,
        )
        assert backend_device_role == "camera_right_arm"

    def test_ac9_adapter_frame_source_requires_arbiter_context(self, hw_config, tmp_path):
        """AC#9: the adapter returned by build_adapter_from_config raises an error
        if get_observation() is called without first entering make_arbiter_frame_source().
        This prevents any path that bypasses the arbiter (including OpenCV by-index).
        """
        guarded_arm = _make_guarded_arm_mock()
        adapter, arbiter, backend_device_role = build_adapter_from_config(
            hw_config,
            "arm_left",
            guarded_arm=guarded_arm,
            calibration_dir=tmp_path,
        )
        adapter.connect()
        # Without make_arbiter_frame_source, get_observation must raise RuntimeError.
        with pytest.raises(RuntimeError, match="make_arbiter_frame_source"):
            adapter.get_observation()

    def test_ac3_adapter_built_from_config_frames_through_arbiter(
        self, hw_config, tmp_path
    ):
        """AC#3+AC#9: end-to-end: adapter from build_adapter_from_config receives
        frames through the arbiter lock, never through a raw OpenCV index.

        The MockMjpegBackend is used; BackendError would be raised immediately if
        any code path tried to open a camera by integer index.
        """
        guarded_arm = _make_guarded_arm_mock()
        adapter, arbiter, backend_device_role = build_adapter_from_config(
            hw_config,
            "arm_left",
            guarded_arm=guarded_arm,
            calibration_dir=tmp_path,
        )
        adapter.connect()

        with make_arbiter_frame_source(
            arbiter,
            backend_device_role,
            cam_h=48,
            cam_w=64,
        ) as grab:
            adapter._frame_source = grab
            obs = adapter.get_observation()

        assert "wrist_cam" in obs
        cam = obs["wrist_cam"]
        # After reshape in get_observation: (H, W, 1)
        assert cam.ndim == 3
        assert cam.shape[2] == 1
        assert cam.dtype == np.uint8

    def test_build_adapter_from_config_invalid_arm_role_raises(
        self, hw_config, tmp_path
    ):
        """Invalid arm_role raises ValueError."""
        guarded_arm = _make_guarded_arm_mock()
        with pytest.raises(ValueError, match="arm_role"):
            build_adapter_from_config(
                hw_config,
                "arm_middle",
                guarded_arm=guarded_arm,
                calibration_dir=tmp_path,
            )

# --- AA-22 end ---
