"""AA-22: LeRobot SO-101 adapter for the inchworm robot.

Exposes the SO-101 arm to a LeRobot 0.6.x ``Robot`` subclass whilst routing
all I/O through the project's own safety layers:

- Joint writes  → AA-9 :class:`~inchworm.motion_guard.GuardedArm`
- Camera frames → AA-6 :class:`~inchworm.camera_arbiter.CameraArbiter` +
                  AA-14 :class:`~inchworm.camera_capture.open_capture`
- Camera device → AA-5 :func:`~inchworm.discovery.require_device` (usbipd
                  BusId, never OpenCV index)
- Action/obs contract → AA-20 :mod:`inchworm.policy_contract`

Importable WITHOUT lerobot installed
-------------------------------------
All ``lerobot.*`` imports are guarded under a ``TYPE_CHECKING`` block or a
lazy ``_try_import_lerobot()`` helper.  The module can be imported in the
main Python 3.10 environment where lerobot is absent; only instantiating
:class:`InchwormLeRobotAdapter` requires lerobot to be present.

Observation features
---------------------
- Five joint position floats, keys ``"<joint>.pos"`` ordered
  ``shoulder_pan.pos`` … ``wrist_roll.pos`` (proximal → distal, gripper
  **excluded**, AC#4).
- One wrist camera image with key ``"wrist_cam"``, shape ``(H, W, 1)``
  uint8 mono.  Shape ``(H, W, 1)`` is chosen so LeRobot's
  ``hw_to_dataset_features`` treats it as a single-channel image (flagged
  as a depth map internally; that convention is documented here and is the
  correct choice for a mono greyscale OV9281 frame — see lerobot
  ``feature_utils.hw_to_dataset_features``).

Action features
---------------
Five joint floats only; keys match :data:`~inchworm.policy_contract.JOINT_ORDER`
with ``.pos`` suffix.  The gripper key is **structurally absent**, not just
filtered at call time (AC#4).

Contract mismatch
-----------------
At construction time the adapter verifies that
:data:`~inchworm.policy_contract.N_JOINTS` == 5 and
:data:`~inchworm.policy_contract.JOINT_ORDER` matches the expected ordered
tuple.  A mismatch raises :class:`AdapterContractMismatch` (AC#5).

Frame convention
----------------
Observations are delivered in the **chassis-fixed frame** (base_link of the
commanded arm).  The camera is the wrist camera of the arm selected by *role*.
"""

from __future__ import annotations

import contextlib
import logging
from pathlib import Path
from typing import TYPE_CHECKING, Any, Callable, Dict, Iterator, Optional, Tuple

import numpy as np

from inchworm.policy_contract import (
    CONTRACT_VERSION,
    JOINT_ORDER,
    N_JOINTS,
    DEFAULT_FRAME_HEIGHT,
    DEFAULT_FRAME_WIDTH,
    POLICY_JOINT_NAMES,
)

if TYPE_CHECKING:  # pragma: no cover
    from lerobot.robots.robot import Robot as _LeRobotBase
    from lerobot.robots.config import RobotConfig as _LeRobotConfig
    from lerobot.lerobot_types import RobotAction, RobotObservation

__all__ = [
    "AdapterContractMismatch",
    "ADAPTER_JOINT_ORDER",
    "OBSERVATION_JOINT_KEYS",
    "ACTION_JOINT_KEYS",
    "CAMERA_KEY",
    "CAMERA_SHAPE",
    "InchwormLeRobotAdapter",
    "InchwormLeRobotConfig",
    "make_arbiter_frame_source",
    "build_adapter_from_config",
]

log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Adapter-level constants (derived from AA-20 contract)
# ---------------------------------------------------------------------------

#: Canonical joint order for the adapter (matches :data:`inchworm.policy_contract.JOINT_ORDER`).
ADAPTER_JOINT_ORDER: Tuple[str, ...] = JOINT_ORDER  # 5-tuple, gripper absent

#: Observation feature keys for joints (``"<name>.pos"`` for each joint).
OBSERVATION_JOINT_KEYS: Tuple[str, ...] = tuple(f"{j}.pos" for j in ADAPTER_JOINT_ORDER)

#: Action feature keys for joints (same set, gripper structurally absent).
ACTION_JOINT_KEYS: Tuple[str, ...] = OBSERVATION_JOINT_KEYS

#: Camera observation key.
CAMERA_KEY: str = "wrist_cam"

#: Default camera shape ``(H, W, C)`` for the mono OV9281.
#: ``C=1`` so LeRobot ``hw_to_dataset_features`` treats it as a single-channel
#: (depth-map) image — the correct choice for a greyscale uint8 frame.
CAMERA_SHAPE: Tuple[int, int, int] = (DEFAULT_FRAME_HEIGHT, DEFAULT_FRAME_WIDTH, 1)


# ---------------------------------------------------------------------------
# Contract mismatch guard (AC#5)
# ---------------------------------------------------------------------------


class AdapterContractMismatch(RuntimeError):
    """Raised when the AA-20 contract constants do not match the adapter's expectations.

    This fires at construction time so version drift is caught at import rather
    than silently producing wrong-shaped data.
    """


def _check_contract() -> None:
    """Validate AA-20 contract constants at adapter construction time (AC#5)."""
    if N_JOINTS != 5:
        raise AdapterContractMismatch(
            f"AA-20 contract N_JOINTS={N_JOINTS}, adapter expects 5. "
            "Update AA-22 adapter to match."
        )
    expected = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
    if JOINT_ORDER != expected:
        raise AdapterContractMismatch(
            f"AA-20 contract JOINT_ORDER={JOINT_ORDER!r} does not match "
            f"adapter expected {expected!r}. Update AA-22 adapter to match."
        )
    if not POLICY_JOINT_NAMES.issuperset(expected):
        raise AdapterContractMismatch(
            f"AA-20 POLICY_JOINT_NAMES={POLICY_JOINT_NAMES!r} does not cover "
            f"all expected joints {expected!r}."
        )


# ---------------------------------------------------------------------------
# Lazy lerobot import helper
# ---------------------------------------------------------------------------


def _require_lerobot() -> Any:
    """Import and return the lerobot.robots.robot module, raising ImportError if absent."""
    try:
        import lerobot.robots.robot as _m  # noqa: PLC0415
        return _m
    except ImportError as exc:
        raise ImportError(
            "lerobot is not installed. Activate the lerobot_env virtual environment:\n"
            "  cd robot/lerobot_env && uv run python ..."
        ) from exc


def _require_lerobot_config() -> Any:
    """Import and return lerobot.robots.config module."""
    try:
        import lerobot.robots.config as _m  # noqa: PLC0415
        return _m
    except ImportError as exc:
        raise ImportError(
            "lerobot is not installed. Activate the lerobot_env environment."
        ) from exc


# ---------------------------------------------------------------------------
# InchwormLeRobotConfig
# ---------------------------------------------------------------------------


def _make_config_class() -> type:
    """Build and return InchwormLeRobotConfig, a draccus-registered RobotConfig subclass."""
    cfg_mod = _require_lerobot_config()
    RobotConfig = cfg_mod.RobotConfig  # type: ignore[attr-defined]

    import dataclasses  # noqa: PLC0415

    @dataclasses.dataclass(kw_only=True)
    class InchwormLeRobotConfig(RobotConfig):
        """Configuration for :class:`InchwormLeRobotAdapter`.

        Parameters
        ----------
        arm_role:
            Which arm to expose (``"arm_left"`` or ``"arm_right"``).
        frame_height:
            Camera frame height in pixels (must match the resolution used
            during training). Default: :data:`DEFAULT_FRAME_HEIGHT` (480).
        frame_width:
            Camera frame width in pixels. Default: :data:`DEFAULT_FRAME_WIDTH` (640).
        """

        arm_role: str = "arm_left"
        frame_height: int = DEFAULT_FRAME_HEIGHT
        frame_width: int = DEFAULT_FRAME_WIDTH

    return InchwormLeRobotConfig


# ---------------------------------------------------------------------------
# InchwormLeRobotAdapter
# ---------------------------------------------------------------------------


def _build_adapter_class() -> type:
    """Dynamically build InchwormLeRobotAdapter with lerobot.robots.robot.Robot as base."""
    lm = _require_lerobot()
    Robot = lm.Robot  # type: ignore[attr-defined]

    class InchwormLeRobotAdapter(Robot):
        """LeRobot Robot adapter for one SO-101 arm of the inchworm robot.

        Routes all I/O through project safety layers; never opens devices
        directly.

        Parameters
        ----------
        config:
            :class:`InchwormLeRobotConfig` (arm_role, frame_height, frame_width).
        guarded_arm:
            AA-9 :class:`~inchworm.motion_guard.GuardedArm` for the arm.
            Required for :meth:`send_action`.
        frame_source:
            Callable ``() -> np.ndarray`` returning a uint8 mono frame of shape
            ``(H, W)``.  Must be constructed from AA-6/AA-14 by the caller.
            AC#3: this adapter never opens a camera; the caller enforces the
            arbiter lock.
        camera_wh:
            ``(width, height)`` tuple resolved via AA-5 discovery.  When
            ``None``, defaults to ``(frame_width, frame_height)`` from config.
        calibration_dir:
            Override for the lerobot calibration directory.  Defaults to a
            tmp path so tests don't write to the filesystem.
        """

        name: str = "inchworm_so101"

        # Populated lazily by _make_config_class(); set at class definition time
        # in _build_adapter_class so subclass tests can override.
        config_class = None  # replaced below

        def __init__(
            self,
            config: Any,
            *,
            guarded_arm: Any,
            frame_source: Callable[[], np.ndarray],
            camera_wh: Optional[Tuple[int, int]] = None,
            calibration_dir: Optional[Path] = None,
        ) -> None:
            # Contract check at construction time (AC#5).
            _check_contract()

            if calibration_dir is None:
                import tempfile  # noqa: PLC0415
                calibration_dir = Path(tempfile.mkdtemp(prefix="inchworm_lerobot_cal_"))

            # Override calibration_dir so Robot.__init__ writes there.
            object.__setattr__(config, "calibration_dir", calibration_dir)

            super().__init__(config)

            self._config = config
            self._guarded_arm = guarded_arm
            self._frame_source = frame_source
            self._connected = False

            # Resolve camera shape (AC#9): prefer camera_wh from AA-5 discovery.
            w = camera_wh[0] if camera_wh else config.frame_width
            h = camera_wh[1] if camera_wh else config.frame_height
            self._cam_h: int = h
            self._cam_w: int = w

        # ------------------------------------------------------------------
        # observation_features / action_features
        # ------------------------------------------------------------------

        @property
        def observation_features(self) -> Dict[str, Any]:
            """Observation space: 5 joint floats + 1 mono camera (H, W, 1).

            Camera shape uses ``(H, W, 1)`` so LeRobot ``hw_to_dataset_features``
            treats it as a single-channel image.  The shape is resolved from AA-5
            discovery at construction time (AC#9).

            Gripper is **structurally absent** (AC#4).
            """
            obs: Dict[str, Any] = {key: float for key in OBSERVATION_JOINT_KEYS}
            obs[CAMERA_KEY] = (self._cam_h, self._cam_w, 1)
            return obs

        @property
        def action_features(self) -> Dict[str, Any]:
            """Action space: 5 joint floats only, gripper **structurally absent** (AC#4)."""
            return {key: float for key in ACTION_JOINT_KEYS}

        # ------------------------------------------------------------------
        # Connection / calibration
        # ------------------------------------------------------------------

        @property
        def is_connected(self) -> bool:
            return self._connected

        def connect(self, calibrate: bool = True) -> None:
            """Mark the adapter as connected (device connection handled externally)."""
            if self._connected:
                log.warning("InchwormLeRobotAdapter.connect() called while already connected")
                return
            self._connected = True
            log.info("InchwormLeRobotAdapter connected (role=%s)", self._config.arm_role)

        @property
        def is_calibrated(self) -> bool:
            """Always True: calibration is managed by AA-8/AA-9, not by LeRobot."""
            return True

        def calibrate(self) -> None:
            """No-op: calibration is managed by the AA-8/AA-9 subsystems."""
            log.debug(
                "InchwormLeRobotAdapter.calibrate() called — no-op; "
                "calibration is managed by AA-8/AA-9."
            )

        def configure(self) -> None:
            """No-op: arm configuration is managed by the AA-8/AA-9 subsystems."""

        def disconnect(self) -> None:
            self._connected = False
            log.info("InchwormLeRobotAdapter disconnected (role=%s)", self._config.arm_role)

        # ------------------------------------------------------------------
        # get_observation / send_action
        # ------------------------------------------------------------------

        def get_observation(self) -> Dict[str, Any]:
            """Return current observation.

            Returns
            -------
            dict
                Keys: ``"<joint>.pos"`` floats (5 joints, gripper absent) and
                ``"wrist_cam"`` mono uint8 ndarray of shape ``(H, W, 1)``.

            Raises
            ------
            RuntimeError
                If the adapter is not connected.
            """
            if not self._connected:
                raise RuntimeError(
                    "InchwormLeRobotAdapter.get_observation() called while not connected"
                )

            # --- joint positions via guarded arm -------------------------
            obs: Dict[str, Any] = {}
            for joint in ADAPTER_JOINT_ORDER:
                pos_deg = self._guarded_arm.read_joint_deg(joint)  # type: ignore[attr-defined]
                obs[f"{joint}.pos"] = float(pos_deg)

            # --- camera frame via injected frame_source (AA-6/AA-14) -----
            # AC#3: frame_source must have been constructed inside an arbiter
            # lock; this adapter never opens a camera directly.
            raw: np.ndarray = self._frame_source()
            if raw.ndim == 2:
                # Contract (AA-20) delivers (H, W); reshape to (H, W, 1) for LeRobot.
                frame = raw[:, :, np.newaxis]
            elif raw.ndim == 3 and raw.shape[2] == 1:
                frame = raw
            else:
                raise ValueError(
                    f"frame_source returned unexpected shape {raw.shape}; "
                    "expected (H, W) or (H, W, 1) uint8 mono."
                )
            obs[CAMERA_KEY] = frame

            return obs

        def send_action(self, action: Dict[str, Any]) -> Dict[str, Any]:
            """Send joint action to the arm via AA-9 GuardedArm.

            Only keys in :data:`ACTION_JOINT_KEYS` are accepted.  Any gripper
            key raises :class:`~inchworm.policy_actions.GripperCommandBlocked`
            (via the GuardedArm) (AC#4).

            Parameters
            ----------
            action:
                Dict mapping ``"<joint>.pos"`` → float (degrees).

            Returns
            -------
            dict
                The action dict as sent (after GuardedArm acceptance).
            """
            if not self._connected:
                raise RuntimeError(
                    "InchwormLeRobotAdapter.send_action() called while not connected"
                )

            # Validate keys: only ACTION_JOINT_KEYS are accepted.
            allowed_set = set(ACTION_JOINT_KEYS)
            extra = {k for k in action if k not in allowed_set}
            if extra:
                from inchworm.policy_actions import GripperCommandBlocked  # noqa: PLC0415
                raise GripperCommandBlocked(sorted(extra))

            # Dispatch each joint via GuardedArm (AA-9 enforcement).
            sent: Dict[str, Any] = {}
            for key, val in action.items():
                joint = key.removesuffix(".pos")
                self._guarded_arm.apply_joint_command(joint, float(val))  # type: ignore[attr-defined]
                sent[key] = val

            return sent

    return InchwormLeRobotAdapter


# ---------------------------------------------------------------------------
# Public factory — build the class with lerobot available
# ---------------------------------------------------------------------------


def _make_adapter_class() -> type:
    """Return InchwormLeRobotAdapter with all lerobot base classes resolved.

    Call this in lerobot_env; in the main 3.10 env this will raise ImportError
    if lerobot is absent.
    """
    cfg_class = _make_config_class()
    adapter = _build_adapter_class()
    adapter.config_class = cfg_class  # type: ignore[attr-defined]
    return adapter


# ---------------------------------------------------------------------------
# Convenience: expose stable names without requiring lerobot at import time
# ---------------------------------------------------------------------------

def InchwormLeRobotConfig(*args: Any, **kwargs: Any) -> Any:  # type: ignore[misc]
    """Convenience factory: returns an InchwormLeRobotConfig instance.

    Requires lerobot to be installed.  Deferred to runtime so the module can
    be imported in the main 3.10 env.
    """
    cls = _make_config_class()
    return cls(*args, **kwargs)


def InchwormLeRobotAdapter(*args: Any, **kwargs: Any) -> Any:  # type: ignore[misc]
    """Convenience factory: returns an InchwormLeRobotAdapter instance.

    Requires lerobot to be installed.  Deferred to runtime so the module can
    be imported in the main 3.10 env.
    """
    cls = _make_adapter_class()
    return cls(*args, **kwargs)


# --- AA-22 ---
# Arbiter-backed frame-source factory and config-driven adapter builder.
#
# AC#3: Camera access from the LeRobot path ALWAYS goes through the USB arbiter
#        (AA-6 CameraArbiter). Production code must call make_arbiter_frame_source
#        to get a frame_source; it may never pass an arbitrary callable.
# AC#9: Camera devices are resolved through AA-5 discovery (usbipd BusId),
#        never by raw OpenCV integer index. build_adapter_from_config enforces
#        this by mapping arm_role → backend_device_role internally.

#: Map from arm role to the backend camera device role used by Backend/CameraArbiter.
#: cam_left/cam_right in discovery naming both use usbip_busid (never by OpenCV index).
_ARM_ROLE_TO_BACKEND_CAMERA: Dict[str, str] = {
    "arm_left": "camera_left_arm",
    "arm_right": "camera_right_arm",
}

#: Map from backend camera device role to the AA-5 discovery role.
#: V4L2CameraTransport.open() already performs this mapping internally;
#: this copy is here so tests can assert the mapping without re-opening hardware.
_BACKEND_CAMERA_TO_DISCOVERY_ROLE: Dict[str, str] = {
    "camera_left_arm": "cam_left",
    "camera_right_arm": "cam_right",
    "camera_scanner": "cam_scanner",
}


@contextlib.contextmanager
def make_arbiter_frame_source(
    arbiter: Any,
    backend_device_role: str,
    settings: Optional[Any] = None,
    *,
    cam_h: int = CAMERA_SHAPE[0],
    cam_w: int = CAMERA_SHAPE[1],
) -> Iterator[Callable[[], "np.ndarray"]]:
    """Context manager: acquire the AA-6 CameraArbiter once per episode and yield
    a frame-source callable that returns grayscale frames shaped ``(H, W)``.

    The arbiter lock is acquired **once** for the lifetime of the episode (per
    the ``with`` block), NOT once per frame.  This is the only correct usage of
    :class:`~inchworm.camera_arbiter.CameraArbiter` for the adapter: the OV9281
    must stay open across all observation steps of one episode.

    AC#3: every camera frame in the LeRobot path is gated by the AA-6 global
    exclusive camera lock before the transport is opened.  No caller may bypass
    this by passing a raw ``cv2.VideoCapture`` or OpenCV integer index to the
    adapter.

    AC#9: the AA-5 :func:`~inchworm.discovery.require_device` resolution happens
    inside the AA-6 :meth:`~inchworm.camera_arbiter.CameraArbiter.acquire` call
    (for the mock backend the lock is still acquired so the same code path runs;
    for the hardware backend V4L2CameraTransport.open() calls require_device).

    Parameters
    ----------
    arbiter:
        An :class:`~inchworm.camera_arbiter.CameraArbiter` instance built from
        the project backend (never instantiated with a raw camera path).
    backend_device_role:
        Backend camera device role name, one of ``"camera_left_arm"``,
        ``"camera_right_arm"``, or ``"camera_scanner"``.  The arbiter resolves
        this role through AA-5 discovery (usbipd BusId, never OpenCV index).
    settings:
        Optional :class:`~inchworm.camera_capture.CameraSettings` applied after
        opening the transport.  ``None`` leaves the transport at its power-on
        defaults.
    cam_h:
        Expected frame height (pixels); default :data:`CAMERA_SHAPE` ``[0]``.
    cam_w:
        Expected frame width (pixels); default :data:`CAMERA_SHAPE` ``[1]``.

    Yields
    ------
    Callable[[], np.ndarray]
        Zero-argument callable.  Each call grabs one frame from the open
        transport and returns a ``(H, W)`` uint8 mono ndarray.  The arbiter
        lock is held for the duration of the ``with`` block.

    Raises
    ------
    CameraArbiterTimeoutError
        If the exclusive global camera lock cannot be acquired.
    """
    from inchworm.camera_capture import open_capture  # noqa: PLC0415

    with arbiter.acquire(backend_device_role) as transport:
        capture = open_capture(transport, role=backend_device_role)
        if settings is not None:
            capture.apply_settings(settings)

        def _grab() -> "np.ndarray":
            result = capture.grab()
            frame = result.data  # shape (H, W), uint8
            return frame

        yield _grab


def build_adapter_from_config(
    cfg: Any,
    arm_role: str,
    *,
    guarded_arm: Any,
    settings: Optional[Any] = None,
    calibration_dir: Optional[Path] = None,
    camera_wh: Optional[Tuple[int, int]] = None,
) -> Any:
    """Build an :class:`InchwormLeRobotAdapter` from a
    :class:`~inchworm.config.HardwareConfig`.

    This is the **production** entry point.  It maps *arm_role* to the correct
    backend camera device role, creates a
    :class:`~inchworm.camera_arbiter.CameraArbiter` from the config's backend,
    and returns the adapter.  The caller must use the adapter inside a
    :func:`make_arbiter_frame_source` context to stream camera frames.

    AC#9: *arm_role* → backend device role → AA-5 discovery naming is
    enforced here.  The caller never supplies a raw OpenCV index or by-id path.

    Parameters
    ----------
    cfg:
        A :class:`~inchworm.config.HardwareConfig` (loaded from
        ``hardware.yaml`` via :func:`~inchworm.config.load_config`).
    arm_role:
        ``"arm_left"`` or ``"arm_right"``.
    guarded_arm:
        AA-9 :class:`~inchworm.motion_guard.GuardedArm` for the arm.
    settings:
        Optional :class:`~inchworm.camera_capture.CameraSettings` to apply
        when opening the camera inside :func:`make_arbiter_frame_source`.
    calibration_dir:
        Override for the calibration directory; defaults to a temp directory.
    camera_wh:
        ``(width, height)`` override from AA-5 discovery.  ``None`` means use
        the frame dimensions from the adapter config defaults.

    Returns
    -------
    tuple[adapter, arbiter, backend_device_role]
        *adapter*: uninitialised :class:`InchwormLeRobotAdapter` (call
        :meth:`~InchwormLeRobotAdapter.connect` before use).
        *arbiter*: :class:`~inchworm.camera_arbiter.CameraArbiter` to pass to
        :func:`make_arbiter_frame_source`.
        *backend_device_role*: the resolved backend camera role name (e.g.
        ``"camera_left_arm"``).

    Raises
    ------
    ValueError
        If *arm_role* is not ``"arm_left"`` or ``"arm_right"``.
    """
    from inchworm.backends import create_backend  # noqa: PLC0415
    from inchworm.camera_arbiter import CameraArbiter  # noqa: PLC0415

    if arm_role not in _ARM_ROLE_TO_BACKEND_CAMERA:
        raise ValueError(
            f"arm_role must be 'arm_left' or 'arm_right', got {arm_role!r}. "
            f"Known roles: {list(_ARM_ROLE_TO_BACKEND_CAMERA)}"
        )

    backend_device_role = _ARM_ROLE_TO_BACKEND_CAMERA[arm_role]
    backend = create_backend(cfg)
    arbiter = CameraArbiter(backend)

    # Determine camera dimensions from config (used when camera_wh is not given).
    # This avoids any reliance on live discovery for the shape alone.
    if camera_wh is None:
        if arm_role == "arm_left":
            _cam_cfg = cfg.cameras.left_arm
        else:
            _cam_cfg = cfg.cameras.right_arm
        # Use the adapter/contract defaults; actual live resolution comes from
        # apply_settings inside make_arbiter_frame_source.
        frame_h: int = CAMERA_SHAPE[0]
        frame_w: int = CAMERA_SHAPE[1]
    else:
        frame_w, frame_h = camera_wh

    Config = _make_config_class()
    config = Config(
        arm_role=arm_role,
        frame_height=frame_h,
        frame_width=frame_w,
    )

    # Placeholder frame_source: a closed-over stub that raises if called before
    # make_arbiter_frame_source is entered.  Production callers must enter the
    # context manager before calling get_observation().  This prevents silent
    # use of a raw camera handle.
    def _unbound_frame_source() -> "np.ndarray":
        raise RuntimeError(
            "InchwormLeRobotAdapter.get_observation() called outside of a "
            "make_arbiter_frame_source() context.  Wrap the episode loop inside:\n"
            "  with make_arbiter_frame_source(arbiter, backend_device_role) as fs:\n"
            "      adapter._frame_source = fs\n"
            "      for step in episode: adapter.get_observation()"
        )

    AdapterCls = _make_adapter_class()
    adapter = AdapterCls(
        config,
        guarded_arm=guarded_arm,
        frame_source=_unbound_frame_source,
        camera_wh=camera_wh,
        calibration_dir=calibration_dir,
    )

    return adapter, arbiter, backend_device_role


# --- AA-22 end ---
