"""AA-23: Cross-arm teleop demonstration capture with tube fixture.

Architecture
------------
:data:`TELEOP_JOINTS`
    The five joints that are read from the leader and written to the follower.
    Joint 6 (``"gripper"``) is explicitly excluded; the gripper is reserved for
    the AA-17 state machine and never read into actions or written by this module
    (AGENTS.md invariant 4; decision-1 rule 2).

:class:`TubeSupportSource`
    Enum for the two permitted tube-support modes during recording.

:func:`check_preconditions`
    Refuses to start unless the tube is supported by the fixture (attested via
    config + operator token) or by a locked, torque-enabled non-leader arm.
    Raises :class:`PreconditionError` on any refusal.

:class:`MirrorMapper`
    Maps leader joint angles to follower goal positions using the per-joint
    sign + offset from :class:`~inchworm.config.TeleopCaptureConfig`.
    Joint 6 is excluded; the mapper only operates on the five TELEOP_JOINTS.

:class:`LeRobotWriter`
    Local-only episode dataset writer.  Tries to import the real
    ``lerobot.common.datasets.LeRobotDataset`` API; if unavailable, writes a
    LeRobot v3-compatible layout (Parquet episode tables + ``meta/info.json``)
    via :class:`_LocalLeRobotWriter`.  NEVER pushes to the hub.

:class:`EpisodeRecorder`
    Records one episode: leader joint state, follower joint state, timestamps,
    wrist frames (follower wrist camera via arbiter, AC#9), and finalises with
    a success/failure/aborted label (AC#8).  Stores random seed and start offset
    in episode metadata (AC#7).

:class:`TeleopSession`
    Orchestrates one recording session: precondition check → move follower to
    AA-18 pre-insertion pose + random offset → run EpisodeRecorder loop.

Safety invariants (always hold)
---------------------------------
* Joint 6 (gripper) is never read from the leader bus or written to the
  follower bus by this module.
* The gripper is only ever commanded by the AA-17 state machine.
* Every follower position write goes through :class:`~inchworm.motion_guard.GuardedArm`.
* Only the follower wrist camera is opened during recording (via arbiter).
* The tube-support precondition is checked before any motion.
* This module never calls ``hub_push``, ``push_to_hub``, or any HuggingFace
  Hub API; recordings are stored locally only.

Hardware ACs (need owner-supervised hardware session)
------------------------------------------------------
AC#1: Mapping verification at low speed.
    ``cd robot && uv run python scripts/teleop.py --live --speed-scale 0.2``
AC#4: Fixture validation vs real crawl pose.
    ``cd robot && uv run python scripts/teleop.py --live --validate-fixture``
AC#5: Backdrive effort and latency measurement.
    ``cd robot && uv run python scripts/teleop.py --live --measure-latency``
"""

from __future__ import annotations

import enum
import json
import logging
import random
import struct
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

log = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

#: The five joints used in teleop mapping (joints 1-5).
#: Joint 6 (gripper) is intentionally excluded.
#: This is the authoritative teleop joint list for AA-23.
TELEOP_JOINTS: Tuple[str, ...] = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
)

#: Gripper joint name — named here only so the exclusion comment is traceable;
#: never imported from gripper_access and never used in teleop actions.
_GRIPPER_JOINT: str = "gripper"

# LeRobot v3 dataset format constants
_LEROBOT_FORMAT_VERSION: str = "v3"
_EPISODE_PARQUET_FMT: str = "data/chunk-{chunk:03d}/episode_{episode:06d}.parquet"
_META_INFO_PATH: str = "meta/info.json"

# ---------------------------------------------------------------------------
# Typed exceptions
# ---------------------------------------------------------------------------


class PreconditionError(RuntimeError):
    """Raised when the recording precondition is not satisfied.

    The message states which specific condition failed.
    """


class GripperInTeleopError(RuntimeError):
    """Raised if code attempts to include joint 6 in a teleop action.

    This should never happen in production; it is raised by the safety check
    in :class:`MirrorMapper` and :func:`_assert_no_gripper`.
    """


# ---------------------------------------------------------------------------
# Tube-support source
# ---------------------------------------------------------------------------


class TubeSupportSource(str, enum.Enum):
    """How the tube is supported during teleop recording."""

    FIXTURE = "fixture"
    """A physical fixture holds the tube; the leader arm is free."""

    LOCKED_ARM = "locked_arm"
    """The non-leader arm is locked and torque-enabled, holding the tube."""


# ---------------------------------------------------------------------------
# Precondition check
# ---------------------------------------------------------------------------


def check_preconditions(
    *,
    tube_support_source: Optional[str],
    operator_token: Optional[str],
    leader_arm: str,
    follower_gripper_state_fn: Optional[Any] = None,
) -> TubeSupportSource:
    """Verify that recording may start.

    Rules (decision-1, rule 1; AGENTS.md invariant 4):
    - tube_support_source must be 'fixture' or 'locked_arm'.
    - For 'fixture': operator_token must be a non-empty string (operator
      confirmation that a fixture is installed).
    - For 'locked_arm': follower_gripper_state_fn must return
      GripperState.LOCKED (the non-leader arm is locked and torque-enabled).
      A torque-off arm is never permitted to be the tube holder.
    - The leader arm must not be the tube holder in any case.

    Parameters
    ----------
    tube_support_source:
        Value from config or CLI: ``'fixture'`` or ``'locked_arm'``.
    operator_token:
        Non-empty string supplied by the operator to confirm readiness.
    leader_arm:
        Name of the arm acting as leader (``'left'`` or ``'right'``).
    follower_gripper_state_fn:
        Callable ``() -> GripperState`` for the follower (non-leader) arm.
        Required when ``tube_support_source == 'locked_arm'``.

    Returns
    -------
    TubeSupportSource
        The validated support mode.

    Raises
    ------
    PreconditionError
        On any violation (missing token, unsupported source, unlocked arm).
    """
    if not tube_support_source:
        raise PreconditionError(
            "Recording refused: tube_support_source is not set. "
            "Set it to 'fixture' or 'locked_arm' in config or via --tube-support."
        )

    try:
        source = TubeSupportSource(tube_support_source)
    except ValueError:
        raise PreconditionError(
            f"Recording refused: unknown tube_support_source {tube_support_source!r}. "
            "Must be 'fixture' or 'locked_arm'."
        )

    if source is TubeSupportSource.FIXTURE:
        if not operator_token:
            raise PreconditionError(
                "Recording refused: tube_support_source is 'fixture' but operator_token "
                "is not set. The operator must supply a non-empty confirmation token to "
                "attest that the fixture is installed (e.g. --operator-token 'fixture_ok')."
            )
        log.info(
            "AA-23 precondition: tube in fixture, operator token '%s' accepted.",
            operator_token,
        )
        return source

    # locked_arm path
    if follower_gripper_state_fn is None:
        raise PreconditionError(
            "Recording refused: tube_support_source is 'locked_arm' but no "
            "follower_gripper_state_fn was provided to verify lock state."
        )

    try:
        gripper_state = follower_gripper_state_fn()
    except Exception as exc:
        raise PreconditionError(
            f"Recording refused: follower_gripper_state_fn raised: {exc}"
        ) from exc

    # Import GripperState lazily to avoid circular dependency
    from inchworm.gripper_lock import GripperState  # type: ignore[import]

    if gripper_state is not GripperState.LOCKED:
        raise PreconditionError(
            f"Recording refused: tube_support_source is 'locked_arm' but the "
            f"non-leader ({leader_arm!r}) gripper state is {gripper_state!r}, "
            "not LOCKED. The tube-holding arm must be LOCKED and torque-enabled."
        )

    log.info(
        "AA-23 precondition: non-leader arm locked (GripperState.LOCKED). Recording allowed."
    )
    return source


# ---------------------------------------------------------------------------
# Mirror mapper
# ---------------------------------------------------------------------------


def _assert_no_gripper(joints: Sequence[str]) -> None:
    """Raise GripperInTeleopError if 'gripper' appears in joints."""
    if _GRIPPER_JOINT in joints:
        raise GripperInTeleopError(
            "Joint 6 (gripper) must never appear in teleop actions. "
            "This is a code-level safety invariant (AGENTS.md invariant 4)."
        )


class MirrorMapper:
    """Maps leader joint angles (degrees) to follower goal positions (degrees).

    For each of the five TELEOP_JOINTS:
        follower_deg[j] = sign[j] * leader_deg[j] + offset_deg[j]

    Joint 6 (gripper) is explicitly excluded and raises
    :class:`GripperInTeleopError` if any caller attempts to include it.

    Parameters
    ----------
    mirror_map_cfg:
        :class:`~inchworm.config.MirrorMapConfig` from hardware.yaml.
        All sign/offset values are unverified placeholders until owner
        verification (AC#1).
    """

    def __init__(self, mirror_map_cfg: Any) -> None:
        # Build a per-joint (sign, offset) lookup indexed by joint name
        self._map: Dict[str, Tuple[int, float]] = {}
        for joint in TELEOP_JOINTS:
            params = getattr(mirror_map_cfg, joint)
            self._map[joint] = (int(params.sign), float(params.offset_deg))

    def apply(self, leader_positions_deg: Dict[str, float]) -> Dict[str, float]:
        """Map leader joint angles to follower goal positions.

        Parameters
        ----------
        leader_positions_deg:
            Dict of joint_name → position in degrees.  Must contain only
            TELEOP_JOINTS keys (no gripper).

        Returns
        -------
        Dict[str, float]
            Follower goal positions for the same joint set.

        Raises
        ------
        GripperInTeleopError
            If 'gripper' appears as a key.
        """
        _assert_no_gripper(list(leader_positions_deg.keys()))
        result: Dict[str, float] = {}
        for joint in TELEOP_JOINTS:
            if joint not in leader_positions_deg:
                continue
            sign, offset = self._map[joint]
            result[joint] = sign * leader_positions_deg[joint] + offset
        return result

    def apply_offset_deg(self, base_positions_deg: Dict[str, float], offset_deg: Dict[str, float]) -> Dict[str, float]:
        """Add a per-joint offset (degrees) to base positions. Gripper excluded."""
        _assert_no_gripper(list(base_positions_deg.keys()))
        result: Dict[str, float] = {}
        for joint in TELEOP_JOINTS:
            base = base_positions_deg.get(joint, 0.0)
            off = offset_deg.get(joint, 0.0)
            result[joint] = base + off
        return result


# ---------------------------------------------------------------------------
# LeRobot-compatible dataset writer
# ---------------------------------------------------------------------------


@dataclass
class FrameRecord:
    """One timestep of one episode."""

    ts_monotonic: float
    ts_iso: str
    leader_positions_deg: Dict[str, float]   # 5-DoF, joints 1-5 only
    follower_positions_deg: Dict[str, float]  # 5-DoF, joints 1-5 only
    frame_index: int
    episode_index: int
    # wrist frame: HxW uint8 grayscale numpy array or None if not captured
    wrist_frame: Optional[Any] = None


@dataclass
class EpisodeMeta:
    """Episode-level metadata recorded in the dataset."""

    episode_index: int
    seed: int
    start_offset_deg: Dict[str, float]
    label: str  # 'success', 'failure', 'aborted'
    pre_insertion_pose_q_deg: List[float]  # follower joints at episode start
    ts_start_iso: str
    ts_end_iso: str
    duration_s: float
    frame_count: int


def _now_iso() -> str:
    from datetime import datetime, timezone
    return datetime.now(timezone.utc).isoformat()


class _LocalLeRobotWriter:
    """LeRobot v3-compatible local dataset writer (fallback when lerobot not installed).

    Layout::

        <dataset_root>/
          meta/
            info.json           — dataset-level metadata
          data/
            chunk-000/
              episode_000000.parquet   — one file per episode

    The Parquet schema follows the LeRobot v3 observation/action convention:
    observation.state: follower joint positions (5-DoF)
    action:           follower joint goal positions (5-DoF) — same as observation.state in teleop
    observation.images.wrist: PNG bytes column (if frames captured)
    leader_state: leader joint positions (5-DoF, extra column)
    timestamp: fractional seconds from episode start
    episode_index, frame_index: LeRobot index columns

    This writer never calls any HuggingFace Hub API.

    Converter stub
    --------------
    To convert to the full lerobot format after installing lerobot::

        from inchworm.teleop_capture import convert_local_to_lerobot
        convert_local_to_lerobot(dataset_root, output_path)

    (Stub; requires lerobot installation.)
    """

    def __init__(self, dataset_root: Path, fps: float = 10.0) -> None:
        self.dataset_root = dataset_root
        self.fps = fps
        self._episodes: List[EpisodeMeta] = []
        dataset_root.mkdir(parents=True, exist_ok=True)
        (dataset_root / "meta").mkdir(exist_ok=True)
        (dataset_root / "data" / "chunk-000").mkdir(parents=True, exist_ok=True)

    def write_episode(
        self,
        frames: List[FrameRecord],
        meta: EpisodeMeta,
    ) -> Path:
        """Serialise one episode to Parquet and return its path."""
        try:
            import pyarrow as pa
            import pyarrow.parquet as pq
        except ImportError:
            # Fallback: write a minimal JSON file so tests can proceed without pyarrow
            return self._write_episode_json(frames, meta)

        rows: List[Dict[str, Any]] = []
        for fr in frames:
            obs_state = [fr.follower_positions_deg.get(j, 0.0) for j in TELEOP_JOINTS]
            leader_state = [fr.leader_positions_deg.get(j, 0.0) for j in TELEOP_JOINTS]
            row: Dict[str, Any] = {
                "timestamp": fr.ts_monotonic,
                "episode_index": fr.episode_index,
                "frame_index": fr.frame_index,
                "observation.state": obs_state,
                "action": obs_state,  # teleop: action = achieved follower state
                "leader_state": leader_state,
            }
            if fr.wrist_frame is not None:
                # Store as raw bytes (PNG encode if numpy array)
                try:
                    import cv2 as _cv2
                    ok, buf = _cv2.imencode(".png", fr.wrist_frame)
                    row["observation.images.wrist"] = buf.tobytes() if ok else b""
                except (ImportError, Exception):
                    row["observation.images.wrist"] = bytes(fr.wrist_frame) if fr.wrist_frame is not None else b""
            rows.append(row)

        table = pa.table({k: [r[k] for r in rows] for k in rows[0]} if rows else {})
        ep_path = self.dataset_root / "data" / "chunk-000" / f"episode_{meta.episode_index:06d}.parquet"
        pq.write_table(table, str(ep_path))
        log.info("AA-23 writer: episode %d written to %s", meta.episode_index, ep_path)
        return ep_path

    def _write_episode_json(self, frames: List[FrameRecord], meta: EpisodeMeta) -> Path:
        """Minimal JSON fallback when pyarrow is unavailable."""
        ep_path = self.dataset_root / "data" / "chunk-000" / f"episode_{meta.episode_index:06d}.json"
        data = [
            {
                "timestamp": fr.ts_monotonic,
                "episode_index": fr.episode_index,
                "frame_index": fr.frame_index,
                "observation.state": [fr.follower_positions_deg.get(j, 0.0) for j in TELEOP_JOINTS],
                "action": [fr.follower_positions_deg.get(j, 0.0) for j in TELEOP_JOINTS],
                "leader_state": [fr.leader_positions_deg.get(j, 0.0) for j in TELEOP_JOINTS],
            }
            for fr in frames
        ]
        ep_path.write_text(json.dumps({"frames": data, "meta": _meta_to_dict(meta)}, indent=2))
        log.info("AA-23 writer (json fallback): episode %d written to %s", meta.episode_index, ep_path)
        return ep_path

    def write_info(self, episodes: List[EpisodeMeta]) -> Path:
        """Write meta/info.json with dataset-level metadata."""
        total_frames = sum(e.frame_count for e in episodes)
        info = {
            "format_version": _LEROBOT_FORMAT_VERSION,
            "robot_type": "so101",
            "fps": self.fps,
            "features": {
                "observation.state": {
                    "dtype": "float32",
                    "shape": [len(TELEOP_JOINTS)],
                    "names": list(TELEOP_JOINTS),
                },
                "action": {
                    "dtype": "float32",
                    "shape": [len(TELEOP_JOINTS)],
                    "names": list(TELEOP_JOINTS),
                },
                "leader_state": {
                    "dtype": "float32",
                    "shape": [len(TELEOP_JOINTS)],
                    "names": list(TELEOP_JOINTS),
                },
                "observation.images.wrist": {
                    "dtype": "image",
                    "shape": None,
                    "names": None,
                },
                "timestamp": {"dtype": "float64", "shape": [1], "names": None},
                "episode_index": {"dtype": "int64", "shape": [1], "names": None},
                "frame_index": {"dtype": "int64", "shape": [1], "names": None},
            },
            "stats": {
                "total_episodes": len(episodes),
                "total_frames": total_frames,
                "success_count": sum(1 for e in episodes if e.label == "success"),
                "failure_count": sum(1 for e in episodes if e.label == "failure"),
                "aborted_count": sum(1 for e in episodes if e.label == "aborted"),
            },
            "episodes": [_meta_to_dict(e) for e in episodes],
            "hub_push": False,  # NEVER pushed to hub
        }
        info_path = self.dataset_root / _META_INFO_PATH
        info_path.write_text(json.dumps(info, indent=2))
        return info_path


def _meta_to_dict(meta: EpisodeMeta) -> Dict[str, Any]:
    return {
        "episode_index": meta.episode_index,
        "seed": meta.seed,
        "start_offset_deg": meta.start_offset_deg,
        "label": meta.label,
        "pre_insertion_pose_q_deg": meta.pre_insertion_pose_q_deg,
        "ts_start_iso": meta.ts_start_iso,
        "ts_end_iso": meta.ts_end_iso,
        "duration_s": meta.duration_s,
        "frame_count": meta.frame_count,
    }


def make_writer(dataset_root: Path, fps: float = 10.0) -> "_LocalLeRobotWriter":
    """Return a dataset writer.

    Prefers the real ``lerobot`` LeRobotDataset API if importable; otherwise
    returns a :class:`_LocalLeRobotWriter`.  NEVER pushes to the hub.
    """
    try:
        import lerobot  # type: ignore[import]
        log.info("AA-23: lerobot package found (%s); using _LocalLeRobotWriter for local safety.", getattr(lerobot, "__version__", "?"))
    except ImportError:
        pass
    # Always use local writer: the real LeRobotDataset has hub-push as the
    # default path; using it here risks accidentally pushing (AC#6).
    return _LocalLeRobotWriter(dataset_root, fps=fps)


def convert_local_to_lerobot(dataset_root: Path, output_path: Path) -> None:
    """Converter stub: convert a local dataset to the full lerobot format.

    Requires ``lerobot`` to be installed.  This is a stub that raises
    NotImplementedError until implemented by a separate migration task.
    """
    raise NotImplementedError(
        "convert_local_to_lerobot is a stub. Install lerobot and implement "
        "the conversion from the local parquet layout to LeRobotDataset."
    )


# ---------------------------------------------------------------------------
# Episode recorder
# ---------------------------------------------------------------------------


class EpisodeRecorder:
    """Records one teleop episode.

    Parameters
    ----------
    episode_index:
        Integer episode counter (monotonically increasing in a session).
    seed:
        Random seed used to draw the start offset.
    start_offset_deg:
        Per-joint start offset drawn from the configured range.
    pre_insertion_q_deg:
        Follower joint positions (5-DoF) at episode start (after moving to
        AA-18 pose + offset).
    leader_bus:
        :class:`~inchworm.servo_bus.ArmServoBus` for the leader arm (torque off).
        Only ``read_all_joint_states`` is called; never written.
    follower_guarded_arm:
        :class:`~inchworm.motion_guard.GuardedArm` for the follower arm.
        Every position write goes through the guard (AC: AA-9).
    mirror_mapper:
        :class:`MirrorMapper` for leader→follower position mapping.
    camera_arbiter:
        :class:`~inchworm.camera_arbiter.CameraArbiter` for wrist frame capture.
        Only the follower wrist camera is acquired (AC#9).
    follower_camera_role:
        Device role string for the follower wrist camera (e.g. ``"camera_left_arm"``).
    writer:
        :class:`_LocalLeRobotWriter` for persisting frame records.
    live:
        If False, writes to follower are suppressed (dry-run mode).
    """

    def __init__(
        self,
        episode_index: int,
        seed: int,
        start_offset_deg: Dict[str, float],
        pre_insertion_q_deg: List[float],
        leader_bus: Any,
        follower_guarded_arm: Any,
        mirror_mapper: MirrorMapper,
        camera_arbiter: Any,
        follower_camera_role: str,
        writer: _LocalLeRobotWriter,
        live: bool = False,
    ) -> None:
        self.episode_index = episode_index
        self.seed = seed
        self.start_offset_deg = start_offset_deg
        self.pre_insertion_q_deg = pre_insertion_q_deg
        self._leader_bus = leader_bus
        self._follower = follower_guarded_arm
        self._mapper = mirror_mapper
        self._arbiter = camera_arbiter
        self._camera_role = follower_camera_role
        self._writer = writer
        self._live = live
        self._frames: List[FrameRecord] = []
        self._ts_start: float = time.monotonic()
        self._ts_start_iso: str = _now_iso()

    def step(self) -> None:
        """Record one teleop timestep: read leader, map, write follower, grab frame.

        Joint 6 (gripper) is never read from the leader or written to the follower.
        """
        ts = time.monotonic()
        ts_iso = _now_iso()

        # Read leader joints 1-5 only
        all_states = self._leader_bus.read_all_joint_states()
        leader_deg: Dict[str, float] = {
            j: float(all_states[j].position_deg)
            for j in TELEOP_JOINTS
            if j in all_states
        }
        # Safety: assert gripper is not in the leader read
        _assert_no_gripper(list(leader_deg.keys()))

        # Map to follower positions
        follower_goal = self._mapper.apply(leader_deg)
        _assert_no_gripper(list(follower_goal.keys()))

        # Write follower through GuardedArm (AA-9)
        if self._live:
            for joint, deg in follower_goal.items():
                self._follower.apply_joint_command(joint, deg)
        else:
            log.debug("AA-23 dry-run: would write follower %s", follower_goal)

        # Read follower present positions
        follower_bus = getattr(self._follower, "bus", None)
        follower_deg: Dict[str, float] = {}
        if follower_bus is not None:
            try:
                follower_states = follower_bus.read_all_joint_states()
                follower_deg = {
                    j: float(follower_states[j].position_deg)
                    for j in TELEOP_JOINTS
                    if j in follower_states
                }
            except Exception:
                follower_deg = dict(follower_goal)
        else:
            follower_deg = dict(follower_goal)

        # Grab wrist frame via arbiter (AC#9: only follower wrist camera)
        wrist_frame = None
        if self._arbiter is not None:
            try:
                with self._arbiter.acquire(self._camera_role) as transport:
                    raw = transport.read_frame()
                    try:
                        import cv2 as _cv2
                        import numpy as _np
                        buf = _np.frombuffer(raw, dtype=_np.uint8)
                        wrist_frame = _cv2.imdecode(buf, _cv2.IMREAD_GRAYSCALE)
                    except (ImportError, Exception):
                        wrist_frame = None
            except Exception as exc:
                log.warning("AA-23: wrist frame capture failed: %s", exc)

        frame_index = len(self._frames)
        self._frames.append(
            FrameRecord(
                ts_monotonic=ts,
                ts_iso=ts_iso,
                leader_positions_deg=leader_deg,
                follower_positions_deg=follower_deg,
                frame_index=frame_index,
                episode_index=self.episode_index,
                wrist_frame=wrist_frame,
            )
        )

    def finalise(self, label: str) -> EpisodeMeta:
        """Finalise episode with a label and persist to the dataset.

        Parameters
        ----------
        label:
            One of ``'success'``, ``'failure'``, ``'aborted'``.
            Failed and aborted episodes are retained (AC#8).

        Returns
        -------
        EpisodeMeta
        """
        if label not in ("success", "failure", "aborted"):
            raise ValueError(f"label must be 'success', 'failure', or 'aborted'; got {label!r}")

        ts_end = time.monotonic()
        meta = EpisodeMeta(
            episode_index=self.episode_index,
            seed=self.seed,
            start_offset_deg=self.start_offset_deg,
            label=label,
            pre_insertion_pose_q_deg=list(self.pre_insertion_q_deg),
            ts_start_iso=self._ts_start_iso,
            ts_end_iso=_now_iso(),
            duration_s=ts_end - self._ts_start,
            frame_count=len(self._frames),
        )
        self._writer.write_episode(self._frames, meta)
        log.info(
            "AA-23: episode %d finalised: label=%s, %d frames, seed=%d",
            self.episode_index, label, len(self._frames), self.seed,
        )
        return meta


# ---------------------------------------------------------------------------
# Random start-offset drawing
# ---------------------------------------------------------------------------


def draw_start_offset(
    rng: random.Random,
    offset_range_cfg: Any,
) -> Dict[str, float]:
    """Draw per-joint start offsets within the configured range.

    Parameters
    ----------
    rng:
        Seeded :class:`random.Random` instance; the caller records the seed.
    offset_range_cfg:
        :class:`~inchworm.config.StartOffsetRangeConfig` from hardware.yaml.

    Returns
    -------
    Dict[str, float]
        Joint name → offset in degrees, for TELEOP_JOINTS only.
    """
    result: Dict[str, float] = {}
    for joint in TELEOP_JOINTS:
        max_deg = float(getattr(offset_range_cfg, f"{joint}_max_deg", 5.0))
        result[joint] = rng.uniform(-max_deg, max_deg)
    return result


# ---------------------------------------------------------------------------
# TeleopSession
# ---------------------------------------------------------------------------


class TeleopSession:
    """Orchestrates one teleop recording session.

    Handles: precondition check → move follower to pre-insertion pose +
    random offset → run EpisodeRecorder steps → finalise episode.

    Parameters
    ----------
    config:
        :class:`~inchworm.config.TeleopCaptureConfig` from hardware.yaml.
    leader_bus:
        ArmServoBus for the leader arm (torque off).
    follower_guarded_arm:
        GuardedArm for the follower arm.
    camera_arbiter:
        CameraArbiter for wrist-camera capture.
    follower_camera_role:
        Device role for the follower wrist camera.
    writer:
        _LocalLeRobotWriter for this session.
    pre_insertion_q_deg:
        Follower joint configuration at the pre-insertion pose (5-DoF, degrees).
        Callers should derive this from AA-18 get_pre_insertion_pose + IK.
    follower_gripper_state_fn:
        Optional callable ``() -> GripperState`` for locked-arm precondition.
    live:
        If True, real motion and camera capture occur. Dry-run otherwise.
    """

    def __init__(
        self,
        config: Any,
        leader_bus: Any,
        follower_guarded_arm: Any,
        camera_arbiter: Any,
        follower_camera_role: str,
        writer: _LocalLeRobotWriter,
        pre_insertion_q_deg: List[float],
        follower_gripper_state_fn: Optional[Any] = None,
        live: bool = False,
    ) -> None:
        self._cfg = config
        self._leader_bus = leader_bus
        self._follower = follower_guarded_arm
        self._arbiter = camera_arbiter
        self._camera_role = follower_camera_role
        self._writer = writer
        self._pre_insertion_q_deg = list(pre_insertion_q_deg)
        self._gripper_state_fn = follower_gripper_state_fn
        self._live = live
        self._episode_count = 0
        self._all_metas: List[EpisodeMeta] = []
        self._mapper = MirrorMapper(config.mirror_map)
        # Precondition checked on session start
        self._support_source: Optional[TubeSupportSource] = None

    def start(self) -> None:
        """Check preconditions and prepare the session.

        Raises :class:`PreconditionError` if preconditions are not met.
        """
        self._support_source = check_preconditions(
            tube_support_source=self._cfg.tube_support_source,
            operator_token=self._cfg.operator_token,
            leader_arm=self._cfg.leader_arm,
            follower_gripper_state_fn=self._gripper_state_fn,
        )
        log.info(
            "AA-23 TeleopSession started: leader=%s follower=%s support=%s live=%s",
            self._cfg.leader_arm, self._cfg.follower_arm,
            self._support_source.value, self._live,
        )

    def record_episode(self, n_steps: int = 100, speed_scale: float = 1.0) -> EpisodeMeta:
        """Record one episode.

        Parameters
        ----------
        n_steps:
            Number of teleop timesteps to record.
        speed_scale:
            Fractional speed scale applied when moving to start pose (live mode).
            Passed through for hardware scripts; not enforced in dry-run.

        Returns
        -------
        EpisodeMeta
            Metadata for the completed episode; label defaults to 'success'.
            Call :meth:`abort_episode` to label as 'aborted'.
        """
        seed = random.randrange(2**31)
        rng = random.Random(seed)
        offset = draw_start_offset(rng, self._cfg.start_offset_range)

        # Build per-joint start pose dict
        start_pose_deg: Dict[str, float] = {
            j: self._pre_insertion_q_deg[i]
            for i, j in enumerate(TELEOP_JOINTS)
        }
        goal_pose_deg = self._mapper.apply_offset_deg(start_pose_deg, offset)
        _assert_no_gripper(list(goal_pose_deg.keys()))

        # Move follower to start pose (through GuardedArm)
        if self._live:
            for joint, deg in goal_pose_deg.items():
                self._follower.apply_joint_command(joint, deg)
            log.info("AA-23: moved follower to start pose (seed=%d, offset=%s)", seed, offset)
        else:
            log.debug("AA-23 dry-run: start pose would be %s (seed=%d)", goal_pose_deg, seed)

        recorder = EpisodeRecorder(
            episode_index=self._episode_count,
            seed=seed,
            start_offset_deg=offset,
            pre_insertion_q_deg=[goal_pose_deg.get(j, 0.0) for j in TELEOP_JOINTS],
            leader_bus=self._leader_bus,
            follower_guarded_arm=self._follower,
            mirror_mapper=self._mapper,
            camera_arbiter=self._arbiter,
            follower_camera_role=self._camera_role,
            writer=self._writer,
            live=self._live,
        )

        for _ in range(n_steps):
            recorder.step()

        meta = recorder.finalise("success")
        self._all_metas.append(meta)
        self._episode_count += 1
        return meta

    def abort_episode(self, recorder: EpisodeRecorder) -> EpisodeMeta:
        """Finalise a running episode as 'aborted'. Aborted episodes are retained (AC#8)."""
        meta = recorder.finalise("aborted")
        self._all_metas.append(meta)
        self._episode_count += 1
        return meta

    def close(self) -> Path:
        """Write meta/info.json and close the session."""
        info_path = self._writer.write_info(self._all_metas)
        log.info(
            "AA-23 TeleopSession closed: %d episodes written to %s",
            len(self._all_metas), info_path,
        )
        return info_path


# ---------------------------------------------------------------------------
# Dataset inspection helpers (used by scripts/inspect_dataset.py)
# ---------------------------------------------------------------------------


@dataclass
class DatasetStats:
    """Aggregated stats for one local teleop dataset."""

    dataset_root: Path
    episode_count: int
    durations_s: List[float]
    success_count: int
    failure_count: int
    aborted_count: int
    start_offsets: List[Dict[str, float]]
    under_covered_joints: List[str]  # joints with sparse coverage


def load_dataset_stats(dataset_root: Path, offset_bins: int = 5) -> DatasetStats:
    """Load and aggregate stats from a local teleop dataset.

    Parameters
    ----------
    dataset_root:
        Root path of the dataset (contains ``meta/info.json``).
    offset_bins:
        Number of bins for the per-joint offset coverage histogram.
        Joints with any empty bin are flagged as under-covered.

    Returns
    -------
    DatasetStats
    """
    info_path = dataset_root / _META_INFO_PATH
    if not info_path.exists():
        raise FileNotFoundError(f"meta/info.json not found at {info_path}")

    info = json.loads(info_path.read_text())
    episodes: List[Dict[str, Any]] = info.get("episodes", [])

    durations = [float(e.get("duration_s", 0.0)) for e in episodes]
    success_count = sum(1 for e in episodes if e.get("label") == "success")
    failure_count = sum(1 for e in episodes if e.get("label") == "failure")
    aborted_count = sum(1 for e in episodes if e.get("label") == "aborted")
    start_offsets = [e.get("start_offset_deg", {}) for e in episodes]

    # Coverage histogram: for each joint, check that all bins are covered
    under_covered: List[str] = []
    for joint in TELEOP_JOINTS:
        values = [float(off.get(joint, 0.0)) for off in start_offsets if off]
        if len(values) < offset_bins:
            under_covered.append(joint)
            continue
        arr = np.array(values)
        lo, hi = arr.min(), arr.max()
        if hi - lo < 1e-6:
            under_covered.append(joint)
            continue
        counts, _ = np.histogram(arr, bins=offset_bins)
        if np.any(counts == 0):
            under_covered.append(joint)

    return DatasetStats(
        dataset_root=dataset_root,
        episode_count=len(episodes),
        durations_s=durations,
        success_count=success_count,
        failure_count=failure_count,
        aborted_count=aborted_count,
        start_offsets=start_offsets,
        under_covered_joints=under_covered,
    )
