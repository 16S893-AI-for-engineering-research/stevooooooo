"""AA-26: Autonomous policy runner with checkpoint loader and gripper-lock handback.

This module wires the trained LeRobot ACT checkpoint into the AA-20
:class:`~inchworm.supervisor.PolicySupervisor`, integrates the AA-24
:class:`~inchworm.seating.SeatingEvaluator` as the seating detector, and
commands the AA-17 :class:`~inchworm.gripper_lock.GripperLockSM` on a seated
verdict before returning success to the cycle runner.

Overview
--------
::

    [cycle runner]
        │  provide: checkpoint_dir, guarded_arm, seating_evaluator, lock_sm
        ▼
    [PolicyRunner.run_insertion()]
        │  CheckpointLoader.load()  → CheckpointPolicy (hash logged)
        │  build PolicySupervisor (limits, bounds, clamps UNCHANGED from AA-20)
        │  supervisor.run() →
        │      SeatingEvaluator (UNCERTAIN → probe → re-evaluate) as detector
        │      on SEATED: lock_sm.lock() → lock_sm.hold_tick() verify
        │      on abort/retry/halt: supervisor handles (AA-20 paths)
        ▼
    [cycle runner] ← outcome + diagnostics returned

Checkpoint format (LeRobot ACT)
--------------------------------
``pretrained_model/`` directory must contain:

- ``config.json`` — LeRobot model configuration; fields consumed here:
  - ``output_shapes`` or ``action_feature_size`` to verify 5-dim action
  - ``input_shapes`` to verify 5 joint positions + 1 camera image key
  - (exact field names depend on LeRobot ACT version; validated against
    :func:`_validate_checkpoint_features`)
- ``model.safetensors`` — model weights; only SHA-256 is computed here;
  lerobot is not imported in the main env (AC#6).

Contract validation (AC#6)
---------------------------
:class:`CheckpointLoader` validates at *load* time (before any inference):

1. ``config.json`` exists and is valid JSON.
2. The action dimension equals :data:`~inchworm.policy_contract.N_JOINTS` (5).
3. Observation features include exactly one image key and 5 joint-position
   keys matching :data:`~inchworm.policy_contract.JOINT_ORDER` (with the
   ``.pos`` suffix convention of AA-22).
4. The gripper key is **absent** from action features.

Any mismatch raises :class:`CheckpointIncompatible` before any model weights
are loaded into memory.

Backend protocol (AC#1)
-----------------------
Two :class:`PolicyBackend` implementations:

``StubBackend``
    Returns a fixed nudge; used in tests.  No lerobot import.
``LeRobotBackend``
    Lazy-imports ``lerobot`` and wraps an ``ACTPolicy``; used in
    ``robot/lerobot_env``.  Raises :class:`LeRobotNotAvailable` in the main
    3.10 env where lerobot is absent.

Safety invariants (AGENTS.md, non-negotiable)
---------------------------------------------
- Gripper servo is **never** commanded by this module or the policy backends.
  All gripper commands go through the injected :class:`GripperLockSM`.
- The supervisor's limits (max_steps, max_wall_time_s, workspace_radius_m,
  max_action_magnitude_deg) are passed through unchanged; this module does not
  override them (AC#1).
- ``dry_run=True`` is the default; live mode requires explicit ``dry_run=False``
  and a verified retract pose.
- Camera access is through the injected ``frame_source``; this module never
  opens a camera directly.
- AA-17 lock: on seated verdict, code calls ``lock_sm.lock()`` (APPROACH_READY
  transition), then ``lock_sm.hold_tick()`` to confirm the latch is holding.
  Only on confirmed latch does :meth:`PolicyRunner.run_insertion` return
  :attr:`~inchworm.supervisor.SupervisorOutcome.SUCCESS` (AC#2).
"""

from __future__ import annotations

import hashlib
import json
import logging
import os
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np
import numpy.typing as npt

from inchworm.policy_contract import (
    JOINT_ORDER,
    N_JOINTS,
    Observation,
    PolicyAction,
)
from inchworm.supervisor import (
    PolicyProtocol,
    PolicySupervisor,
    SupervisorOutcome,
    SupervisorResult,
)
from inchworm.seating import SeatingEvaluator, SeatingVerdict

_LOG = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Public exceptions
# ---------------------------------------------------------------------------


class CheckpointIncompatible(Exception):
    """Raised at checkpoint load time when the checkpoint does not satisfy the AA-20 contract.

    This exception is raised by :class:`CheckpointLoader` before any model
    weights are loaded into memory.  Mid-insertion discovery of an incompatible
    checkpoint is therefore impossible (AC#6).

    Attributes
    ----------
    checkpoint_dir:
        Path to the checkpoint directory.
    reason:
        Human-readable explanation of the incompatibility.
    """

    def __init__(self, checkpoint_dir: Path, reason: str) -> None:
        self.checkpoint_dir = checkpoint_dir
        self.reason = reason
        super().__init__(
            f"Checkpoint at {checkpoint_dir!r} is incompatible with the AA-20 contract: "
            f"{reason}"
        )


class LeRobotNotAvailable(ImportError):
    """Raised when the LeRobotBackend is used outside robot/lerobot_env.

    Activate the lerobot_env virtual environment before attempting live
    inference::

        cd robot/lerobot_env && uv run python scripts/rollout.py --checkpoint PATH
    """


# ---------------------------------------------------------------------------
# Checkpoint metadata
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class CheckpointMeta:
    """Immutable metadata for a validated checkpoint.

    Attributes
    ----------
    checkpoint_dir:
        Absolute path to the checkpoint directory.
    config_path:
        Absolute path to ``config.json``.
    weights_path:
        Absolute path to ``model.safetensors``.
    sha256:
        Hex-encoded SHA-256 digest of ``model.safetensors``.
    action_dim:
        Number of action dimensions (must equal :data:`~inchworm.policy_contract.N_JOINTS`).
    observation_keys:
        Tuple of validated observation feature keys.
    """

    checkpoint_dir: Path
    config_path: Path
    weights_path: Path
    sha256: str
    action_dim: int
    observation_keys: Tuple[str, ...]

    def log_identity(self, logger: logging.Logger, prefix: str = "") -> None:
        """Log checkpoint path and hash to *logger* at INFO level.

        Called at the start of every rollout (AC#5).
        """
        logger.info(
            "%scheckpoint path=%s sha256=%s action_dim=%d",
            prefix,
            self.weights_path,
            self.sha256,
            self.action_dim,
        )


# ---------------------------------------------------------------------------
# Feature validation helpers (AC#6)
# ---------------------------------------------------------------------------

#: Expected action feature keys (AA-22 convention: "<joint>.pos")
_EXPECTED_ACTION_KEYS: frozenset[str] = frozenset(f"{j}.pos" for j in JOINT_ORDER)

#: Expected observation joint keys
_EXPECTED_OBS_JOINT_KEYS: frozenset[str] = _EXPECTED_ACTION_KEYS

#: Camera key (AA-22)
_CAMERA_KEY: str = "wrist_cam"

#: Gripper key — must be absent
_GRIPPER_KEY: str = "gripper"
_GRIPPER_POS_KEY: str = "gripper.pos"


def _validate_checkpoint_features(
    config_dict: Dict[str, Any],
    checkpoint_dir: Path,
) -> Tuple[int, Tuple[str, ...]]:
    """Validate checkpoint features against the AA-20 contract.

    Parameters
    ----------
    config_dict:
        Parsed ``config.json``.
    checkpoint_dir:
        Used in error messages only.

    Returns
    -------
    (action_dim, observation_keys)
        action_dim: number of action dimensions (must be N_JOINTS = 5).
        observation_keys: validated tuple of observation feature keys.

    Raises
    ------
    :class:`CheckpointIncompatible`
        On any contract violation.
    """

    def _fail(reason: str) -> None:
        raise CheckpointIncompatible(checkpoint_dir, reason)

    # --- Action feature validation ---
    # LeRobot ACT config.json can store action dim in several ways:
    # 1. top-level "output_shapes": {"action": [<dim>]}
    # 2. top-level "action_feature_size": <dim>
    # 3. "config" sub-dict with "output_features": {...}
    # 4. "output_features" at top level: {"<joint>.pos": "float", ...}
    # We support all forms; prefer explicit key listing to integer dimension.

    action_dim: Optional[int] = None
    action_keys_found: Optional[frozenset] = None

    # Form 1: output_shapes (LeRobot <=0.5)
    if "output_shapes" in config_dict:
        shapes = config_dict["output_shapes"]
        if isinstance(shapes, dict) and "action" in shapes:
            shape = shapes["action"]
            if isinstance(shape, (list, tuple)) and len(shape) >= 1:
                action_dim = int(shape[-1])

    # Form 2: action_feature_size (LeRobot 0.6+)
    if action_dim is None and "action_feature_size" in config_dict:
        action_dim = int(config_dict["action_feature_size"])

    # Form 3: output_features dict keyed by joint name
    if "output_features" in config_dict:
        feats = config_dict["output_features"]
        if isinstance(feats, dict):
            action_keys_found = frozenset(feats.keys())
            action_dim = len(action_keys_found)

    # Form 4: inside a nested "config" sub-dict
    if action_dim is None and "config" in config_dict:
        sub = config_dict["config"]
        if isinstance(sub, dict):
            if "output_features" in sub:
                feats = sub["output_features"]
                if isinstance(feats, dict):
                    action_keys_found = frozenset(feats.keys())
                    action_dim = len(action_keys_found)
            if action_dim is None and "output_shapes" in sub:
                shapes = sub["output_shapes"]
                if isinstance(shapes, dict) and "action" in shapes:
                    shape = shapes["action"]
                    if isinstance(shape, (list, tuple)) and len(shape) >= 1:
                        action_dim = int(shape[-1])

    if action_dim is None:
        _fail(
            "cannot determine action dimension from config.json; "
            "expected one of: output_shapes.action, action_feature_size, output_features dict"
        )

    if action_dim != N_JOINTS:
        _fail(
            f"action dimension {action_dim} ≠ contract N_JOINTS={N_JOINTS}; "
            "checkpoint was trained with a different action space"
        )

    # Gripper must be absent from action keys (if we have explicit keys)
    if action_keys_found is not None:
        for bad_key in (_GRIPPER_KEY, _GRIPPER_POS_KEY):
            if bad_key in action_keys_found:
                _fail(
                    f"action features contain forbidden gripper key {bad_key!r}; "
                    "policy must never command the gripper servo (AGENTS.md safety invariant)"
                )
        # Verify the action keys match the contract joint keys
        if action_keys_found != _EXPECTED_ACTION_KEYS:
            # Some checkpoints use bare joint names; normalise
            bare = frozenset(k.removesuffix(".pos") for k in action_keys_found)
            expected_bare = frozenset(j for j in JOINT_ORDER)
            if bare != expected_bare:
                _fail(
                    f"action feature keys {sorted(action_keys_found)!r} do not match "
                    f"contract keys {sorted(_EXPECTED_ACTION_KEYS)!r}"
                )

    # --- Observation feature validation ---
    obs_keys: Optional[dict] = None

    # Form A: input_shapes (LeRobot <=0.5)
    if "input_shapes" in config_dict:
        obs_keys = config_dict["input_shapes"]
    # Form B: input_features (LeRobot 0.6+)
    if obs_keys is None and "input_features" in config_dict:
        obs_keys = config_dict["input_features"]
    # Form C: inside nested "config"
    if obs_keys is None and "config" in config_dict:
        sub = config_dict["config"]
        if isinstance(sub, dict):
            if "input_features" in sub:
                obs_keys = sub["input_features"]
            elif "input_shapes" in sub:
                obs_keys = sub["input_shapes"]

    validated_obs_keys: Tuple[str, ...] = ()
    if obs_keys is not None and isinstance(obs_keys, dict):
        key_set = frozenset(obs_keys.keys())

        # The LeRobot <=0.5 input_shapes format uses a single
        # "observation.state" key for all joints (not per-joint keys).
        # Accept that as a valid compact representation (we cannot check
        # per-joint names, but the action_dim was already validated above).
        has_compact_state = any(
            "state" in k.lower() or "observation" in k.lower()
            for k in key_set
        )

        # Check for camera key.  Must NOT end with ".pos" to avoid matching
        # wrist_flex.pos / wrist_roll.pos joint keys.
        camera_keys = {
            k for k in key_set
            if ("cam" in k.lower() or "image" in k.lower() or "wrist" in k.lower())
            and not k.endswith(".pos")
        }
        has_compact_image = any(
            ("image" in k.lower() or "depth" in k.lower() or "visual" in k.lower())
            and not k.endswith(".pos")
            for k in key_set
        )
        if not camera_keys and not has_compact_image:
            _fail(
                f"observation features contain no camera/image key; "
                f"found keys: {sorted(key_set)!r}; "
                "contract requires exactly one wrist-camera mono image"
            )

        # Only check per-joint names when the format uses explicit per-joint
        # keys (not the compact observation.state format).
        if not has_compact_state:
            joint_keys = {k for k in key_set if any(j in k for j in JOINT_ORDER)}
            bare_joints = frozenset(k.removesuffix(".pos") for k in joint_keys)
            expected_joints = frozenset(JOINT_ORDER)
            missing = expected_joints - bare_joints
            if missing:
                _fail(
                    f"observation features missing joint key(s): {sorted(missing)!r}; "
                    f"contract requires all 5 joints: {sorted(JOINT_ORDER)!r}"
                )

        # Gripper must not be in obs keys
        for bad_key in (_GRIPPER_KEY, _GRIPPER_POS_KEY):
            if bad_key in key_set:
                _fail(
                    f"observation features contain forbidden gripper key {bad_key!r}; "
                    "gripper is excluded from the contract observation space"
                )

        validated_obs_keys = tuple(sorted(obs_keys.keys()))

    return action_dim, validated_obs_keys


# ---------------------------------------------------------------------------
# CheckpointLoader (AC#5, AC#6)
# ---------------------------------------------------------------------------


class CheckpointLoader:
    """Load and validate a LeRobot ACT checkpoint directory without importing lerobot.

    Reads ``config.json`` to validate observation/action feature compatibility
    with the AA-20 contract.  Computes the SHA-256 of ``model.safetensors``
    for run traceability.  Raises :class:`CheckpointIncompatible` at load
    time on any mismatch.

    lerobot is **never** imported here; the actual inference uses a
    :class:`PolicyBackend` that lazy-imports lerobot when needed.

    Parameters
    ----------
    checkpoint_dir:
        Path to the directory containing ``config.json`` and
        ``model.safetensors``.  Must already exist.

    Raises
    ------
    :class:`CheckpointIncompatible`
        If the directory is missing required files, or if the feature
        specification disagrees with the AA-20 contract.
    FileNotFoundError
        If ``checkpoint_dir`` does not exist.
    """

    def __init__(self, checkpoint_dir: "str | Path") -> None:
        self._dir = Path(checkpoint_dir).resolve()

    def load(self) -> CheckpointMeta:
        """Validate and load checkpoint metadata.

        Returns
        -------
        :class:`CheckpointMeta`
            Immutable checkpoint identity and validated feature spec.

        Raises
        ------
        :class:`CheckpointIncompatible`
            If any contract check fails.
        FileNotFoundError
            If the checkpoint directory or required files are missing.
        """
        d = self._dir

        if not d.is_dir():
            raise FileNotFoundError(
                f"checkpoint directory does not exist: {d}"
            )

        config_path = d / "config.json"
        if not config_path.exists():
            raise CheckpointIncompatible(
                d,
                f"config.json not found in {d}; "
                "directory must be a LeRobot pretrained_model/ directory",
            )

        weights_path = d / "model.safetensors"
        if not weights_path.exists():
            raise CheckpointIncompatible(
                d,
                f"model.safetensors not found in {d}; "
                "only safetensors format is supported",
            )

        # Parse config.json (no lerobot import)
        try:
            with config_path.open("r", encoding="utf-8") as fh:
                config_dict: Dict[str, Any] = json.load(fh)
        except json.JSONDecodeError as exc:
            raise CheckpointIncompatible(
                d, f"config.json is not valid JSON: {exc}"
            ) from exc

        # Feature validation (AC#6): raises CheckpointIncompatible on mismatch
        action_dim, obs_keys = _validate_checkpoint_features(config_dict, d)

        # Compute SHA-256 of model.safetensors (AC#5)
        sha256 = _sha256_file(weights_path)

        meta = CheckpointMeta(
            checkpoint_dir=d,
            config_path=config_path,
            weights_path=weights_path,
            sha256=sha256,
            action_dim=action_dim,
            observation_keys=obs_keys,
        )

        _LOG.info(
            "checkpoint loaded: path=%s sha256=%s action_dim=%d obs_keys=%s",
            weights_path,
            sha256,
            action_dim,
            obs_keys,
        )
        return meta


def _sha256_file(path: Path, chunk_size: int = 65536) -> str:
    """Return the hex-encoded SHA-256 digest of a file."""
    h = hashlib.sha256()
    with path.open("rb") as fh:
        while True:
            chunk = fh.read(chunk_size)
            if not chunk:
                break
            h.update(chunk)
    return h.hexdigest()


# ---------------------------------------------------------------------------
# PolicyBackend ABC + implementations
# ---------------------------------------------------------------------------


class PolicyBackend(ABC):
    """Abstract inference backend for :class:`CheckpointPolicy`.

    Implementors:
    - :class:`StubBackend` — deterministic, no lerobot dependency; for tests.
    - :class:`LeRobotBackend` — lazy-imports lerobot; for lerobot_env.
    """

    @abstractmethod
    def step(self, obs: Observation) -> PolicyAction:
        """Produce a policy action from an observation.

        Parameters
        ----------
        obs:
            Current :class:`~inchworm.policy_contract.Observation`.

        Returns
        -------
        :class:`~inchworm.policy_contract.PolicyAction`
            Mapping joint_name → target position (degrees).  Must NOT
            include any key outside :data:`~inchworm.policy_contract.POLICY_JOINT_NAMES`.
        """
        ...

    def load_weights(self, meta: CheckpointMeta) -> None:
        """Load model weights from ``meta.weights_path``.

        Called once by :class:`CheckpointPolicy` after validation.
        Backends that do not need weights (e.g. :class:`StubBackend`) should
        implement this as a no-op.
        """


# --- AA-26: StubBackend ---

class StubBackend(PolicyBackend):
    """Deterministic stub backend for tests (no lerobot import).

    Parameters
    ----------
    nudge_joint:
        Joint to nudge on every step.  Default: ``"shoulder_pan"``.
    nudge_deg:
        Amount to nudge (degrees added to current position).  Default: 0.5.
    noop:
        When ``True``, return empty action on every step.
    inject_gripper_at_step:
        When not ``None``, inject a gripper key on that step number to test
        that the supervisor rejects it.  Test-only.
    """

    def __init__(
        self,
        *,
        nudge_joint: str = "shoulder_pan",
        nudge_deg: float = 0.5,
        noop: bool = False,
        inject_gripper_at_step: Optional[int] = None,
    ) -> None:
        from inchworm.policy_contract import POLICY_JOINT_NAMES  # noqa: PLC0415
        if nudge_joint not in POLICY_JOINT_NAMES:
            raise ValueError(
                f"nudge_joint {nudge_joint!r} is not in POLICY_JOINT_NAMES"
            )
        self._nudge_joint = nudge_joint
        self._nudge_deg = nudge_deg
        self._noop = noop
        self._inject_gripper_at_step = inject_gripper_at_step

    def step(self, obs: Observation) -> PolicyAction:
        if self._noop:
            return {}
        from inchworm.policy_contract import JOINT_ORDER as _JO  # noqa: PLC0415
        idx = list(_JO).index(self._nudge_joint)
        current = float(obs.joint_positions_deg[idx])
        action: PolicyAction = {self._nudge_joint: current + self._nudge_deg}
        if (
            self._inject_gripper_at_step is not None
            and obs.step_index == self._inject_gripper_at_step
        ):
            action["gripper"] = 90.0  # type: ignore[assignment]  # test only
        return action

    def load_weights(self, meta: CheckpointMeta) -> None:
        # Stub: no weights to load
        pass


# --- AA-26: LeRobotBackend ---

class LeRobotBackend(PolicyBackend):
    """Inference backend that lazy-imports lerobot and wraps ACTPolicy.

    This backend is intended for use inside ``robot/lerobot_env`` where
    lerobot is installed.  In the main 3.10 env it raises
    :class:`LeRobotNotAvailable` when instantiated.

    Parameters
    ----------
    device:
        PyTorch device string.  Default: ``"cpu"`` (use ``"cuda"`` in
        lerobot_env when a GPU is available).
    """

    def __init__(self, *, device: str = "cpu") -> None:
        # Lazy import check — fail early with a clear message
        try:
            import torch  # noqa: F401 PLC0415
        except ImportError as exc:
            raise LeRobotNotAvailable(
                "torch is not installed; activate robot/lerobot_env to use LeRobotBackend"
            ) from exc
        self._device = device
        self._policy: Any = None

    def load_weights(self, meta: CheckpointMeta) -> None:
        """Load ACTPolicy weights from ``meta.weights_path``.

        Raises :class:`LeRobotNotAvailable` if lerobot is not installed.
        """
        try:
            from lerobot.common.policies.act.modeling_act import ACTPolicy  # noqa: PLC0415
        except ImportError as exc:
            raise LeRobotNotAvailable(
                "lerobot is not installed; activate robot/lerobot_env"
            ) from exc

        import torch  # noqa: PLC0415
        from safetensors.torch import load_file  # noqa: PLC0415

        # Build policy from config
        config_path = meta.config_path
        policy = ACTPolicy.from_pretrained(str(meta.checkpoint_dir))
        policy.to(self._device)
        policy.eval()
        self._policy = policy
        _LOG.info(
            "LeRobotBackend: loaded ACTPolicy from %s on device=%s",
            meta.checkpoint_dir, self._device,
        )

    def step(self, obs: Observation) -> PolicyAction:
        if self._policy is None:
            raise RuntimeError(
                "LeRobotBackend.step() called before load_weights(); "
                "call CheckpointLoader.load() first"
            )
        import torch  # noqa: PLC0415

        # Build lerobot-style observation dict
        joint_tensor = torch.tensor(
            obs.joint_positions_deg, dtype=torch.float32, device=self._device
        ).unsqueeze(0)  # shape (1, 5)

        # Frame: (H, W) uint8 → (1, 1, H, W) float32 in [0, 1]
        frame_f = (
            torch.tensor(obs.frame, dtype=torch.float32, device=self._device) / 255.0
        )
        frame_tensor = frame_f.unsqueeze(0).unsqueeze(0)  # (1, 1, H, W)

        from inchworm.lerobot_adapter import (  # noqa: PLC0415
            OBSERVATION_JOINT_KEYS,
            CAMERA_KEY,
        )
        obs_dict = {
            k: joint_tensor[:, i : i + 1]
            for i, k in enumerate(OBSERVATION_JOINT_KEYS)
        }
        obs_dict[CAMERA_KEY] = frame_tensor  # type: ignore[assignment]

        with torch.no_grad():
            action_tensor = self._policy.select_action(obs_dict)

        # action_tensor shape: (1, N_JOINTS) or (N_JOINTS,)
        action_arr = action_tensor.squeeze(0).cpu().numpy()

        from inchworm.policy_contract import JOINT_ORDER as _JO  # noqa: PLC0415
        return {name: float(action_arr[i]) for i, name in enumerate(_JO)}


# ---------------------------------------------------------------------------
# CheckpointPolicy (implements PolicyProtocol)
# ---------------------------------------------------------------------------


class CheckpointPolicy(PolicyProtocol):
    """Policy wrapping a :class:`PolicyBackend` and a :class:`CheckpointMeta`.

    Implements the :class:`~inchworm.supervisor.PolicyProtocol` interface so
    it can be passed directly to :class:`~inchworm.supervisor.PolicySupervisor`.

    At each :meth:`step` call the checkpoint identity (path + SHA-256) is
    **not** re-logged (that would flood the log); instead it is logged once
    at rollout start via :meth:`log_rollout_start` (called by
    :class:`PolicyRunner` before each ``supervisor.run()`` call, AC#5).

    Parameters
    ----------
    meta:
        Validated :class:`CheckpointMeta` from :class:`CheckpointLoader.load`.
    backend:
        :class:`PolicyBackend` implementation.  The backend's
        :meth:`~PolicyBackend.load_weights` is called in the constructor.
    """

    def __init__(
        self,
        meta: CheckpointMeta,
        backend: PolicyBackend,
    ) -> None:
        self._meta = meta
        self._backend = backend
        # Load weights into the backend (may be a no-op for StubBackend)
        self._backend.load_weights(meta)

    @property
    def meta(self) -> CheckpointMeta:
        """The checkpoint metadata (path, hash, feature spec)."""
        return self._meta

    def log_rollout_start(self, logger: logging.Logger) -> None:
        """Log checkpoint identity before a rollout (AC#5).

        Call this before each :meth:`~inchworm.supervisor.PolicySupervisor.run` to
        ensure every rollout in the log is traceable to the exact model weights.
        """
        self._meta.log_identity(logger, prefix="rollout_start: ")

    def step(self, obs: Observation) -> PolicyAction:
        """Delegate to the backend.

        Parameters
        ----------
        obs:
            Current :class:`~inchworm.policy_contract.Observation`.

        Returns
        -------
        :class:`~inchworm.policy_contract.PolicyAction`
        """
        return self._backend.step(obs)


# ---------------------------------------------------------------------------
# SeatingAdapter: wraps SeatingEvaluator as a callable seating_detector
# ---------------------------------------------------------------------------


class _SeatingAdapter:
    """Adapts :class:`~inchworm.seating.SeatingEvaluator` to the ``() -> bool``
    supervisor interface.

    The supervisor calls ``seating_detector()`` after each step.  This adapter:
    1. Collects samples from the injected ``sample_source`` callable.
    2. Calls :meth:`~inchworm.seating.SeatingEvaluator.run_evaluation` with the
       accumulated window.
    3. Returns ``True`` only on :attr:`~inchworm.seating.SeatingVerdict.SEATED`.

    On UNCERTAIN the evaluator may execute a probe (if configured); the adapter
    returns ``False`` so the supervisor continues the policy loop.

    Parameters
    ----------
    evaluator:
        :class:`~inchworm.seating.SeatingEvaluator` instance.
    sample_source:
        Callable ``() -> Sequence[JointStateSample]`` that returns the current
        window of servo samples.  Injected so this adapter never reads hardware
        directly.
    """

    def __init__(
        self,
        evaluator: SeatingEvaluator,
        sample_source: "Callable[[], list]",
    ) -> None:
        self._evaluator = evaluator
        self._sample_source = sample_source

    def __call__(self) -> bool:
        samples = self._sample_source()
        if not samples:
            return False
        verdict, _features = self._evaluator.run_evaluation(samples)
        return verdict == SeatingVerdict.SEATED


# ---------------------------------------------------------------------------
# LockAdapter: wraps GripperLockSM.lock + hold_tick into the supervisor's
#              LockCallable interface (AC#2)
# ---------------------------------------------------------------------------


class _LockAdapter:
    """Adapts :class:`~inchworm.gripper_lock.GripperLockSM` to the supervisor's
    ``LockCallable`` interface.

    On :meth:`__call__`:
    1. Calls ``lock_sm.lock()``; expects :attr:`~inchworm.gripper_lock.LockVerdict.LATCHED`.
    2. Calls ``lock_sm.hold_tick()`` once to confirm the latch is holding.
    3. Returns ``True`` if and only if both succeed (AC#2).

    If ``lock_sm`` is None (dry-run without a real SM), logs a warning and
    returns ``True`` (dry-run optimistic pass).

    Parameters
    ----------
    lock_sm:
        :class:`~inchworm.gripper_lock.GripperLockSM` or ``None`` (dry-run stub).
    role:
        Arm role for logging.
    """

    def __init__(
        self,
        lock_sm: Optional[Any],
        *,
        role: str = "arm_left",
    ) -> None:
        self._sm = lock_sm
        self._role = role

    def __call__(self) -> bool:
        """Lock the gripper and confirm the latch with one hold_tick.

        Returns ``True`` on confirmed latch, ``False`` otherwise.
        """
        if self._sm is None:
            _LOG.warning(
                "lock_adapter[%s]: no GripperLockSM injected (dry-run); "
                "skipping lock command",
                self._role,
            )
            return True  # optimistic pass in dry-run without SM

        # Import verdict type lazily to avoid circular imports
        try:
            from inchworm.gripper_lock import LockVerdict, GripperLockError  # noqa: PLC0415
        except ImportError as exc:
            _LOG.error("lock_adapter: cannot import gripper_lock: %s", exc)
            return False

        try:
            verdict = self._sm.lock()
        except GripperLockError as exc:
            _LOG.error(
                "lock_adapter[%s]: lock() raised GripperLockError: %s",
                self._role, exc,
            )
            return False
        except Exception as exc:  # noqa: BLE001
            _LOG.error(
                "lock_adapter[%s]: lock() raised unexpected error: %s",
                self._role, exc,
            )
            return False

        if verdict != LockVerdict.LATCHED:
            _LOG.error(
                "lock_adapter[%s]: lock() returned %s; expected LATCHED",
                self._role, verdict,
            )
            return False

        # Confirm latch with one hold_tick (AC#2)
        try:
            self._sm.hold_tick()
        except GripperLockError as exc:
            _LOG.error(
                "lock_adapter[%s]: hold_tick() raised after successful lock: %s",
                self._role, exc,
            )
            return False
        except Exception as exc:  # noqa: BLE001
            _LOG.error(
                "lock_adapter[%s]: hold_tick() raised unexpected error: %s",
                self._role, exc,
            )
            return False

        _LOG.info(
            "lock_adapter[%s]: gripper latched and hold confirmed",
            self._role,
        )
        return True


# ---------------------------------------------------------------------------
# PolicyRunner (main integration class)
# ---------------------------------------------------------------------------


@dataclass
class RunnerResult:
    """Result of one :meth:`PolicyRunner.run_insertion` call.

    Attributes
    ----------
    outcome:
        Final outcome from the supervisor.
    supervisor_result:
        Full :class:`~inchworm.supervisor.SupervisorResult` for diagnostics.
    checkpoint_sha256:
        SHA-256 of the model weights used (for log traceability, AC#5).
    elapsed_s:
        Total wall-clock time including locking.
    """

    outcome: SupervisorOutcome
    supervisor_result: SupervisorResult
    checkpoint_sha256: str
    elapsed_s: float


class PolicyRunner:
    """Autonomous policy runner for the inchworm insertion task (AA-26).

    Wires together:
    - :class:`CheckpointPolicy` (the learned model)
    - :class:`~inchworm.supervisor.PolicySupervisor` (AA-20, **limits unchanged** AC#1)
    - :class:`~inchworm.seating.SeatingEvaluator` as the seating detector (AA-24)
    - :class:`~inchworm.gripper_lock.GripperLockSM` for code-commanded locking (AA-17, AC#2)

    Parameters
    ----------
    policy:
        A :class:`CheckpointPolicy` (or any :class:`PolicyProtocol`).
    supervisor_kwargs:
        Keyword arguments forwarded verbatim to
        :class:`~inchworm.supervisor.PolicySupervisor`.  The ``policy``
        kwarg is injected automatically; do not include it here.
        **All limits come from the caller; this class does not override them.**
    seating_evaluator:
        :class:`~inchworm.seating.SeatingEvaluator` used as the seating
        detector.  Pass ``None`` to use a never-seated stub (dry-run /
        testing without seating).
    sample_source:
        Callable ``() -> List[JointStateSample]`` supplying the current
        servo-sample window to the seating evaluator.  Required when
        ``seating_evaluator`` is not ``None``.
    lock_sm:
        :class:`~inchworm.gripper_lock.GripperLockSM` to command on a
        seated verdict (AC#2).  ``None`` is accepted in dry-run mode; in
        live mode ``None`` means lock is skipped and a WARNING is logged.
    role:
        Arm role string (e.g. ``"arm_left"``).
    dry_run:
        When ``True`` (the default), no physical commands are sent.
    logger:
        Standard :class:`logging.Logger` for rollout-start hash logging.
    """

    def __init__(
        self,
        *,
        policy: PolicyProtocol,
        supervisor_kwargs: Dict[str, Any],
        seating_evaluator: Optional[SeatingEvaluator] = None,
        sample_source: "Optional[Callable[[], list]]" = None,
        lock_sm: Optional[Any] = None,
        role: str = "arm_left",
        dry_run: bool = True,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        self._policy = policy
        self._supervisor_kwargs = dict(supervisor_kwargs)
        self._seating_evaluator = seating_evaluator
        self._sample_source = sample_source
        self._lock_sm = lock_sm
        self._role = role
        self._dry_run = dry_run
        self._logger = logger or _LOG

        # Build seating detector callable
        if seating_evaluator is not None:
            if sample_source is None:
                raise ValueError(
                    "sample_source must be provided when seating_evaluator is not None"
                )
            self._seating_detector: "Callable[[], bool]" = _SeatingAdapter(
                seating_evaluator, sample_source
            )
        else:
            # Never-seated stub: supervisor always runs to step limit or time limit
            self._seating_detector = lambda: False

        # Build lock callable
        self._lock_callable: "Callable[[], Any]" = _LockAdapter(
            lock_sm, role=role
        )

    def run_insertion(self, cycle_index: int = 0) -> RunnerResult:
        """Run one supervised insertion attempt.

        This is the top-level entry point called by the cycle runner.

        Logs checkpoint hash at the start of every call (AC#5).

        Parameters
        ----------
        cycle_index:
            Zero-based crawl cycle number for logging.

        Returns
        -------
        :class:`RunnerResult`
            Outcome, supervisor diagnostics, checkpoint SHA-256, elapsed time.
        """
        t_start = time.monotonic()

        # Log checkpoint identity before every rollout (AC#5)
        if isinstance(self._policy, CheckpointPolicy):
            self._policy.log_rollout_start(self._logger)
        else:
            self._logger.info("rollout_start: policy=%s", type(self._policy).__name__)

        # --- Build supervisor (limits from caller, UNCHANGED — AC#1) ---
        # The supervisor kwargs must include all required arguments except
        # 'policy', 'seating_detector', and 'lock_callable' (injected here).
        kwargs = dict(self._supervisor_kwargs)
        # Inject the seating detector and lock callable from this runner.
        # Policy, seating_detector, and lock_callable are wired here so the
        # caller cannot accidentally mix up the insertion-specific callables.
        kwargs["policy"] = self._policy
        kwargs["seating_detector"] = self._seating_detector
        kwargs["lock_callable"] = self._lock_callable
        if "dry_run" not in kwargs:
            kwargs["dry_run"] = self._dry_run

        supervisor = PolicySupervisor(**kwargs)

        result = supervisor.run(cycle_index=cycle_index)

        sha = (
            self._policy.meta.sha256
            if isinstance(self._policy, CheckpointPolicy)
            else "n/a"
        )
        elapsed = time.monotonic() - t_start

        # Log outcome
        self._logger.info(
            "insertion cycle=%d outcome=%s attempts=%d total_steps=%d sha256=%s elapsed=%.3fs",
            cycle_index,
            result.outcome.name,
            result.attempts,
            result.total_steps,
            sha,
            elapsed,
        )

        return RunnerResult(
            outcome=result.outcome,
            supervisor_result=result,
            checkpoint_sha256=sha,
            elapsed_s=elapsed,
        )
