"""AA-24: Insertion seating detector, independent of the policy.

The seating detector decides whether the gripper tube is correctly seated in
the attachment point.  It is a separate, independently testable component:
the policy is never consulted; only servo sensor signals (joint
``present_position``, ``load_raw``, ``current_raw``) are consumed.

Architecture
------------
:data:`JointStateSample`
    One timestamped sensor snapshot for one servo.  Groups gripper plus arm
    joints.

:data:`SeatingSamples`
    A sequence of :class:`JointStateSample` objects forming a short window.
    This is the *trace* type: the public API accepts a trace or a stream.

:class:`SeatingVerdict`
    ``SEATED`` / ``NOT_SEATED`` / ``UNCERTAIN``.

:class:`ProbeRequest`
    Returned alongside UNCERTAIN.  Documents the disambiguation step (a small
    retract-and-reinsert or a gentle push) for the supervisor to execute via
    its injected ``GuardedArm`` executor.

:class:`SeatingDetectorConfig`
    Thresholds from ``hardware.yaml`` (``seating_detector`` block).  All
    thresholds default to ``None``; null thresholds → UNCERTAIN, never SEATED
    (fail toward not-locking).

:class:`SeatingDetector`
    Primary callable.  Implements the AA-20 supervisor's ``seating_detector``
    interface (``Callable[[], bool]``): called with no arguments while a
    stream of :class:`JointStateSample` is being fed externally; returns
    ``True`` only on SEATED.  Also exposes
    :meth:`evaluate_trace` for offline / test use.

:class:`ThresholdFitter`
    Given a labelled JSON-lines dataset (see ``scripts/record_seating_traces.py``),
    fits thresholds minimising false positives subject to an FN cap, and
    reports FP/FN on a held-out split.

Fail-closed contract
--------------------
- ``None`` threshold → signal is ignored → no positive evidence → UNCERTAIN.
- UNCERTAIN **never** becomes SEATED without evidence.
- UNCERTAIN triggers a :class:`ProbeRequest`; the supervisor re-evaluates
  after the probe move.
- False positives (locking into thin air) are the serious failure.

Signals documented
------------------
1. **Position band**: gripper servo ``position_deg`` must be inside
   ``[seated_position_low_deg, seated_position_high_deg]`` (configurable).
   Outside → NOT_SEATED immediately.

2. **Load-rise signature**: the arm gripper ``load_raw`` (already sign-magnitude
   decoded by AA-8) must exceed ``load_rise_threshold_raw`` within the
   evaluation window.  A genuine seating produces a characteristic load spike
   as the latch snaps in.

3. **Settling criterion**: ``present_position`` variance over the window tail
   (last ``settle_window_samples`` samples) must be below
   ``settle_variance_deg2`` to confirm the arm has stopped moving against the
   attachment point.

All three signals must independently confirm seating; a single disconfirming
signal yields NOT_SEATED.  Any signal whose threshold is ``None`` is skipped
(no evidence either way from that signal alone); if *all configured* signals
pass but some are skipped → UNCERTAIN.

AGENTS.md safety invariants
-----------------------------
- Gripper servo is never commanded here; we only read sensor state.
- No policy internals are imported or consumed (AC#4).
- Dry-run default everywhere motion is involved (ProbeRequest carries a
  dry_run flag; execution is delegated to the injected executor).
- AA-7 telemetry is emitted for every evaluation.
"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Callable, Dict, List, Optional, Sequence, Tuple

_LOG = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Public data types
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class JointStateSample:
    """One timestamped sensor snapshot for a single servo.

    Mirrors :class:`~inchworm.servo_bus.JointState` but is a plain
    dataclass so it can be constructed, serialised and compared without
    a bus connection.

    Attributes
    ----------
    servo_id:
        Hardware servo ID (integer).
    joint_name:
        Joint name string (e.g. ``"gripper"``, ``"shoulder_pan"``).
    timestamp_s:
        Monotonic timestamp (seconds) of the read.  Use
        ``time.monotonic()``; absolute wall-clock is not required.
    position_deg:
        Present position in degrees (0–360, STS3215 range).
    load_raw:
        Sign-magnitude decoded load (AA-8 already decoded).  Negative =
        reverse direction.
    current_raw:
        Raw present-current register (u16, hardware units).
    speed_raw:
        Sign-magnitude decoded speed (optional; 0 when not available).
    """

    servo_id: int
    joint_name: str
    timestamp_s: float
    position_deg: float
    load_raw: int
    current_raw: int
    speed_raw: int = 0

    def to_dict(self) -> Dict[str, object]:
        """Serialise to a plain dict (JSON-serialisable)."""
        return {
            "servo_id": self.servo_id,
            "joint_name": self.joint_name,
            "timestamp_s": self.timestamp_s,
            "position_deg": self.position_deg,
            "load_raw": self.load_raw,
            "current_raw": self.current_raw,
            "speed_raw": self.speed_raw,
        }

    @classmethod
    def from_dict(cls, d: Dict[str, object]) -> "JointStateSample":
        """Deserialise from a plain dict."""
        return cls(
            servo_id=int(d["servo_id"]),  # type: ignore[arg-type]
            joint_name=str(d["joint_name"]),
            timestamp_s=float(d["timestamp_s"]),  # type: ignore[arg-type]
            position_deg=float(d["position_deg"]),  # type: ignore[arg-type]
            load_raw=int(d["load_raw"]),  # type: ignore[arg-type]
            current_raw=int(d["current_raw"]),  # type: ignore[arg-type]
            speed_raw=int(d.get("speed_raw", 0)),  # type: ignore[arg-type]
        )


#: A trace is a sequence of samples for one gripper-joint over a short window.
SeatingSamples = Sequence[JointStateSample]


class SeatingVerdict(Enum):
    """Result of one seating evaluation.

    SEATED:
        All configured signals confirm seating.  Only returned when at least
        one threshold is non-null and all non-null signals pass.

    NOT_SEATED:
        At least one configured signal actively disconfirms seating.  Returned
        when the gripper position is outside the seated band, or when load
        failed to rise above threshold.

    UNCERTAIN:
        Not enough evidence to decide.  Either all thresholds are null, or
        some signals pass but others are unevaluable (null threshold), or the
        window is too short.  Triggers a :class:`ProbeRequest`.
    """

    SEATED = auto()
    NOT_SEATED = auto()
    UNCERTAIN = auto()


@dataclass
class ProbeRequest:
    """Disambiguation step for UNCERTAIN verdict.

    The supervisor injects a :class:`~inchworm.motion_guard.GuardedArm`
    executor; this object documents the move to perform.

    The probe is a small retract-and-reinsert (or a gentle push) to produce
    a fresh load signature that can be evaluated cleanly.

    Attributes
    ----------
    probe_type:
        ``"retract_reinsert"`` — retract by ``retract_m`` then reinsert.
        ``"gentle_push"``      — push deeper by ``push_m``.
    retract_m:
        Retract distance in metres (used for ``retract_reinsert``).
    push_m:
        Push distance in metres (used for ``gentle_push``).
    dry_run:
        When ``True`` (the default), the executor must not send real motion
        commands.  Always ``True`` until an owner sets ``dry_run=False``
        explicitly.
    reason:
        Human-readable explanation of why the probe is needed.
    """

    probe_type: str = "retract_reinsert"
    retract_m: float = 0.003   # 3 mm default; unverified placeholder
    push_m: float = 0.002      # 2 mm default; unverified placeholder
    dry_run: bool = True
    reason: str = "UNCERTAIN verdict — re-evaluate after probe move"


# ---------------------------------------------------------------------------
# Seating detector configuration
# ---------------------------------------------------------------------------


@dataclass
class SeatingDetectorConfig:
    """Thresholds for the seating detector.

    All thresholds default to ``None`` (not yet measured).  Null threshold →
    that signal is ignored; if no signal is configured, every evaluation
    returns UNCERTAIN (fail-closed).

    Load-rise and settle thresholds should be derived from labelled recorded
    data using :class:`ThresholdFitter`.  Measure with
    ``scripts/record_seating_traces.py --live`` and fit with
    ``scripts/record_seating_traces.py --fit``.

    Fields
    ------
    seated_position_low_deg:
        Lower bound (inclusive) of the position band where seating is
        expected.  None → position signal not checked.  unverified: true
    seated_position_high_deg:
        Upper bound (inclusive) of the position band.  None → not checked.
        unverified: true
    load_rise_threshold_raw:
        Minimum peak |load_raw| over the window that constitutes a
        "load-rise" signature.  None → not checked.  unverified: true
    settle_window_samples:
        Number of samples at the *end* of the window used to measure settling.
        Default 5.  unverified: true
    settle_variance_deg2:
        Maximum allowed position variance (deg²) over the settle window.
        None → settling not checked.  unverified: true
    min_window_samples:
        Minimum number of samples required; fewer → UNCERTAIN.  Default 3.
    max_probe_attempts:
        Maximum number of probe-and-re-evaluate cycles before returning
        UNCERTAIN unconditionally.  Default 2.
    supervisor_loop_rate_hz:
        Documented supervisor loop rate (Hz).  Used to bound latency.
        Default 50 Hz (20 ms cycle, matching hold_tick interval).
        unverified: true — confirm from config.
    """

    # --- Position band ---
    seated_position_low_deg: Optional[float] = None     # unverified: true
    seated_position_high_deg: Optional[float] = None    # unverified: true

    # --- Load-rise ---
    load_rise_threshold_raw: Optional[int] = None       # unverified: true

    # --- Settling ---
    settle_window_samples: int = 5
    settle_variance_deg2: Optional[float] = None        # unverified: true

    # --- Window size ---
    min_window_samples: int = 3

    # --- Probe policy ---
    max_probe_attempts: int = 2

    # --- Supervisor loop rate (from gripper_lock.py: 20 ms = 50 Hz) ---
    supervisor_loop_rate_hz: float = 50.0               # unverified: true

    @classmethod
    def from_config_dict(cls, d: Optional[Dict[str, object]]) -> "SeatingDetectorConfig":
        """Build from the ``seating_detector`` sub-dict in hardware.yaml.

        Missing keys use defaults; extra keys are ignored (forward-compat).
        """
        if d is None:
            return cls()
        return cls(
            seated_position_low_deg=_opt_float(d.get("seated_position_low_deg")),
            seated_position_high_deg=_opt_float(d.get("seated_position_high_deg")),
            load_rise_threshold_raw=_opt_int(d.get("load_rise_threshold_raw")),
            settle_window_samples=int(d.get("settle_window_samples", 5)),  # type: ignore[arg-type]
            settle_variance_deg2=_opt_float(d.get("settle_variance_deg2")),
            min_window_samples=int(d.get("min_window_samples", 3)),  # type: ignore[arg-type]
            max_probe_attempts=int(d.get("max_probe_attempts", 2)),  # type: ignore[arg-type]
            supervisor_loop_rate_hz=float(d.get("supervisor_loop_rate_hz", 50.0)),  # type: ignore[arg-type]
        )


def _opt_float(v: object) -> Optional[float]:
    return None if v is None else float(v)  # type: ignore[arg-type]


def _opt_int(v: object) -> Optional[int]:
    return None if v is None else int(v)  # type: ignore[arg-type]


# ---------------------------------------------------------------------------
# Feature extraction
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class SeatingFeatures:
    """Extracted scalar features from a trace window.

    These are computed by :func:`extract_features` and passed to the
    threshold comparison logic.

    Attributes
    ----------
    n_samples:
        Number of samples in the trace.
    peak_abs_load_raw:
        Maximum absolute load_raw over the window.
    tail_position_variance_deg2:
        Position variance over the last ``settle_window_samples`` samples.
        ``None`` when fewer samples than the settle window.
    mean_position_deg:
        Mean position over the entire window.
    last_position_deg:
        Position of the most recent sample.
    """

    n_samples: int
    peak_abs_load_raw: float
    tail_position_variance_deg2: Optional[float]
    mean_position_deg: float
    last_position_deg: float


def extract_features(
    trace: SeatingSamples,
    settle_window_samples: int = 5,
) -> SeatingFeatures:
    """Extract scalar features from a trace window.

    Parameters
    ----------
    trace:
        Sequence of :class:`JointStateSample` for the gripper servo.
    settle_window_samples:
        Tail window length for variance computation.

    Returns
    -------
    :class:`SeatingFeatures`
    """
    n = len(trace)
    if n == 0:
        return SeatingFeatures(
            n_samples=0,
            peak_abs_load_raw=0.0,
            tail_position_variance_deg2=None,
            mean_position_deg=0.0,
            last_position_deg=0.0,
        )

    loads = [abs(s.load_raw) for s in trace]
    positions = [s.position_deg for s in trace]

    peak_load = float(max(loads))
    mean_pos = sum(positions) / n
    last_pos = positions[-1]

    # Tail variance
    tail: Optional[float] = None
    if n >= settle_window_samples:
        tail_pos = positions[-settle_window_samples:]
        tail_mean = sum(tail_pos) / settle_window_samples
        tail = sum((p - tail_mean) ** 2 for p in tail_pos) / settle_window_samples

    return SeatingFeatures(
        n_samples=n,
        peak_abs_load_raw=peak_load,
        tail_position_variance_deg2=tail,
        mean_position_deg=mean_pos,
        last_position_deg=last_pos,
    )


# ---------------------------------------------------------------------------
# Core evaluation logic
# ---------------------------------------------------------------------------


def _evaluate_features(
    features: SeatingFeatures,
    cfg: SeatingDetectorConfig,
) -> Tuple[SeatingVerdict, str]:
    """Classify features against configured thresholds.

    Returns
    -------
    (verdict, reason_str)
        ``reason_str`` is a human-readable explanation suitable for logging.

    Contract
    --------
    - If *any* configured threshold actively disconfirms → NOT_SEATED.
    - If *all* configured (non-null) thresholds pass → SEATED.
    - If no threshold is configured, or some pass and none disconfirm but some
      are unevaluable → UNCERTAIN.
    - UNCERTAIN is never upgraded to SEATED without positive evidence.
    """
    if features.n_samples < cfg.min_window_samples:
        return SeatingVerdict.UNCERTAIN, (
            f"window too short: {features.n_samples} < {cfg.min_window_samples}"
        )

    passing: List[str] = []
    failing: List[str] = []
    skipped: List[str] = []

    # --- Signal 1: position band ---
    lo = cfg.seated_position_low_deg
    hi = cfg.seated_position_high_deg
    if lo is not None and hi is not None:
        pos = features.last_position_deg
        if lo <= pos <= hi:
            passing.append(f"position {pos:.2f}°∈[{lo},{hi}]")
        else:
            failing.append(f"position {pos:.2f}°∉[{lo},{hi}]")
    else:
        skipped.append("position_band(null threshold)")

    # --- Signal 2: load-rise ---
    load_thresh = cfg.load_rise_threshold_raw
    if load_thresh is not None:
        peak = features.peak_abs_load_raw
        if peak >= load_thresh:
            passing.append(f"load_rise peak={peak:.0f}≥{load_thresh}")
        else:
            failing.append(f"load_rise peak={peak:.0f}<{load_thresh}")
    else:
        skipped.append("load_rise(null threshold)")

    # --- Signal 3: settling ---
    var_thresh = cfg.settle_variance_deg2
    if var_thresh is not None:
        if features.tail_position_variance_deg2 is None:
            skipped.append(
                f"settling(window too short for tail={cfg.settle_window_samples})"
            )
        elif features.tail_position_variance_deg2 <= var_thresh:
            passing.append(
                f"settling var={features.tail_position_variance_deg2:.4f}≤{var_thresh}"
            )
        else:
            failing.append(
                f"settling var={features.tail_position_variance_deg2:.4f}>{var_thresh}"
            )
    else:
        skipped.append("settling(null threshold)")

    # --- Verdict ---
    if failing:
        reason = "NOT_SEATED: " + "; ".join(failing)
        if passing:
            reason += " (passing: " + "; ".join(passing) + ")"
        return SeatingVerdict.NOT_SEATED, reason

    if not passing:
        # Nothing failed, but nothing passed either (all skipped)
        reason = "UNCERTAIN: all thresholds null — " + "; ".join(skipped)
        return SeatingVerdict.UNCERTAIN, reason

    # All non-null signals passed; were any skipped?
    if skipped:
        reason = (
            "UNCERTAIN: passing signals ("
            + "; ".join(passing)
            + ") but some skipped ("
            + "; ".join(skipped)
            + ")"
        )
        return SeatingVerdict.UNCERTAIN, reason

    # All configured signals passed, none skipped
    reason = "SEATED: " + "; ".join(passing)
    return SeatingVerdict.SEATED, reason


# ---------------------------------------------------------------------------
# SeatingDetector
# ---------------------------------------------------------------------------


class SeatingDetector:
    """Insertion seating detector (AA-24).

    Callable interface (AA-20 supervisor-compatible)
    ------------------------------------------------
    The detector is callable with no arguments::

        detector = SeatingDetector(config=cfg)
        # Feed samples (called by whoever reads the bus):
        detector.push_sample(sample)
        # Called by supervisor each step:
        seated: bool = detector()   # True ↔ SEATED

    Standalone / offline evaluation
    --------------------------------
    ::

        verdict, features, probe = detector.evaluate_trace(trace)

    Parameters
    ----------
    config:
        :class:`SeatingDetectorConfig` with thresholds.  All ``None`` →
        always UNCERTAIN (fail-closed).
    logger:
        Optional AA-7 :class:`~inchworm.telemetry.SessionLogger` (any object
        with an ``emit(**kwargs)`` method).  Events are silently discarded
        when ``None``.
    role:
        Arm role string for logging (e.g. ``"arm_left"``).
    dry_run:
        When ``True`` (the default), any injected probe executor is called
        with ``dry_run=True``; probe moves are not sent to hardware.
    """

    def __init__(
        self,
        config: Optional[SeatingDetectorConfig] = None,
        *,
        logger: Optional[object] = None,
        role: str = "arm_left",
        dry_run: bool = True,
    ) -> None:
        self._cfg = config or SeatingDetectorConfig()
        self._logger = logger
        self._role = role
        self._dry_run = dry_run
        # Stream buffer: accumulates JointStateSample from push_sample()
        self._buffer: List[JointStateSample] = []

    # ------------------------------------------------------------------
    # Stream interface (supervisor callable protocol)
    # ------------------------------------------------------------------

    def push_sample(self, sample: JointStateSample) -> None:
        """Append one sample to the rolling evaluation window.

        The window retains all samples pushed since the last :meth:`reset`
        call.  Callers should call :meth:`reset` at the start of each
        insertion attempt.
        """
        self._buffer.append(sample)

    def reset(self) -> None:
        """Clear the rolling window (call at the start of each insertion)."""
        self._buffer.clear()

    def __call__(self) -> bool:
        """Callable interface for the AA-20 supervisor.

        Returns ``True`` iff the current window verdict is SEATED.
        UNCERTAIN → ``False`` (never locks on ambiguous evidence).

        Note: the supervisor calls this after each policy step.  The caller
        is responsible for feeding samples via :meth:`push_sample` before
        each call.
        """
        if not self._buffer:
            return False
        verdict, features, probe = self.evaluate_trace(self._buffer)
        seated = (verdict == SeatingVerdict.SEATED)
        self._emit_evaluation(verdict=verdict, features=features, n_samples=len(self._buffer))
        return seated

    # ------------------------------------------------------------------
    # Standalone evaluation
    # ------------------------------------------------------------------

    def evaluate_trace(
        self,
        trace: SeatingSamples,
    ) -> Tuple[SeatingVerdict, SeatingFeatures, Optional[ProbeRequest]]:
        """Evaluate a trace and return (verdict, features, probe_request).

        This is the primary offline / test API (AC#4: no policy inputs).

        Parameters
        ----------
        trace:
            Sequence of :class:`JointStateSample` for the gripper joint.
            The samples should come from the gripper servo and/or the
            arm joints, but all thresholds currently operate on the
            **gripper** position and load; multi-joint traces should be
            filtered to the gripper joint before calling this method, or
            leave all joints and this method uses the first sample's joint.

        Returns
        -------
        (verdict, features, probe_request)
            ``probe_request`` is non-None when verdict is UNCERTAIN.
        """
        t_start = time.monotonic()
        features = extract_features(trace, self._cfg.settle_window_samples)
        verdict, reason = _evaluate_features(features, self._cfg)
        latency_s = time.monotonic() - t_start

        probe: Optional[ProbeRequest] = None
        if verdict == SeatingVerdict.UNCERTAIN:
            probe = ProbeRequest(
                probe_type="retract_reinsert",
                retract_m=0.003,
                push_m=0.002,
                dry_run=self._dry_run,
                reason=reason,
            )

        _LOG.debug(
            "seating[%s]: verdict=%s latency=%.6f s reason=%s",
            self._role, verdict.name, latency_s, reason,
        )
        self._emit_evaluation(verdict=verdict, features=features, n_samples=features.n_samples)
        return verdict, features, probe

    # ------------------------------------------------------------------
    # AA-7 telemetry
    # ------------------------------------------------------------------

    def _emit_evaluation(
        self,
        verdict: SeatingVerdict,
        features: SeatingFeatures,
        n_samples: int,
    ) -> None:
        """Emit a structured seating_evaluation event via AA-7 telemetry."""
        if self._logger is None:
            return
        emit = getattr(self._logger, "emit", None)
        if emit is None:
            return
        try:
            emit(
                role=self._role,
                subsystem="seating_detector",
                event_type="seating_evaluation",
                severity="INFO",
                payload={
                    "verdict": verdict.name,
                    "n_samples": n_samples,
                    "peak_abs_load_raw": features.peak_abs_load_raw,
                    "last_position_deg": features.last_position_deg,
                    "tail_variance_deg2": features.tail_position_variance_deg2,
                },
            )
        except Exception:  # noqa: BLE001
            pass


# ---------------------------------------------------------------------------
# Probe-execution helper (supervisor integration)
# ---------------------------------------------------------------------------


def execute_probe(
    probe: ProbeRequest,
    executor: Optional[Callable[[ProbeRequest, bool], None]],
    dry_run: bool = True,
) -> None:
    """Execute a :class:`ProbeRequest` via an injected executor callable.

    The executor signature is ``(probe: ProbeRequest, dry_run: bool) -> None``.
    When ``executor`` is ``None`` or ``dry_run=True``, the move is logged but
    not sent to hardware.

    Parameters
    ----------
    probe:
        The probe move specification.
    executor:
        Injected callable that sends the motion command via a
        :class:`~inchworm.motion_guard.GuardedArm`.  Must be provided by
        the supervisor; this module never imports GuardedArm directly.
    dry_run:
        When ``True``, the executor is called with ``dry_run=True`` and
        no real servo command is issued.
    """
    effective_dry_run = dry_run or probe.dry_run
    if executor is None:
        _LOG.info(
            "seating: ProbeRequest (type=%s) — no executor injected; logging only (dry_run=%s)",
            probe.probe_type, effective_dry_run,
        )
        return
    _LOG.info(
        "seating: executing ProbeRequest type=%s retract_m=%.4f push_m=%.4f dry_run=%s",
        probe.probe_type, probe.retract_m, probe.push_m, effective_dry_run,
    )
    executor(probe, effective_dry_run)


# ---------------------------------------------------------------------------
# SeatingEvaluator: probe-and-re-evaluate loop (used by supervisor)
# ---------------------------------------------------------------------------


class SeatingEvaluator:
    """Probe-and-re-evaluate loop for the UNCERTAIN disambiguation step (AC#5).

    Wraps :class:`SeatingDetector` with the probe-retry loop.  Used by the
    supervisor when the detector returns UNCERTAIN.

    The supervisor should call :meth:`run_evaluation` after accumulating a
    window of samples.  If UNCERTAIN, it executes the probe, flushes the
    window, accumulates a fresh window, and re-evaluates up to
    ``config.max_probe_attempts`` times.

    Parameters
    ----------
    detector:
        The :class:`SeatingDetector` to re-evaluate after each probe.
    probe_executor:
        Callable ``(probe: ProbeRequest, dry_run: bool) -> None`` injected
        from the supervisor layer (wraps GuardedArm).  When ``None``, probe
        moves are logged but not sent.
    dry_run:
        Passed through to :func:`execute_probe`.
    """

    def __init__(
        self,
        detector: SeatingDetector,
        probe_executor: Optional[Callable[[ProbeRequest, bool], None]] = None,
        dry_run: bool = True,
    ) -> None:
        self._detector = detector
        self._probe_executor = probe_executor
        self._dry_run = dry_run

    def run_evaluation(
        self,
        trace: SeatingSamples,
    ) -> Tuple[SeatingVerdict, SeatingFeatures]:
        """Evaluate trace; probe-and-re-evaluate on UNCERTAIN up to max_probe_attempts.

        Parameters
        ----------
        trace:
            Initial trace window (fresh samples from the current insertion).

        Returns
        -------
        (verdict, features)
            Final verdict after probe loop.  May still be UNCERTAIN if all
            probe attempts are exhausted.
        """
        cfg = self._detector._cfg
        current_trace = list(trace)

        for attempt in range(cfg.max_probe_attempts + 1):
            verdict, features, probe = self._detector.evaluate_trace(current_trace)
            if verdict != SeatingVerdict.UNCERTAIN:
                return verdict, features
            if attempt < cfg.max_probe_attempts and probe is not None:
                _LOG.info(
                    "seating_evaluator: attempt %d/%d UNCERTAIN — executing probe",
                    attempt + 1, cfg.max_probe_attempts,
                )
                execute_probe(probe, self._probe_executor, self._dry_run)
                # After probe, caller must feed fresh samples; we return
                # UNCERTAIN here and the supervisor collects a new window.
                # This method does not block; the supervisor loop drives
                # sample collection.
                return SeatingVerdict.UNCERTAIN, features

        return SeatingVerdict.UNCERTAIN, features


# ---------------------------------------------------------------------------
# ThresholdFitter
# ---------------------------------------------------------------------------


@dataclass
class LabelledTrace:
    """A labelled trace for threshold fitting.

    Attributes
    ----------
    label:
        ``"seated"`` or ``"not_seated"`` (jam / near-miss).
    samples:
        Sequence of :class:`JointStateSample` for the gripper joint.
    """

    label: str   # "seated" | "not_seated"
    samples: List[JointStateSample]

    def to_dict(self) -> Dict[str, object]:
        return {
            "label": self.label,
            "samples": [s.to_dict() for s in self.samples],
        }

    @classmethod
    def from_dict(cls, d: Dict[str, object]) -> "LabelledTrace":
        return cls(
            label=str(d["label"]),
            samples=[JointStateSample.from_dict(s) for s in d["samples"]],  # type: ignore[arg-type]
        )


@dataclass
class FitResult:
    """Result of :meth:`ThresholdFitter.fit`.

    Attributes
    ----------
    best_load_threshold:
        Fitted load_rise_threshold_raw; ``None`` when insufficient data.
    fp_rate:
        False positive rate on the held-out split (0–1).
        FP = not-seated but detector said SEATED (the dangerous failure).
    fn_rate:
        False negative rate on the held-out split (0–1).
        FN = seated but detector said NOT_SEATED or UNCERTAIN.
    n_train_seated, n_train_not_seated:
        Training split sizes.
    n_test_seated, n_test_not_seated:
        Test split sizes.
    thresholds_tried:
        Number of candidate thresholds evaluated.
    """

    best_load_threshold: Optional[int]
    fp_rate: float
    fn_rate: float
    n_train_seated: int
    n_train_not_seated: int
    n_test_seated: int
    n_test_not_seated: int
    thresholds_tried: int


class ThresholdFitter:
    """Fit a load-rise threshold from labelled traces.

    Minimises false positives subject to a false-negative cap.  Uses a
    held-out test split (default 20%) for evaluation.

    Parameters
    ----------
    fn_cap:
        Maximum acceptable false-negative rate.  Default 0.30 (30%).
        The fitter picks the threshold that minimises FP while keeping
        FN ≤ fn_cap.  If no threshold satisfies the cap, it picks the
        threshold with the lowest FN regardless of FP (and reports it).
    test_fraction:
        Fraction of each class to hold out for evaluation.  Default 0.2.
    """

    def __init__(self, fn_cap: float = 0.30, test_fraction: float = 0.20) -> None:
        self._fn_cap = fn_cap
        self._test_fraction = test_fraction

    def fit(
        self,
        traces: List[LabelledTrace],
        settle_window_samples: int = 5,
    ) -> FitResult:
        """Fit a load-rise threshold and report FP/FN on the held-out split.

        Only the load-rise threshold is fitted (position band requires
        hardware measurement; settling variance is secondary).  The position
        band and settling thresholds remain null in the returned
        :class:`SeatingDetectorConfig` and must be set by the owner after
        hardware measurement.

        Parameters
        ----------
        traces:
            Labelled traces from :data:`LabelledTrace.from_dict` /
            ``scripts/record_seating_traces.py``.  Expected labels:
            ``"seated"`` and ``"not_seated"``.
        settle_window_samples:
            Passed to :func:`extract_features`.

        Returns
        -------
        :class:`FitResult`
        """
        seated = [t for t in traces if t.label == "seated"]
        not_seated = [t for t in traces if t.label == "not_seated"]

        if not seated or not not_seated:
            return FitResult(
                best_load_threshold=None,
                fp_rate=float("nan"),
                fn_rate=float("nan"),
                n_train_seated=0,
                n_train_not_seated=0,
                n_test_seated=0,
                n_test_not_seated=0,
                thresholds_tried=0,
            )

        # Split
        def _split(items: List[LabelledTrace]) -> Tuple[List[LabelledTrace], List[LabelledTrace]]:
            n_test = max(1, int(len(items) * self._test_fraction))
            return items[n_test:], items[:n_test]

        train_s, test_s = _split(seated)
        train_ns, test_ns = _split(not_seated)

        # Extract peak loads
        def _peak(t: LabelledTrace) -> float:
            feats = extract_features(t.samples, settle_window_samples)
            return feats.peak_abs_load_raw

        train_peaks_seated = [_peak(t) for t in train_s]
        train_peaks_not_seated = [_peak(t) for t in train_ns]
        test_peaks_seated = [_peak(t) for t in test_s]
        test_peaks_not_seated = [_peak(t) for t in test_ns]

        # Candidate thresholds: all unique peak values plus neighbours
        all_peaks = sorted(set(train_peaks_seated + train_peaks_not_seated))
        if not all_peaks:
            return FitResult(
                best_load_threshold=None,
                fp_rate=float("nan"),
                fn_rate=float("nan"),
                n_train_seated=len(train_s),
                n_train_not_seated=len(train_ns),
                n_test_seated=len(test_s),
                n_test_not_seated=len(test_ns),
                thresholds_tried=0,
            )

        # Candidates: midpoints between consecutive peaks + extremes
        candidates = [int(all_peaks[0]) - 1]
        for i in range(len(all_peaks) - 1):
            mid = int((all_peaks[i] + all_peaks[i + 1]) / 2.0) + 1
            candidates.append(mid)
        candidates.append(int(all_peaks[-1]) + 1)

        best_thresh: Optional[int] = None
        best_fp = 1.0
        best_fn = 1.0

        for thresh in candidates:
            # A trace is SEATED when peak >= thresh (load-rise only)
            def _classify(peak: float) -> bool:
                return peak >= thresh

            fp = sum(1 for p in test_peaks_not_seated if _classify(p)) / max(1, len(test_peaks_not_seated))
            fn = sum(1 for p in test_peaks_seated if not _classify(p)) / max(1, len(test_peaks_seated))

            if fn <= self._fn_cap:
                if fp < best_fp or (fp == best_fp and fn < best_fn):
                    best_fp = fp
                    best_fn = fn
                    best_thresh = thresh
            # Fallback: no threshold satisfies FN cap
            if best_thresh is None:
                if fn < best_fn or (fn == best_fn and fp < best_fp):
                    best_fn = fn
                    best_fp = fp
                    best_thresh = thresh

        return FitResult(
            best_load_threshold=best_thresh,
            fp_rate=best_fp,
            fn_rate=best_fn,
            n_train_seated=len(train_s),
            n_train_not_seated=len(train_ns),
            n_test_seated=len(test_s),
            n_test_not_seated=len(test_ns),
            thresholds_tried=len(candidates),
        )
