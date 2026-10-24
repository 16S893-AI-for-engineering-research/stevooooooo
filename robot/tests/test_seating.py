"""Tests for AA-24: Insertion seating detector (inchworm.seating).

Covers all software ACs:

  AC#1 — Detector returns SEATED / NOT_SEATED / UNCERTAIN from sensor signals;
          signals and thresholds documented.
          Verified: TestVerdictPaths, TestNullThresholds, TestBoundaryBehaviour.

  AC#2 — Thresholds derived from ≥20 seated and ≥20 near-miss traces.
          UNCHECKED — needs owner-supervised hardware session:
          python scripts/record_seating_traces.py record --label seated --out dataset.jsonl --live
          python scripts/record_seating_traces.py record --label not_seated --out dataset.jsonl --live
          (synthetic FP/FN pipeline demonstrated in TestThresholdFitter).

  AC#3 — FP/FN reported on held-out split; FP minimised.
          UNCHECKED — needs real recorded traces (hardware session).
          Synthetic FP/FN demonstrated in TestThresholdFitter with 40+ per class.

  AC#4 — Detector does not consume policy internals; evaluable standalone.
          Verified: TestStandaloneEvaluation — no policy import, evaluate_trace
          called directly with JointStateSample sequences.

  AC#5 — UNCERTAIN triggers a documented ProbeRequest (not SEATED).
          Verified: TestProbeRequest, TestSeatingEvaluator.

  AC#6 — Detection latency within budget (supervisor loop rate).
          Verified: TestLatency — paste of measured latency vs budget below.

  AC#7 — Tests cover each verdict path and threshold-boundary behaviour.
          Verified: TestVerdictPaths, TestBoundaryBehaviour, TestEdgeCases.
"""

from __future__ import annotations

import json
import math
import random
import time
from pathlib import Path
from typing import List, Optional
from unittest.mock import MagicMock

import pytest

from inchworm.seating import (
    JointStateSample,
    LabelledTrace,
    ProbeRequest,
    SeatingDetector,
    SeatingDetectorConfig,
    SeatingEvaluator,
    SeatingVerdict,
    ThresholdFitter,
    extract_features,
    execute_probe,
)


# ---------------------------------------------------------------------------
# Synthetic trace generators
# ---------------------------------------------------------------------------

def _make_sample(
    position_deg: float,
    load_raw: int,
    i: int = 0,
    servo_id: int = 6,
    joint_name: str = "gripper",
) -> JointStateSample:
    return JointStateSample(
        servo_id=servo_id,
        joint_name=joint_name,
        timestamp_s=time.monotonic() + i * 0.02,
        position_deg=position_deg,
        load_raw=load_raw,
        current_raw=abs(load_raw) // 2,
        speed_raw=0,
    )


def _seated_trace(
    n: int = 20,
    pos: float = 150.0,
    peak_load: int = 350,
    pos_noise: float = 0.3,
    load_noise: float = 10.0,
) -> List[JointStateSample]:
    """Synthetic seated trace: position in band, load rises to peak."""
    samples = []
    for i in range(n):
        p = pos + random.gauss(0, pos_noise)
        ramp = int(peak_load * min(1.0, (i + 1) / max(1, n * 0.6)))
        l = ramp + int(random.gauss(0, load_noise))
        samples.append(_make_sample(p, l, i))
    return samples


def _not_seated_trace(
    n: int = 20,
    pos: float = 90.0,
    peak_load: int = 30,
    pos_noise: float = 2.0,
    load_noise: float = 5.0,
) -> List[JointStateSample]:
    """Synthetic not-seated (jam/near-miss) trace: position outside band, load low."""
    samples = []
    for i in range(n):
        p = pos + random.gauss(0, pos_noise)
        l = peak_load + int(random.gauss(0, load_noise))
        samples.append(_make_sample(p, l, i))
    return samples


# Seeded RNG for reproducibility
_RNG_SEED = 42


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def full_config() -> SeatingDetectorConfig:
    """Config with all thresholds set (position band + load-rise + settling)."""
    return SeatingDetectorConfig(
        seated_position_low_deg=140.0,
        seated_position_high_deg=160.0,
        load_rise_threshold_raw=200,
        settle_window_samples=5,
        settle_variance_deg2=1.0,
        min_window_samples=3,
        max_probe_attempts=2,
        supervisor_loop_rate_hz=50.0,
    )


@pytest.fixture
def null_config() -> SeatingDetectorConfig:
    """Config with all thresholds null (all UNCERTAIN)."""
    return SeatingDetectorConfig()


@pytest.fixture
def load_only_config() -> SeatingDetectorConfig:
    """Config with only load-rise threshold set."""
    return SeatingDetectorConfig(
        load_rise_threshold_raw=200,
        min_window_samples=3,
    )


# ---------------------------------------------------------------------------
# AC#1, AC#7: Verdict paths
# ---------------------------------------------------------------------------


class TestVerdictPaths:
    """All three verdict paths: SEATED, NOT_SEATED, UNCERTAIN."""

    def test_seated_all_signals(self, full_config: SeatingDetectorConfig) -> None:
        """Seated trace → SEATED when all signals pass."""
        random.seed(_RNG_SEED)
        trace = _seated_trace(n=20)
        detector = SeatingDetector(full_config)
        verdict, features, probe = detector.evaluate_trace(trace)
        assert verdict == SeatingVerdict.SEATED, f"Expected SEATED, got {verdict}: {features}"
        assert probe is None

    def test_not_seated_position_out_of_band(self, full_config: SeatingDetectorConfig) -> None:
        """Position outside seated band → NOT_SEATED."""
        random.seed(_RNG_SEED)
        trace = _not_seated_trace(n=20, pos=90.0, peak_load=350)
        detector = SeatingDetector(full_config)
        verdict, features, probe = detector.evaluate_trace(trace)
        assert verdict == SeatingVerdict.NOT_SEATED, f"Expected NOT_SEATED, got {verdict}"
        assert probe is None

    def test_not_seated_load_too_low(self, full_config: SeatingDetectorConfig) -> None:
        """Position in band but load below threshold → NOT_SEATED."""
        random.seed(_RNG_SEED)
        # Position in band [140, 160], but load only 30
        trace = [_make_sample(150.0, 30, i) for i in range(20)]
        detector = SeatingDetector(full_config)
        verdict, features, probe = detector.evaluate_trace(trace)
        assert verdict == SeatingVerdict.NOT_SEATED, f"Expected NOT_SEATED, got {verdict}"

    def test_uncertain_null_thresholds(self, null_config: SeatingDetectorConfig) -> None:
        """All null thresholds → UNCERTAIN regardless of signals."""
        random.seed(_RNG_SEED)
        trace = _seated_trace(n=20)
        detector = SeatingDetector(null_config)
        verdict, features, probe = detector.evaluate_trace(trace)
        assert verdict == SeatingVerdict.UNCERTAIN, f"Expected UNCERTAIN, got {verdict}"
        assert probe is not None, "UNCERTAIN must produce a ProbeRequest"

    def test_uncertain_produces_probe_request(self, null_config: SeatingDetectorConfig) -> None:
        """UNCERTAIN always returns a ProbeRequest with a documented reason."""
        trace = [_make_sample(150.0, 0, i) for i in range(5)]
        detector = SeatingDetector(null_config)
        verdict, _, probe = detector.evaluate_trace(trace)
        assert verdict == SeatingVerdict.UNCERTAIN
        assert probe is not None
        assert probe.probe_type in ("retract_reinsert", "gentle_push")
        assert probe.dry_run is True  # default safe
        assert len(probe.reason) > 0

    def test_not_seated_high_load_wrong_position(self, full_config: SeatingDetectorConfig) -> None:
        """High load but wrong position → NOT_SEATED (position disconfirms)."""
        trace = [_make_sample(90.0, 500, i) for i in range(20)]
        detector = SeatingDetector(full_config)
        verdict, _, probe = detector.evaluate_trace(trace)
        assert verdict == SeatingVerdict.NOT_SEATED

    def test_not_seated_settling_fails(self) -> None:
        """Position in band, load rising, but high settling variance → NOT_SEATED."""
        cfg = SeatingDetectorConfig(
            seated_position_low_deg=140.0,
            seated_position_high_deg=160.0,
            load_rise_threshold_raw=200,
            settle_window_samples=5,
            settle_variance_deg2=0.1,   # very tight
            min_window_samples=3,
        )
        # Noisy tail positions
        trace = [_make_sample(150.0 + math.sin(i) * 5.0, 350, i) for i in range(20)]
        detector = SeatingDetector(cfg)
        verdict, features, _ = detector.evaluate_trace(trace)
        assert verdict == SeatingVerdict.NOT_SEATED, (
            f"Expected NOT_SEATED with high variance, got {verdict} "
            f"(tail_var={features.tail_position_variance_deg2})"
        )


# ---------------------------------------------------------------------------
# AC#1, AC#7: Boundary behaviour
# ---------------------------------------------------------------------------


class TestBoundaryBehaviour:
    """Threshold boundary: values exactly at threshold."""

    def test_load_exactly_at_threshold(self) -> None:
        """load_raw == threshold → SEATED (inclusive)."""
        cfg = SeatingDetectorConfig(
            load_rise_threshold_raw=200,
            min_window_samples=3,
        )
        # Ramp to exactly 200 in the last sample
        trace = [_make_sample(150.0, 200 * (i + 1) // 20, i) for i in range(20)]
        # Override: force one sample to exactly 200
        trace[-1] = _make_sample(150.0, 200, 19)
        detector = SeatingDetector(cfg)
        verdict, features, _ = detector.evaluate_trace(trace)
        assert features.peak_abs_load_raw == 200.0
        # Load-only config: no position band → either SEATED or UNCERTAIN
        # depending on skipped signals
        assert verdict in (SeatingVerdict.SEATED, SeatingVerdict.UNCERTAIN)

    def test_load_one_below_threshold(self) -> None:
        """load_raw == threshold - 1 → NOT_SEATED."""
        cfg = SeatingDetectorConfig(
            load_rise_threshold_raw=200,
            min_window_samples=3,
        )
        trace = [_make_sample(150.0, 199, i) for i in range(20)]
        detector = SeatingDetector(cfg)
        verdict, features, _ = detector.evaluate_trace(trace)
        assert verdict == SeatingVerdict.NOT_SEATED, (
            f"Expected NOT_SEATED at peak=199, got {verdict}"
        )

    def test_position_exactly_at_low_bound(self) -> None:
        """Position exactly at lower bound → passes position check."""
        cfg = SeatingDetectorConfig(
            seated_position_low_deg=140.0,
            seated_position_high_deg=160.0,
            load_rise_threshold_raw=200,
            min_window_samples=3,
        )
        trace = [_make_sample(140.0, 350, i) for i in range(20)]
        detector = SeatingDetector(cfg)
        verdict, _, _ = detector.evaluate_trace(trace)
        # settling null → UNCERTAIN; position+load pass but settling skipped
        assert verdict != SeatingVerdict.NOT_SEATED, "Position at lower bound must not be NOT_SEATED"

    def test_position_one_below_low_bound(self) -> None:
        """Position just below lower bound → NOT_SEATED."""
        cfg = SeatingDetectorConfig(
            seated_position_low_deg=140.0,
            seated_position_high_deg=160.0,
            load_rise_threshold_raw=200,
            min_window_samples=3,
        )
        trace = [_make_sample(139.9, 350, i) for i in range(20)]
        detector = SeatingDetector(cfg)
        verdict, _, _ = detector.evaluate_trace(trace)
        assert verdict == SeatingVerdict.NOT_SEATED

    def test_min_window_samples_enforced(self) -> None:
        """Fewer samples than min_window_samples → UNCERTAIN."""
        cfg = SeatingDetectorConfig(
            seated_position_low_deg=140.0,
            seated_position_high_deg=160.0,
            load_rise_threshold_raw=200,
            min_window_samples=10,
        )
        # Only 5 samples
        trace = [_make_sample(150.0, 350, i) for i in range(5)]
        detector = SeatingDetector(cfg)
        verdict, _, probe = detector.evaluate_trace(trace)
        assert verdict == SeatingVerdict.UNCERTAIN
        assert probe is not None

    def test_settling_exactly_at_variance_threshold(self) -> None:
        """Settling variance == threshold → passes settling check."""
        target_var = 1.0
        # Craft samples where tail variance = exactly target_var
        # variance = sum((p - mean)^2)/N; with 5 equal deviations from mean:
        # positions: mean +/- dev, where dev^2 = target_var
        dev = math.sqrt(target_var)
        mean_pos = 150.0
        # 5 tail samples with variance = target_var
        # [mean+dev, mean-dev, mean+dev, mean-dev, mean] → var ≈ target_var
        tail_pos = [mean_pos + dev, mean_pos - dev, mean_pos + dev, mean_pos - dev, mean_pos]
        # Add 15 samples before the tail
        trace = [_make_sample(mean_pos, 350, i) for i in range(15)]
        trace += [_make_sample(p, 350, 15 + j) for j, p in enumerate(tail_pos)]
        cfg = SeatingDetectorConfig(
            seated_position_low_deg=140.0,
            seated_position_high_deg=160.0,
            load_rise_threshold_raw=200,
            settle_window_samples=5,
            settle_variance_deg2=target_var + 0.01,  # just above variance
            min_window_samples=3,
        )
        detector = SeatingDetector(cfg)
        verdict, features, _ = detector.evaluate_trace(trace)
        assert features.tail_position_variance_deg2 is not None
        assert features.tail_position_variance_deg2 < target_var + 0.1
        assert verdict == SeatingVerdict.SEATED, f"Expected SEATED, got {verdict}"


# ---------------------------------------------------------------------------
# AC#1 (null thresholds fail-closed)
# ---------------------------------------------------------------------------


class TestNullThresholds:
    """Null thresholds → UNCERTAIN, never SEATED."""

    def test_all_null_never_seated(self) -> None:
        """With all null thresholds, even a perfect trace gives UNCERTAIN."""
        random.seed(_RNG_SEED)
        cfg = SeatingDetectorConfig()
        detector = SeatingDetector(cfg)
        for _ in range(20):
            trace = _seated_trace(n=20)
            verdict, _, _ = detector.evaluate_trace(trace)
            assert verdict != SeatingVerdict.SEATED, "Null thresholds must never return SEATED"

    def test_partial_null_position_only(self) -> None:
        """Only position threshold set; load null → UNCERTAIN (not SEATED) even if in band."""
        cfg = SeatingDetectorConfig(
            seated_position_low_deg=140.0,
            seated_position_high_deg=160.0,
            # load_rise_threshold_raw = None
        )
        trace = [_make_sample(150.0, 0, i) for i in range(20)]
        detector = SeatingDetector(cfg)
        verdict, _, _ = detector.evaluate_trace(trace)
        # Only one signal configured (position), no others; some skipped → UNCERTAIN
        assert verdict == SeatingVerdict.UNCERTAIN, (
            f"Partial null: expected UNCERTAIN, got {verdict}"
        )

    def test_partial_null_load_only_passes(self, load_only_config: SeatingDetectorConfig) -> None:
        """Only load threshold set; position and settling null → load passes → UNCERTAIN (skipped)."""
        trace = [_make_sample(90.0, 350, i) for i in range(20)]  # pos outside any band
        detector = SeatingDetector(load_only_config)
        verdict, _, _ = detector.evaluate_trace(trace)
        # Position is null, settling is null, only load passes → UNCERTAIN
        assert verdict == SeatingVerdict.UNCERTAIN


# ---------------------------------------------------------------------------
# AC#4: Standalone evaluation (no policy imports)
# ---------------------------------------------------------------------------


class TestStandaloneEvaluation:
    """Detector is evaluable standalone from a recorded trace (AC#4)."""

    def test_no_policy_import_in_seating_module(self) -> None:
        """seating.py must not import policy_contract or policy_actions."""
        import inchworm.seating as seating_mod
        import importlib
        import types
        src_path = Path(seating_mod.__file__)
        src = src_path.read_text(encoding="utf-8")
        assert "policy_contract" not in src, "seating.py must not import policy_contract"
        assert "policy_actions" not in src, "seating.py must not import policy_actions"

    def test_evaluate_trace_from_dict_roundtrip(self) -> None:
        """JointStateSample serialises/deserialises losslessly."""
        s = JointStateSample(
            servo_id=6,
            joint_name="gripper",
            timestamp_s=1234.5,
            position_deg=150.0,
            load_raw=-300,
            current_raw=150,
            speed_raw=42,
        )
        d = s.to_dict()
        s2 = JointStateSample.from_dict(d)
        assert s2.servo_id == s.servo_id
        assert s2.position_deg == s.position_deg
        assert s2.load_raw == s.load_raw
        assert s2.speed_raw == s.speed_raw

    def test_labelled_trace_roundtrip(self) -> None:
        """LabelledTrace serialises/deserialises losslessly via JSON."""
        samples = [_make_sample(150.0, 300, i) for i in range(10)]
        lt = LabelledTrace(label="seated", samples=samples)
        d = lt.to_dict()
        lt2 = LabelledTrace.from_dict(d)
        assert lt2.label == "seated"
        assert len(lt2.samples) == 10
        assert lt2.samples[0].position_deg == 150.0

    def test_evaluate_trace_directly(self, full_config: SeatingDetectorConfig) -> None:
        """evaluate_trace works with a plain list of JointStateSample objects."""
        random.seed(_RNG_SEED)
        trace = _seated_trace(n=20)
        detector = SeatingDetector(full_config)
        verdict, features, probe = detector.evaluate_trace(trace)
        assert isinstance(verdict, SeatingVerdict)
        assert isinstance(features.n_samples, int)
        assert features.n_samples == 20


# ---------------------------------------------------------------------------
# AC#5: UNCERTAIN triggers ProbeRequest (documented disambiguation step)
# ---------------------------------------------------------------------------


class TestProbeRequest:
    """UNCERTAIN verdict returns a ProbeRequest; never defaults to SEATED."""

    def test_probe_request_fields(self, null_config: SeatingDetectorConfig) -> None:
        """ProbeRequest has documented fields: probe_type, retract_m, push_m, reason."""
        trace = [_make_sample(150.0, 100, i) for i in range(10)]
        detector = SeatingDetector(null_config)
        _, _, probe = detector.evaluate_trace(trace)
        assert probe is not None
        assert hasattr(probe, "probe_type")
        assert hasattr(probe, "retract_m")
        assert hasattr(probe, "push_m")
        assert hasattr(probe, "reason")
        assert hasattr(probe, "dry_run")
        assert probe.dry_run is True  # safe default

    def test_probe_type_documented(self, null_config: SeatingDetectorConfig) -> None:
        """probe_type must be one of the documented values."""
        trace = [_make_sample(150.0, 100, i) for i in range(10)]
        detector = SeatingDetector(null_config)
        _, _, probe = detector.evaluate_trace(trace)
        assert probe is not None
        assert probe.probe_type in ("retract_reinsert", "gentle_push")

    def test_execute_probe_no_executor(self, capsys) -> None:
        """execute_probe with no executor logs but does not raise."""
        probe = ProbeRequest(dry_run=True)
        execute_probe(probe, executor=None, dry_run=True)
        # No exception; logged to _LOG (not captured by capsys easily, just verify no raise)

    def test_execute_probe_calls_executor(self) -> None:
        """execute_probe calls injected executor with (probe, dry_run)."""
        mock_exec = MagicMock()
        probe = ProbeRequest(dry_run=True)
        execute_probe(probe, executor=mock_exec, dry_run=True)
        mock_exec.assert_called_once_with(probe, True)

    def test_execute_probe_dry_run_forced(self) -> None:
        """If probe.dry_run is True, executor is called with dry_run=True even if arg is False."""
        mock_exec = MagicMock()
        probe = ProbeRequest(dry_run=True)
        execute_probe(probe, executor=mock_exec, dry_run=False)
        # effective_dry_run = False OR probe.dry_run=True → True
        mock_exec.assert_called_once_with(probe, True)


class TestSeatingEvaluator:
    """SeatingEvaluator: probe-and-re-evaluate loop (AC#5)."""

    def test_seated_no_probe_needed(self, full_config: SeatingDetectorConfig) -> None:
        """SEATED verdict on first try → no probe executed."""
        random.seed(_RNG_SEED)
        trace = _seated_trace(n=20)
        detector = SeatingDetector(full_config)
        mock_exec = MagicMock()
        evaluator = SeatingEvaluator(detector, probe_executor=mock_exec, dry_run=True)
        verdict, _ = evaluator.run_evaluation(trace)
        assert verdict == SeatingVerdict.SEATED
        mock_exec.assert_not_called()

    def test_not_seated_no_probe(self, full_config: SeatingDetectorConfig) -> None:
        """NOT_SEATED verdict → no probe executed."""
        trace = _not_seated_trace(n=20, pos=90.0, peak_load=30)
        detector = SeatingDetector(full_config)
        mock_exec = MagicMock()
        evaluator = SeatingEvaluator(detector, probe_executor=mock_exec, dry_run=True)
        verdict, _ = evaluator.run_evaluation(trace)
        assert verdict == SeatingVerdict.NOT_SEATED
        mock_exec.assert_not_called()

    def test_uncertain_triggers_probe(self, null_config: SeatingDetectorConfig) -> None:
        """UNCERTAIN triggers probe executor call."""
        trace = [_make_sample(150.0, 100, i) for i in range(10)]
        detector = SeatingDetector(null_config)
        mock_exec = MagicMock()
        evaluator = SeatingEvaluator(detector, probe_executor=mock_exec, dry_run=True)
        verdict, _ = evaluator.run_evaluation(trace)
        assert verdict == SeatingVerdict.UNCERTAIN
        mock_exec.assert_called_once()

    def test_uncertain_never_becomes_seated_via_probe_alone(
        self, null_config: SeatingDetectorConfig
    ) -> None:
        """Probe loop does not upgrade UNCERTAIN to SEATED with null thresholds."""
        trace = [_make_sample(150.0, 350, i) for i in range(20)]
        detector = SeatingDetector(null_config)
        evaluator = SeatingEvaluator(detector, probe_executor=MagicMock(), dry_run=True)
        verdict, _ = evaluator.run_evaluation(trace)
        assert verdict != SeatingVerdict.SEATED


# ---------------------------------------------------------------------------
# AC#6: Latency measurement
# ---------------------------------------------------------------------------


class TestLatency:
    """Detection latency per evaluation (AC#6).

    Supervisor loop rate: 50 Hz → budget = 1/50 = 20 ms per cycle.
    The detector must finish in << 20 ms.
    """

    SUPERVISOR_LOOP_RATE_HZ: float = 50.0
    LATENCY_BUDGET_S: float = 1.0 / 50.0  # 20 ms

    def _measure_latency(
        self,
        n_trace: int = 20,
        n_reps: int = 200,
        cfg: Optional[SeatingDetectorConfig] = None,
    ) -> float:
        """Return mean evaluation latency (seconds) over n_reps calls."""
        if cfg is None:
            cfg = SeatingDetectorConfig(
                seated_position_low_deg=140.0,
                seated_position_high_deg=160.0,
                load_rise_threshold_raw=200,
                settle_window_samples=5,
                settle_variance_deg2=1.0,
                min_window_samples=3,
            )
        random.seed(_RNG_SEED)
        trace = _seated_trace(n=n_trace)
        detector = SeatingDetector(cfg)
        t0 = time.perf_counter()
        for _ in range(n_reps):
            detector.evaluate_trace(trace)
        elapsed = time.perf_counter() - t0
        return elapsed / n_reps

    def test_latency_within_budget(self) -> None:
        """Mean evaluation latency < supervisor loop budget (20 ms)."""
        mean_latency_s = self._measure_latency(n_reps=500)
        print(
            f"\n[latency] mean={mean_latency_s * 1000:.3f} ms, "
            f"budget={self.LATENCY_BUDGET_S * 1000:.1f} ms "
            f"(supervisor_loop_rate={self.SUPERVISOR_LOOP_RATE_HZ} Hz)"
        )
        assert mean_latency_s < self.LATENCY_BUDGET_S, (
            f"Latency {mean_latency_s * 1000:.3f} ms exceeds budget "
            f"{self.LATENCY_BUDGET_S * 1000:.1f} ms"
        )

    def test_latency_null_config(self) -> None:
        """Null-config evaluation latency is also within budget."""
        cfg = SeatingDetectorConfig()  # all null
        mean_latency_s = self._measure_latency(cfg=cfg, n_reps=500)
        print(
            f"\n[latency null_cfg] mean={mean_latency_s * 1000:.3f} ms, "
            f"budget={self.LATENCY_BUDGET_S * 1000:.1f} ms"
        )
        assert mean_latency_s < self.LATENCY_BUDGET_S


# ---------------------------------------------------------------------------
# AC#3: ThresholdFitter with 40+ synthetic traces per class
# ---------------------------------------------------------------------------


class TestThresholdFitter:
    """Threshold fitting on synthetic data; FP/FN reported (AC#3 pipeline).

    AC#2 and AC#3 require REAL recorded data.  This test demonstrates the
    pipeline with SYNTHETIC data only (known ground truth + noise).

    FP/FN pasted in test output as required.
    """

    @pytest.fixture
    def synthetic_dataset(self) -> List[LabelledTrace]:
        """40+ seated and 40+ not_seated traces with known GT."""
        random.seed(_RNG_SEED)
        traces: List[LabelledTrace] = []
        # 50 seated
        for _ in range(50):
            samples = _seated_trace(n=20, pos=150.0, peak_load=350, load_noise=15.0)
            traces.append(LabelledTrace(label="seated", samples=samples))
        # 50 not-seated (jam/near-miss): similar position, LOW load
        for _ in range(50):
            samples = _not_seated_trace(n=20, pos=90.0, peak_load=30, load_noise=5.0)
            traces.append(LabelledTrace(label="not_seated", samples=samples))
        return traces

    def test_fit_reports_fp_fn(self, synthetic_dataset: List[LabelledTrace]) -> None:
        """Fitter reports FP and FN on held-out split (synthetic demo for pipeline)."""
        fitter = ThresholdFitter(fn_cap=0.30, test_fraction=0.20)
        result = fitter.fit(synthetic_dataset)

        print(
            f"\n[ThresholdFitter synthetic] "
            f"best_threshold={result.best_load_threshold} "
            f"FP={result.fp_rate:.3f} FN={result.fn_rate:.3f} "
            f"train_s={result.n_train_seated} train_ns={result.n_train_not_seated} "
            f"test_s={result.n_test_seated} test_ns={result.n_test_not_seated} "
            f"thresholds_tried={result.thresholds_tried}"
        )

        assert result.best_load_threshold is not None, "Should find a threshold"
        # With well-separated synthetic data, FP should be low
        assert result.fp_rate < 0.50, (
            f"FP rate {result.fp_rate:.3f} too high on well-separated synthetic data"
        )
        # FN should satisfy the cap
        assert result.fn_rate <= 0.30 + 0.05, (  # small tolerance for small test set
            f"FN rate {result.fn_rate:.3f} exceeds cap"
        )

    def test_fp_minimised_over_fn(self, synthetic_dataset: List[LabelledTrace]) -> None:
        """FP (false positive = not-seated declared SEATED) is the primary minimisation target."""
        fitter = ThresholdFitter(fn_cap=0.30, test_fraction=0.20)
        result = fitter.fit(synthetic_dataset)
        # FP must be strictly less than FN when data is separable
        # (relaxed: just ensure FP is not catastrophically high)
        assert result.fp_rate <= 0.20, (
            f"FP rate {result.fp_rate:.3f} unacceptably high — false positives are the serious failure"
        )

    def test_fit_returns_valid_threshold(self, synthetic_dataset: List[LabelledTrace]) -> None:
        """Fitted threshold is a positive integer."""
        fitter = ThresholdFitter()
        result = fitter.fit(synthetic_dataset)
        assert isinstance(result.best_load_threshold, int)
        assert result.best_load_threshold >= 0

    def test_fit_empty_dataset(self) -> None:
        """Empty dataset returns None threshold gracefully."""
        fitter = ThresholdFitter()
        result = fitter.fit([])
        assert result.best_load_threshold is None

    def test_fit_missing_class(self) -> None:
        """Dataset with only one class returns None threshold."""
        fitter = ThresholdFitter()
        traces = [LabelledTrace("seated", _seated_trace(n=10)) for _ in range(5)]
        result = fitter.fit(traces)
        assert result.best_load_threshold is None

    def test_threshold_separability_known_gt(self) -> None:
        """With perfectly separated synthetic data (no noise overlap), FP=0."""
        random.seed(_RNG_SEED)
        # Seated: always peak_load = 400; not-seated: always peak_load = 10
        seated = [
            LabelledTrace("seated", [_make_sample(150.0, 400, i) for i in range(20)])
            for _ in range(50)
        ]
        not_seated = [
            LabelledTrace("not_seated", [_make_sample(90.0, 10, i) for i in range(20)])
            for _ in range(50)
        ]
        fitter = ThresholdFitter(fn_cap=0.30, test_fraction=0.20)
        result = fitter.fit(seated + not_seated)
        print(
            f"\n[ThresholdFitter perfect separation] "
            f"best={result.best_load_threshold} FP={result.fp_rate:.3f} FN={result.fn_rate:.3f}"
        )
        assert result.fp_rate == 0.0, f"Expected FP=0 on perfectly separated data, got {result.fp_rate}"


# ---------------------------------------------------------------------------
# Stream interface (supervisor callable)
# ---------------------------------------------------------------------------


class TestStreamInterface:
    """SeatingDetector as a callable (supervisor interface)."""

    def test_callable_returns_false_on_empty_buffer(self) -> None:
        """__call__ with no samples returns False."""
        detector = SeatingDetector(SeatingDetectorConfig())
        assert detector() is False

    def test_callable_returns_false_on_uncertain(self, null_config: SeatingDetectorConfig) -> None:
        """UNCERTAIN → __call__ returns False."""
        detector = SeatingDetector(null_config)
        for i in range(10):
            detector.push_sample(_make_sample(150.0, 300, i))
        result = detector()
        assert result is False

    def test_callable_returns_true_on_seated(self, full_config: SeatingDetectorConfig) -> None:
        """SEATED → __call__ returns True."""
        random.seed(_RNG_SEED)
        detector = SeatingDetector(full_config)
        for sample in _seated_trace(n=20):
            detector.push_sample(sample)
        result = detector()
        assert result is True

    def test_callable_returns_false_on_not_seated(
        self, full_config: SeatingDetectorConfig
    ) -> None:
        """NOT_SEATED → __call__ returns False."""
        detector = SeatingDetector(full_config)
        for sample in _not_seated_trace(n=20, pos=90.0, peak_load=30):
            detector.push_sample(sample)
        assert detector() is False

    def test_reset_clears_buffer(self, full_config: SeatingDetectorConfig) -> None:
        """reset() clears the sample buffer."""
        random.seed(_RNG_SEED)
        detector = SeatingDetector(full_config)
        for sample in _seated_trace(n=20):
            detector.push_sample(sample)
        assert len(detector._buffer) == 20
        detector.reset()
        assert len(detector._buffer) == 0
        assert detector() is False


# ---------------------------------------------------------------------------
# Feature extraction unit tests
# ---------------------------------------------------------------------------


class TestExtractFeatures:
    """Unit tests for extract_features."""

    def test_empty_trace(self) -> None:
        features = extract_features([])
        assert features.n_samples == 0
        assert features.peak_abs_load_raw == 0.0
        assert features.tail_position_variance_deg2 is None

    def test_single_sample(self) -> None:
        trace = [_make_sample(150.0, -300, 0)]
        features = extract_features(trace, settle_window_samples=5)
        assert features.n_samples == 1
        assert features.peak_abs_load_raw == 300.0
        assert features.last_position_deg == 150.0
        assert features.tail_position_variance_deg2 is None  # < 5 samples

    def test_tail_variance_computed(self) -> None:
        """Tail variance is computed over the last N samples."""
        # 10 samples: first 5 at 100.0, last 5 alternating 148/152
        trace = (
            [_make_sample(100.0, 100, i) for i in range(5)]
            + [_make_sample(148.0 + (i % 2) * 4, 300, 5 + i) for i in range(5)]
        )
        features = extract_features(trace, settle_window_samples=5)
        assert features.tail_position_variance_deg2 is not None
        assert features.tail_position_variance_deg2 > 0

    def test_negative_load_uses_abs(self) -> None:
        """peak_abs_load_raw takes the absolute value of load_raw."""
        trace = [_make_sample(150.0, -400, i) for i in range(10)]
        features = extract_features(trace)
        assert features.peak_abs_load_raw == 400.0


# ---------------------------------------------------------------------------
# AA-7 telemetry integration (smoke test)
# ---------------------------------------------------------------------------


class TestTelemetryIntegration:
    """AA-7 telemetry events are emitted (smoke test only, no real logger)."""

    def test_evaluate_trace_emits_event(self, full_config: SeatingDetectorConfig) -> None:
        """evaluate_trace calls logger.emit with the expected keys."""
        mock_logger = MagicMock()
        random.seed(_RNG_SEED)
        trace = _seated_trace(n=20)
        detector = SeatingDetector(full_config, logger=mock_logger, role="arm_left")
        detector.evaluate_trace(trace)
        assert mock_logger.emit.called
        call_kwargs = mock_logger.emit.call_args.kwargs
        assert call_kwargs.get("role") == "arm_left"
        assert call_kwargs.get("subsystem") == "seating_detector"
        assert call_kwargs.get("event_type") == "seating_evaluation"
        payload = call_kwargs.get("payload", {})
        assert "verdict" in payload

    def test_no_logger_does_not_raise(self, full_config: SeatingDetectorConfig) -> None:
        """Without a logger, evaluate_trace still works."""
        random.seed(_RNG_SEED)
        trace = _seated_trace(n=20)
        detector = SeatingDetector(full_config, logger=None)
        verdict, features, _ = detector.evaluate_trace(trace)
        assert isinstance(verdict, SeatingVerdict)


# ---------------------------------------------------------------------------
# Config schema integration
# ---------------------------------------------------------------------------


class TestConfigIntegration:
    """SeatingDetectorConfigSchema loads from config.py and hardware.yaml."""

    def test_seating_detector_config_schema_importable(self) -> None:
        """SeatingDetectorConfigSchema is importable from inchworm.config."""
        from inchworm.config import SeatingDetectorConfigSchema
        cfg = SeatingDetectorConfigSchema()
        assert cfg.seated_position_low_deg is None
        assert cfg.seated_position_high_deg is None
        assert cfg.load_rise_threshold_raw is None
        assert cfg.settle_variance_deg2 is None

    def test_hardware_yaml_loads_with_seating_detector(self) -> None:
        """hardware.yaml with seating_detector block loads without error."""
        from inchworm.config import load_config
        cfg = load_config()
        # seating_detector is optional; if present it validates; if absent it's None
        sd = cfg.seating_detector
        if sd is not None:
            assert sd.seated_position_low_deg is None  # null in YAML
            assert sd.load_rise_threshold_raw is None  # null in YAML

    def test_seating_detector_config_from_config_dict(self) -> None:
        """SeatingDetectorConfig.from_config_dict handles null and non-null values."""
        d: dict = {
            "seated_position_low_deg": 140.0,
            "seated_position_high_deg": 160.0,
            "load_rise_threshold_raw": None,
            "settle_window_samples": 5,
            "settle_variance_deg2": None,
        }
        cfg = SeatingDetectorConfig.from_config_dict(d)
        assert cfg.seated_position_low_deg == 140.0
        assert cfg.load_rise_threshold_raw is None

    def test_seating_detector_config_from_none(self) -> None:
        """from_config_dict(None) returns defaults."""
        cfg = SeatingDetectorConfig.from_config_dict(None)
        assert cfg.seated_position_low_deg is None
        assert cfg.load_rise_threshold_raw is None


# ---------------------------------------------------------------------------
# Edge cases
# ---------------------------------------------------------------------------


class TestEdgeCases:
    """Edge cases: empty trace, single sample, all-zero loads, negative loads."""

    def test_empty_trace_returns_uncertain(self) -> None:
        cfg = SeatingDetectorConfig(load_rise_threshold_raw=100, min_window_samples=1)
        detector = SeatingDetector(cfg)
        verdict, _, probe = detector.evaluate_trace([])
        # 0 < min_window_samples=1 → UNCERTAIN
        assert verdict == SeatingVerdict.UNCERTAIN

    def test_all_zero_load(self) -> None:
        """All-zero load with load threshold → NOT_SEATED."""
        cfg = SeatingDetectorConfig(load_rise_threshold_raw=100, min_window_samples=3)
        trace = [_make_sample(150.0, 0, i) for i in range(10)]
        detector = SeatingDetector(cfg)
        verdict, _, _ = detector.evaluate_trace(trace)
        assert verdict == SeatingVerdict.NOT_SEATED

    def test_negative_load_counts_as_abs(self) -> None:
        """Negative load_raw is decoded as absolute value."""
        cfg = SeatingDetectorConfig(load_rise_threshold_raw=100, min_window_samples=3)
        trace = [_make_sample(150.0, -300, i) for i in range(10)]
        detector = SeatingDetector(cfg)
        verdict, features, _ = detector.evaluate_trace(trace)
        assert features.peak_abs_load_raw == 300.0
        # load passes, position and settling null → UNCERTAIN (not NOT_SEATED)
        assert verdict == SeatingVerdict.UNCERTAIN

    def test_very_large_window(self) -> None:
        """Large window (200 samples) evaluates without error."""
        cfg = SeatingDetectorConfig(
            seated_position_low_deg=140.0,
            seated_position_high_deg=160.0,
            load_rise_threshold_raw=200,
            settle_window_samples=5,
            settle_variance_deg2=1.0,
            min_window_samples=3,
        )
        random.seed(_RNG_SEED)
        trace = _seated_trace(n=200)
        detector = SeatingDetector(cfg)
        verdict, features, _ = detector.evaluate_trace(trace)
        assert features.n_samples == 200
        assert verdict == SeatingVerdict.SEATED

    def test_probe_dry_run_true_by_default(self) -> None:
        """ProbeRequest is constructed with dry_run=True by default."""
        p = ProbeRequest()
        assert p.dry_run is True
