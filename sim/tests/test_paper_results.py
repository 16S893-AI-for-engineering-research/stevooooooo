"""Regression tests against the numbers Jonckers et al. (2022) report.

These are example-based rather than property-based: each one pins a specific
figure from the paper. They are slow (each runs a full closed-loop print), so
they are marked ``slow``.

Run just these:      pytest -m slow
Skip them:           pytest -m "not slow"
"""

from __future__ import annotations

import numpy as np
import pytest

from ffam import params as P
from ffam.experiments import (dry_run_trajectory, run_all_experiments,
                              run_multi_segment_truss, validation_report)

pytestmark = pytest.mark.slow


@pytest.fixture(scope="module")
def results():
    """Run the 2x2 experiment once and share it across the module."""
    return run_all_experiments(seed=0)


class TestReportedErrorStatistics:
    """Sec. 5.1's four quadrants: {dry, print} x {uncompensated, compensated}.

    Tolerances are wide (a factor of ~1.6) because PID gains, thruster curves,
    inertia and nozzle friction are all unpublished and had to be inferred.
    What must hold is the magnitude and the ordering, not the digits.
    """

    @pytest.mark.parametrize("key", ["dry_uncompensated", "dry_compensated",
                                     "print_uncompensated", "print_compensated"])
    def test_mean_error_is_the_right_magnitude(self, results, key):
        reported = P.REPORTED[key]
        simulated = results[key].stats()["mean_mm"]
        assert simulated == pytest.approx(reported.mean_mm, rel=0.6), (
            f"{reported.label}: paper {reported.mean_mm} mm, "
            f"simulated {simulated:.3f} mm\n{reported.source}"
        )

    @pytest.mark.parametrize("key", ["dry_uncompensated", "print_uncompensated"])
    def test_max_error_is_the_right_magnitude(self, results, key):
        reported = P.REPORTED[key]
        simulated = results[key].stats()["max_mm"]
        assert simulated == pytest.approx(reported.max_mm, rel=0.7)

    def test_printing_is_far_worse_than_dry_without_compensation(self, results):
        """Sec. 5.1: the print mean error is "approximately an 11-fold increase
        compared to the dry run". Friction, not control, dominates."""
        dry = results["dry_uncompensated"].stats()["mean_mm"]
        printing = results["print_uncompensated"].stats()["mean_mm"]
        assert printing / dry > 5.0

    def test_compensated_print_is_close_to_compensated_dry(self, results):
        """Sec. 5.1: with correction, printing costs only "an increase of 4%"
        over the dry run -- the algorithm absorbs the friction disturbance."""
        dry = results["dry_compensated"].stats()["mean_mm"]
        printing = results["print_compensated"].stats()["mean_mm"]
        assert printing / dry < 2.0

    def test_compensation_helps_in_every_condition(self, results):
        for condition in ("dry", "print"):
            uncompensated = results[f"{condition}_uncompensated"].stats()["mean_mm"]
            compensated = results[f"{condition}_compensated"].stats()["mean_mm"]
            assert compensated < uncompensated


class TestAxisAsymmetry:
    """Sec. 5.1: "the mean error being over twice as large in the X direction
    than in the Y direction, with values of 0.68 and 0.28 mm respectively"."""

    def test_x_error_exceeds_y_error(self, results):
        stats = results["dry_uncompensated"].stats()
        assert stats["mean_x_mm"] > stats["mean_y_mm"]

    def test_asymmetry_ratio_is_about_two(self, results):
        stats = results["dry_uncompensated"].stats()
        ratio = stats["mean_x_mm"] / stats["mean_y_mm"]
        paper_ratio = (P.DRY_RUN_AXIS_MEANS_MM["x"] / P.DRY_RUN_AXIS_MEANS_MM["y"])
        assert ratio == pytest.approx(paper_ratio, rel=0.6)

    def test_axis_means_are_the_right_magnitude(self, results):
        stats = results["dry_uncompensated"].stats()
        assert stats["mean_x_mm"] == pytest.approx(
            P.DRY_RUN_AXIS_MEANS_MM["x"], rel=0.6)
        assert stats["mean_y_mm"] == pytest.approx(
            P.DRY_RUN_AXIS_MEANS_MM["y"], rel=0.6)


class TestImprovementRatios:
    """Sec. 5.1 and 5.3 quote the improvements as percentages, which are
    scale-free and therefore the most meaningful comparison available."""

    def test_dry_run_mean_reduction_near_66_percent(self, results):
        uncompensated = results["dry_uncompensated"].stats()["mean_mm"]
        compensated = results["dry_compensated"].stats()["mean_mm"]
        reduction = 1.0 - compensated / uncompensated
        assert reduction == pytest.approx(P.MEAN_ERROR_REDUCTION_DRY, abs=0.2)

    def test_print_mean_reduction_near_97_percent(self, results):
        uncompensated = results["print_uncompensated"].stats()["mean_mm"]
        compensated = results["print_compensated"].stats()["mean_mm"]
        reduction = 1.0 - compensated / uncompensated
        assert reduction == pytest.approx(P.MEAN_ERROR_REDUCTION_PRINT, abs=0.05)

    def test_print_max_reduction_near_95_percent(self, results):
        uncompensated = results["print_uncompensated"].stats()["max_mm"]
        compensated = results["print_compensated"].stats()["max_mm"]
        reduction = 1.0 - compensated / uncompensated
        assert reduction == pytest.approx(P.MAX_ERROR_REDUCTION_PRINT, abs=0.05)


class TestStationKeeping:
    """Sec. 4.1 and Fig. 4."""

    def test_position_stays_within_the_stated_envelope(self, results):
        """"errors with a magnitude of approximately +/- 2 mm in the x and y
        directions". Treated as an upper bound."""
        errors = results["station_keeping"].flyer_error_mm()
        assert float(np.percentile(errors, 95)) <= P.STATION_KEEPING_POS_TOL_MM * 1.25

    def test_attitude_stays_within_the_stated_envelope(self, results):
        """"maintains orientation about the z axis within +/- 0.5 deg when not
        printing"."""
        attitude = np.abs(results["station_keeping"].flyer_attitude_error_deg())
        assert float(np.max(attitude)) <= P.STATION_KEEPING_ATT_TOL_DEG * 1.25

    def test_nozzle_error_exceeds_body_error(self, results):
        """The point of Fig. 4: the 400 mm offset amplifies body error at the
        nozzle, which is what motivates the whole correction algorithm."""
        result = results["station_keeping"]
        assert result.nozzle_error_mm().max() > result.flyer_error_mm().max()


class TestLargeStructure:
    """Sec. 5.2: 7 segments joined into a 775 mm truss."""

    def test_truss_reaches_775_mm(self):
        truss = run_multi_segment_truss(seed=0, compensate=True)
        assert truss["total_length_mm"] == pytest.approx(
            P.TOTAL_TRUSS_LENGTH_MM, abs=1.0)

    def test_seven_segments(self):
        truss = run_multi_segment_truss(seed=0, compensate=True)
        assert truss["n_segments"] == P.N_SEGMENTS

    def test_every_segment_prints_accurately_enough_to_join(self):
        """Sec. 5.2 needs material deposited onto the previous segment; an error
        approaching the nozzle diameter would miss the join (Sec. 5.1: "If the
        error were this large at a node of the truss, it would result in the
        join being missed")."""
        truss = run_multi_segment_truss(seed=0, compensate=True)
        for result in truss["results"]:
            assert result.stats()["mean_mm"] < P.NOZZLE_DIAMETER_MM

    def test_truss_is_longer_than_the_arm_workspace(self):
        """The headline capability: a structure larger than the workspace that
        produced it."""
        truss = run_multi_segment_truss(seed=0, compensate=True)
        assert truss["total_length_mm"] > 5.0 * P.SEGMENT_LENGTH_MM


class TestValidationReport:
    def test_most_quantities_are_within_tolerance(self, results):
        comparisons = validation_report(results)
        passed = sum(1 for c in comparisons if c.passed)
        assert passed >= 0.8 * len(comparisons), "\n".join(
            f"{c.label} / {c.quantity}: paper {c.paper}, sim {c.simulated:.3f}"
            for c in comparisons if not c.passed
        )

    def test_report_renders(self, results):
        from ffam.experiments import format_report
        text = format_report(validation_report(results))
        assert "VALIDATION" in text
        assert "879542" in text


class TestDeterminism:
    def test_same_seed_gives_identical_results(self):
        """Reproducibility is not optional for a recreation."""
        trajectory = dry_run_trajectory()
        from ffam.simulator import simulate_print
        config = P.SimConfig(seed=42)
        first = simulate_print(trajectory, compensate=True, printing=True,
                               config=config).stats()
        second = simulate_print(trajectory, compensate=True, printing=True,
                                config=P.SimConfig(seed=42)).stats()
        assert first == pytest.approx(second, abs=1e-12)

    def test_different_seeds_give_different_noise(self):
        trajectory = dry_run_trajectory()
        from ffam.simulator import simulate_print
        a = simulate_print(trajectory, compensate=True, printing=True,
                           config=P.SimConfig(seed=1)).stats()["mean_mm"]
        b = simulate_print(trajectory, compensate=True, printing=True,
                           config=P.SimConfig(seed=2)).stats()["mean_mm"]
        assert a != b
