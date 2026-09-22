"""Tests for G-code interpolation (Fig. 6) and truss geometry.

The Fig. 6 worked example is the single most precisely specified algorithm in
the paper, so it gets an exact test; the surrounding properties are checked with
hypothesis across arbitrary move lengths.
"""

from __future__ import annotations

import numpy as np
import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from ffam import params as P
from ffam.gcode import GCodeMove, build_trajectory, interpolate_move
from ffam.truss import (check_freeform_limits, multi_segment_truss,
                        segment_moves, segment_path, total_truss_length)

TCP_SPEED = P.TCP_SPEED_MM_S / 1000.0
PERIOD = P.ARM_COMMAND_PERIOD_S
STEP = TCP_SPEED * PERIOD

# Lengths from well under one step to a full truss chord.
lengths = st.floats(min_value=1e-5, max_value=0.30, allow_nan=False,
                    allow_infinity=False)


def make_move(length: float) -> GCodeMove:
    return GCodeMove(np.zeros(3), np.array([length, 0.0, 0.0]))


class TestFigure6Example:
    """The paper's worked example, Sec. 4.2 and Fig. 6.

    "In this case, a line with a length of 0.9 mm is to be printed. With a TCP
    velocity of 3.33 mm/s, and a period between robot arm commands of 60 ms, the
    TCP moves approximately 0.2 mm between commands. Intermediate points along
    the line are therefore generated at this interval (denoted M1, M2, etc). The
    final intermediate point would, however, require an update period of 30 ms
    between it and the line end point. To ensure that the minimum period is not
    violated, the last intermediate point is eliminated, increasing the length
    of the final interval to 90 ms."
    """

    def test_nominal_step_is_about_point_two_mm(self):
        assert STEP * 1000 == pytest.approx(0.1998, abs=1e-4)

    def test_produces_three_intermediate_points_then_the_end(self):
        points = interpolate_move(make_move(0.9e-3))
        # M1, M2, M3 plus the end point B: the 4th intermediate is eliminated.
        assert len(points) == 4

    def test_final_interval_is_ninety_milliseconds(self):
        points = interpolate_move(make_move(0.9e-3))
        assert points[-1].duration * 1000 == pytest.approx(90.0, abs=1.0)

    def test_intermediate_points_are_spaced_by_the_nominal_step(self):
        points = interpolate_move(make_move(0.9e-3))
        for i, point in enumerate(points[:-1], start=1):
            assert point.position[0] == pytest.approx(STEP * i, abs=1e-9)

    def test_ends_exactly_at_the_commanded_point(self):
        points = interpolate_move(make_move(0.9e-3))
        assert points[-1].position[0] == pytest.approx(0.9e-3, abs=1e-12)


class TestInterpolationProperties:
    @given(lengths)
    @settings(max_examples=300)
    def test_never_violates_the_minimum_period(self, length):
        """The whole point of the scheme: no command may be sent sooner than
        the arm can accept one."""
        for point in interpolate_move(make_move(length)):
            assert point.duration >= PERIOD - 1e-12

    @given(lengths)
    @settings(max_examples=300)
    def test_always_reaches_the_end_point(self, length):
        points = interpolate_move(make_move(length))
        assert points[-1].position[0] == pytest.approx(length, abs=1e-12)

    @given(lengths)
    @settings(max_examples=300)
    def test_distances_sum_to_the_move_length(self, length):
        """No material may be lost or gained by subdividing."""
        points = interpolate_move(make_move(length))
        positions = np.vstack([np.zeros(3)] + [p.position for p in points])
        travelled = np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=1))
        assert travelled == pytest.approx(length, rel=1e-9, abs=1e-12)

    @given(lengths)
    @settings(max_examples=300)
    def test_monotonic_progress_along_the_line(self, length):
        points = interpolate_move(make_move(length))
        xs = [p.position[0] for p in points]
        assert all(b >= a - 1e-12 for a, b in zip(xs, xs[1:]))

    @given(lengths)
    @settings(max_examples=200)
    def test_speed_never_exceeds_the_commanded_tcp_speed(self, length):
        """Extending the final interval must slow down, never speed up: the
        extruder cannot keep up with a faster move."""
        for point in interpolate_move(make_move(length)):
            assert point.velocity <= TCP_SPEED + 1e-9

    @given(lengths)
    @settings(max_examples=200)
    def test_total_duration_is_at_least_the_ideal_time(self, length):
        points = interpolate_move(make_move(length))
        total = sum(p.duration for p in points)
        assert total >= length / TCP_SPEED - 1e-9

    @given(st.floats(min_value=1e-5, max_value=STEP))
    def test_short_moves_become_a_single_command(self, length):
        points = interpolate_move(make_move(length))
        assert len(points) == 1
        assert points[0].duration >= PERIOD - 1e-12

    def test_zero_length_move_is_handled(self):
        points = interpolate_move(GCodeMove(np.zeros(3), np.zeros(3)))
        assert len(points) == 1

    def test_rejects_non_positive_speed(self):
        with pytest.raises(ValueError):
            interpolate_move(make_move(0.01), speed=0.0)

    def test_rejects_non_positive_period(self):
        with pytest.raises(ValueError):
            interpolate_move(make_move(0.01), period=0.0)


class TestTrajectory:
    def test_path_fraction_spans_zero_to_one(self):
        trajectory = build_trajectory(segment_moves(segment_path()))
        assert trajectory[0].path_fraction == pytest.approx(0.0, abs=1e-9)
        assert trajectory[-1].path_fraction == pytest.approx(1.0, abs=1e-6)

    def test_path_fraction_is_monotonic(self):
        trajectory = build_trajectory(segment_moves(segment_path()))
        fractions = [p.path_fraction for p in trajectory]
        assert all(b >= a - 1e-12 for a, b in zip(fractions, fractions[1:]))

    def test_empty_move_list_gives_empty_trajectory(self):
        assert build_trajectory([]) == []


class TestTrussGeometry:
    def test_single_segment_matches_the_paper_length(self):
        """Sec. 5.1: a 110 mm truss element."""
        path = segment_path(length=P.SEGMENT_LENGTH_MM / 1000.0)
        extent = (path[:, 0].max() - path[:, 0].min()) * 1000
        assert extent == pytest.approx(P.SEGMENT_LENGTH_MM, abs=1e-6)

    def test_seven_segments_make_775_mm(self):
        """Sec. 5.2 and the Conclusion: "7 truss segments into a 775 mm long
        truss"."""
        segments = multi_segment_truss()
        assert len(segments) == P.N_SEGMENTS
        total = total_truss_length(segments) * 1000
        assert total == pytest.approx(P.TOTAL_TRUSS_LENGTH_MM, abs=0.5)

    def test_consecutive_segments_overlap(self):
        """Sec. 5.2: segments must overlap or they cannot fuse into one
        structure."""
        segments = multi_segment_truss()
        for previous, current in zip(segments, segments[1:]):
            previous_end = previous.path[:, 0].max()
            current_start = current.path[:, 0].min()
            assert current_start < previous_end

    def test_respects_freeform_nozzle_limits(self):
        """Sec. 3 / Fig. 3: height <= 5.5 mm and angle < 45 deg, else the
        printhead recontacts printed material."""
        assert check_freeform_limits(segment_path())["ok"]

    @given(st.integers(min_value=1, max_value=12))
    def test_any_segment_count_respects_nozzle_limits(self, n):
        for segment in multi_segment_truss(n_segments=n):
            assert check_freeform_limits(segment.path)["ok"]

    @given(st.integers(min_value=1, max_value=12))
    def test_total_length_grows_with_segment_count(self, n):
        segments = multi_segment_truss(n_segments=n)
        expected = n * P.SEGMENT_ADVANCE_MM
        assert total_truss_length(segments) * 1000 == pytest.approx(expected, abs=0.5)

    def test_detects_an_excessive_height(self):
        bad = np.array([[0.0, 0.0, 0.0], [0.01, 0.0, 0.02]])
        assert not check_freeform_limits(bad)["height_ok"]

    def test_detects_an_excessive_angle(self):
        bad = np.array([[0.0, 0.0, 0.0], [0.001, 0.0, 0.004]])
        assert not check_freeform_limits(bad)["angle_ok"]

    def test_rejects_zero_bays(self):
        with pytest.raises(ValueError):
            segment_path(bays=0)

    def test_rejects_overlap_larger_than_advance(self):
        with pytest.raises(ValueError):
            multi_segment_truss(overlap=1.0)

    def test_path_is_continuous(self):
        """Sec. 5.2: each segment is "printed as a continuous print path", so
        there must be no zero-length steps requiring a retract.

        The upper bound is the full segment length, because the final move is
        the return chord that runs the whole length of the element -- that one
        long move is part of the design, not a gap.
        """
        length = P.SEGMENT_LENGTH_MM / 1000.0
        path = segment_path(length=length)
        steps = np.linalg.norm(np.diff(path, axis=0), axis=1)
        assert np.all(steps > 0.0)
        assert np.all(steps <= length + 1e-12)
