"""Property-based tests for transforms and thruster allocation.

These are the invariant tests: they assert properties that must hold for *any*
input, which is where hypothesis earns its place over example-based tests. The
numeric comparisons against the paper's figures live in
``test_paper_results.py``.
"""

from __future__ import annotations

import numpy as np
import pytest
from hypothesis import assume, given, settings
from hypothesis import strategies as st

from ffam import params as P
from ffam.dynamics import (ThrusterArray, allocate_thrusters,
                           build_mapping_matrix)
from ffam.transforms import (apply, compose, identity, invert, transform,
                             translation_of, wrap_angle, yaw_of)

# Bounded to the physical scale of the problem: the ELISSA table is 4 x 7 m.
coords = st.floats(min_value=-10.0, max_value=10.0, allow_nan=False,
                   allow_infinity=False)
angles = st.floats(min_value=-4.0 * np.pi, max_value=4.0 * np.pi,
                   allow_nan=False, allow_infinity=False)
wrenches = st.tuples(
    st.floats(min_value=-5.0, max_value=5.0, allow_nan=False),
    st.floats(min_value=-5.0, max_value=5.0, allow_nan=False),
    st.floats(min_value=-1.0, max_value=1.0, allow_nan=False),
)


class TestWrapAngle:
    @given(angles)
    def test_always_in_range(self, theta):
        """Wrapping must always land in [-pi, pi]."""
        wrapped = wrap_angle(theta)
        assert -np.pi - 1e-9 <= wrapped <= np.pi + 1e-9

    @given(angles)
    def test_preserves_direction(self, theta):
        """Wrapping changes the angle only by whole turns."""
        wrapped = wrap_angle(theta)
        turns = (theta - wrapped) / (2.0 * np.pi)
        assert abs(turns - round(turns)) < 1e-9

    @given(angles)
    def test_idempotent(self, theta):
        once = wrap_angle(theta)
        assert wrap_angle(once) == pytest.approx(once, abs=1e-12)


class TestTransforms:
    @given(coords, coords, angles)
    def test_inverse_round_trip(self, x, y, theta):
        """T . T^-1 = I -- the property Eq. 3 depends on."""
        t = transform(x, y, theta)
        assert np.allclose(compose(t, invert(t)), identity(), atol=1e-9)
        assert np.allclose(compose(invert(t), t), identity(), atol=1e-9)

    @given(coords, coords, angles)
    def test_analytic_inverse_matches_numeric(self, x, y, theta):
        """The analytic rigid inverse must equal a general matrix inverse."""
        t = transform(x, y, theta)
        assert np.allclose(invert(t), np.linalg.inv(t), atol=1e-9)

    @given(coords, coords, angles)
    def test_round_trip_a_point(self, x, y, theta):
        """Mapping a point out and back recovers it exactly."""
        t = transform(x, y, theta)
        point = np.array([0.1, -0.2, 0.05])
        assert np.allclose(apply(invert(t), apply(t, point)), point, atol=1e-9)

    @given(coords, coords, angles)
    def test_extractors_invert_the_constructor(self, x, y, theta):
        t = transform(x, y, theta)
        assert translation_of(t)[:2] == pytest.approx([x, y], abs=1e-9)
        assert yaw_of(t) == pytest.approx(wrap_angle(theta), abs=1e-9)

    @given(coords, coords, angles, coords, coords, angles)
    def test_composition_is_associative(self, x1, y1, t1, x2, y2, t2):
        a, b = transform(x1, y1, t1), transform(x2, y2, t2)
        c = transform(0.3, -0.4, 0.5)
        assert np.allclose(compose(compose(a, b), c), compose(a, compose(b, c)),
                           atol=1e-9)

    @given(coords, coords, angles)
    def test_rotation_block_is_orthonormal(self, x, y, theta):
        """A rigid transform must not scale or shear."""
        rot = transform(x, y, theta)[:3, :3]
        assert np.allclose(rot @ rot.T, np.eye(3), atol=1e-9)
        assert np.linalg.det(rot) == pytest.approx(1.0, abs=1e-9)

    @given(coords, coords, angles)
    def test_preserves_distances(self, x, y, theta):
        t = transform(x, y, theta)
        p, q = np.array([0.4, 0.1, 0.0]), np.array([-0.2, 0.3, 0.1])
        before = np.linalg.norm(p - q)
        after = np.linalg.norm(apply(t, p) - apply(t, q))
        assert after == pytest.approx(before, abs=1e-9)


class TestMappingMatrix:
    def test_shape_and_rank(self):
        """M must be 3 x 8 and rank 3, else 3-DoF control is impossible."""
        mapping = build_mapping_matrix()
        assert mapping.shape == (3, P.N_THRUSTERS)
        assert np.linalg.matrix_rank(mapping) == 3

    def test_rejects_mismatched_geometry(self):
        with pytest.raises(ValueError):
            build_mapping_matrix(np.zeros((4, 2)), np.zeros((3, 2)))

    def test_rejects_non_planar_vectors(self):
        with pytest.raises(ValueError):
            build_mapping_matrix(np.zeros((4, 3)), np.zeros((4, 3)))


class TestThrusterAllocation:
    @given(wrenches)
    @settings(max_examples=200)
    def test_allocation_round_trip(self, wrench):
        """Eq. 1 and Eq. 2 must be consistent: M (2 M^+ u) recovers u.

        This is the core algebraic claim of the paper's allocation scheme. It is
        tested without deadband or PWM quantisation, which are hardware
        non-idealities layered on top and deliberately break exactness.
        """
        wrench = np.array(wrench)
        mapping = build_mapping_matrix()
        forces = allocate_thrusters(wrench, mapping, max_force=np.inf)
        assert np.allclose(mapping @ forces, wrench, atol=1e-9)

    @given(wrenches)
    def test_forces_never_negative(self, wrench):
        """Propeller thrusters are unidirectional -- negative thrust is
        unphysical and would silently flatter the controller."""
        forces = allocate_thrusters(np.array(wrench), build_mapping_matrix())
        assert np.all(forces >= 0.0)

    @given(wrenches)
    def test_forces_respect_saturation(self, wrench):
        forces = allocate_thrusters(np.array(wrench), build_mapping_matrix(),
                                    max_force=P.THRUSTER_MAX_N)
        assert np.all(forces <= P.THRUSTER_MAX_N + 1e-12)

    @given(wrenches, st.floats(min_value=0.01, max_value=0.2))
    def test_deadband_only_zeroes_small_commands(self, wrench, deadband):
        """Deadband must either pass a force through or zero it, never alter it."""
        mapping = build_mapping_matrix()
        plain = allocate_thrusters(np.array(wrench), mapping,
                                   max_force=P.THRUSTER_MAX_N, pwm_levels=0)
        gated = allocate_thrusters(np.array(wrench), mapping,
                                   max_force=P.THRUSTER_MAX_N,
                                   deadband=deadband, pwm_levels=0)
        for before, after in zip(plain, gated):
            assert after == pytest.approx(0.0) or after == pytest.approx(before)

    @given(wrenches)
    def test_pwm_quantisation_is_bounded(self, wrench):
        """Quantisation error cannot exceed half a PWM step."""
        mapping = build_mapping_matrix()
        step = P.THRUSTER_MAX_N / P.PWM_LEVELS
        plain = allocate_thrusters(np.array(wrench), mapping,
                                   max_force=P.THRUSTER_MAX_N, pwm_levels=0)
        quantised = allocate_thrusters(np.array(wrench), mapping,
                                       max_force=P.THRUSTER_MAX_N,
                                       pwm_levels=P.PWM_LEVELS)
        assert np.all(np.abs(quantised - plain) <= step / 2 + 1e-12)

    def test_zero_wrench_gives_zero_thrust(self):
        forces = allocate_thrusters(np.zeros(3), build_mapping_matrix())
        assert np.allclose(forces, 0.0)

    @given(st.floats(min_value=0.1, max_value=3.0))
    def test_scaling_a_wrench_scales_the_forces(self, k):
        """Allocation is linear before clipping, so scaling must pass through."""
        mapping = build_mapping_matrix()
        wrench = np.array([0.2, -0.1, 0.03])
        base = allocate_thrusters(wrench, mapping, max_force=np.inf)
        scaled = allocate_thrusters(k * wrench, mapping, max_force=np.inf)
        assert np.allclose(scaled, k * base, atol=1e-9)

    def test_rejects_bad_wrench_shape(self):
        with pytest.raises(ValueError):
            allocate_thrusters(np.zeros(2), build_mapping_matrix())


class TestThrusterArray:
    def test_lag_approaches_command_monotonically(self):
        """First-order lag must converge to the commanded force without
        overshoot -- an overshoot here would be a sign error.

        Run long enough to actually converge: with tau = 0.2 s, closing to 1e-3
        of a ~0.15 N step takes about 5 time constants, i.e. ~1 s at dt = 2 ms.
        """
        array = ThrusterArray(deadband=0.0, pwm_levels=0)
        wrench = np.array([0.3, 0.0, 0.0])
        target = array.command(wrench)
        forces = np.zeros(array.n_thrusters)

        previous_gap = np.inf
        for _ in range(1500):
            forces, _ = array.step(forces, wrench, 0.002)
            gap = float(np.max(np.abs(target - forces)))
            assert gap <= previous_gap + 1e-12
            previous_gap = gap
        assert np.allclose(forces, target, atol=1e-3)

    def test_rejects_non_positive_tau(self):
        with pytest.raises(ValueError):
            ThrusterArray(tau=0.0)

    @given(wrenches)
    def test_achieved_wrench_is_consistent_with_forces(self, wrench):
        """Whatever the lag state, the achieved wrench must equal M f."""
        array = ThrusterArray()
        forces = np.full(array.n_thrusters, 0.05)
        new_forces, achieved = array.step(forces, np.array(wrench), 0.002)
        assert np.allclose(achieved, array.mapping @ new_forces, atol=1e-12)
