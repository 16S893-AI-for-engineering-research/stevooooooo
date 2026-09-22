"""Tests for the compensation algorithm (Eqs. 3-5), control and disturbances."""

from __future__ import annotations

import numpy as np
import pytest
from hypothesis import assume, given, settings
from hypothesis import strategies as st

from ffam import params as P
from ffam.compensation import (base_offset_transform, compute_arm_command,
                               free_flyer_transform, predict_base_frame,
                               tcp_from_pose)
from ffam.control import PIDController, PoseController, StateEstimator
from ffam.disturbance import (DisturbanceModel, NozzleFriction,
                              ResidualGravityField)
from ffam.params import PIDGains
from ffam.transforms import compose, identity, invert, translation_of

poses = st.tuples(
    st.floats(min_value=-2.0, max_value=2.0, allow_nan=False),
    st.floats(min_value=-3.0, max_value=3.0, allow_nan=False),
    st.floats(min_value=-0.3, max_value=0.3, allow_nan=False),
)
velocities = st.tuples(
    st.floats(min_value=-0.05, max_value=0.05, allow_nan=False),
    st.floats(min_value=-0.05, max_value=0.05, allow_nan=False),
    st.floats(min_value=-0.05, max_value=0.05, allow_nan=False),
)
targets = st.tuples(
    st.floats(min_value=-0.2, max_value=0.2, allow_nan=False),
    st.floats(min_value=-0.2, max_value=0.2, allow_nan=False),
    st.floats(min_value=0.0, max_value=0.01, allow_nan=False),
)


class TestEquation4:
    """Eq. 4: predicted world-to-base transform, one arm period ahead."""

    @given(poses, velocities)
    def test_zero_velocity_predicts_the_current_pose(self, pose, velocity):
        pose = np.array(pose)
        predicted = predict_base_frame(pose, np.zeros(3), P.ARM_COMMAND_PERIOD_S)
        expected = compose(free_flyer_transform(pose), base_offset_transform())
        assert np.allclose(predicted, expected, atol=1e-12)

    @given(poses, velocities)
    def test_prediction_is_first_order_in_position(self, pose, velocity):
        pose, velocity = np.array(pose), np.array(velocity)
        dt = P.ARM_COMMAND_PERIOD_S
        predicted = translation_of(predict_base_frame(pose, velocity, dt))
        baseline = translation_of(predict_base_frame(pose, np.zeros(3), dt))
        assert np.allclose(predicted[:2] - baseline[:2], dt * velocity[:2],
                           atol=1e-12)

    @given(poses, velocities)
    def test_rotational_terms_are_omitted(self, pose, velocity):
        """Sec. 4.2 explicitly drops the rotational terms; the model must too,
        or it would be a different algorithm from the paper's."""
        pose, velocity = np.array(pose), np.array(velocity)
        dt = P.ARM_COMMAND_PERIOD_S
        spun = np.array([velocity[0], velocity[1], 10.0])
        assert np.allclose(predict_base_frame(pose, velocity, dt),
                           predict_base_frame(pose, spun, dt), atol=1e-12)

    def test_rejects_non_positive_dt(self):
        with pytest.raises(ValueError):
            compute_arm_command(np.zeros(3), np.zeros(3), np.zeros(3),
                                np.zeros(3), dt=0.0)


class TestEquation3:
    """Eq. 3: the arm goal expressed in its own base frame."""

    @given(poses, targets)
    def test_command_lands_the_nozzle_on_target(self, pose, target):
        """The defining property of the whole algorithm: if the free-flyer is
        where the estimator thinks it is, applying the commanded arm pose must
        put the nozzle exactly on the world-frame target."""
        pose, target = np.array(pose), np.array(target)
        command = compute_arm_command(
            target_world=target, previous_target_world=target,
            estimated_pose=pose, estimated_velocity=np.zeros(3),
            dt=P.ARM_COMMAND_PERIOD_S, compensate=True,
        )
        achieved = tcp_from_pose(pose, command.target_in_base)
        assert np.allclose(achieved, target, atol=1e-9)

    @given(poses, targets)
    def test_uncompensated_ignores_the_measured_pose(self, pose, target):
        """The uncompensated baseline must command against the nominal pose, so
        free-flyer error passes straight through to the nozzle."""
        pose, target = np.array(pose), np.array(target)
        assume(np.linalg.norm(pose) > 1e-3)
        command = compute_arm_command(
            target_world=target, previous_target_world=target,
            estimated_pose=pose, estimated_velocity=np.zeros(3),
            dt=P.ARM_COMMAND_PERIOD_S, compensate=False,
            nominal_pose=np.zeros(3),
        )
        nominal_command = compute_arm_command(
            target_world=target, previous_target_world=target,
            estimated_pose=np.zeros(3), estimated_velocity=np.zeros(3),
            dt=P.ARM_COMMAND_PERIOD_S, compensate=True,
        )
        assert np.allclose(command.target_in_base,
                           nominal_command.target_in_base, atol=1e-12)

    @given(poses, targets)
    def test_compensation_beats_no_compensation(self, pose, target):
        """For any free-flyer pose error, compensating must not make nozzle
        accuracy worse. This is the paper's central claim, as a property."""
        pose, target = np.array(pose), np.array(target)
        kwargs = dict(target_world=target, previous_target_world=target,
                      estimated_pose=pose, estimated_velocity=np.zeros(3),
                      dt=P.ARM_COMMAND_PERIOD_S, nominal_pose=np.zeros(3))

        compensated = compute_arm_command(compensate=True, **kwargs)
        uncompensated = compute_arm_command(compensate=False, **kwargs)

        error_with = np.linalg.norm(
            tcp_from_pose(pose, compensated.target_in_base) - target)
        error_without = np.linalg.norm(
            tcp_from_pose(pose, uncompensated.target_in_base) - target)
        assert error_with <= error_without + 1e-9


class TestEquation5:
    """Eq. 5: commanded TCP speed from consecutive targets."""

    @given(targets, targets)
    def test_velocity_is_distance_over_time(self, a, b):
        a, b = np.array(a), np.array(b)
        dt = P.ARM_COMMAND_PERIOD_S
        command = compute_arm_command(b, a, np.zeros(3), np.zeros(3), dt=dt)
        assert command.velocity == pytest.approx(np.linalg.norm(b - a) / dt,
                                                 abs=1e-12)

    @given(targets)
    def test_stationary_target_gives_zero_velocity(self, target):
        target = np.array(target)
        command = compute_arm_command(target, target, np.zeros(3), np.zeros(3),
                                      dt=P.ARM_COMMAND_PERIOD_S)
        assert command.velocity == pytest.approx(0.0, abs=1e-12)

    @given(targets, targets)
    def test_velocity_is_never_negative(self, a, b):
        command = compute_arm_command(np.array(b), np.array(a), np.zeros(3),
                                      np.zeros(3), dt=P.ARM_COMMAND_PERIOD_S)
        assert command.velocity >= 0.0


class TestLeverArm:
    """The 400 mm TCP offset, Sec. 4.1."""

    def test_offset_magnitude_matches_the_paper(self):
        offset = translation_of(base_offset_transform(P.TCP_OFFSET_M))
        assert np.linalg.norm(offset) == pytest.approx(P.TCP_OFFSET_M, abs=1e-12)

    @given(st.floats(min_value=-0.02, max_value=0.02))
    def test_yaw_error_appears_predominantly_in_x(self, dtheta):
        """Sec. 5.1 reports x error over twice y error, and Sec. 4.1 quotes
        "up to 5.5 mm in the x direction" -- so yaw must feed x."""
        assume(abs(dtheta) > 1e-4)
        arm_in_base = np.array([0.0, 0.0, 0.0])
        nominal = tcp_from_pose(np.zeros(3), arm_in_base)
        rotated = tcp_from_pose(np.array([0.0, 0.0, dtheta]), arm_in_base)
        displacement = rotated - nominal
        assert abs(displacement[0]) > abs(displacement[1])

    def test_paper_error_budget_is_reproduced(self):
        """+/- 2 mm position and +/- 0.5 deg attitude at a 400 mm lever sum to
        the 5.5 mm the paper quotes for the x direction."""
        yaw_contribution = P.TCP_OFFSET_M * np.radians(
            P.STATION_KEEPING_ATT_TOL_DEG) * 1000
        total = P.STATION_KEEPING_POS_TOL_MM + yaw_contribution
        assert total == pytest.approx(P.NOZZLE_ERROR_MAX_X_MM, abs=0.1)


class TestPIDController:
    def test_zero_error_gives_zero_output(self):
        pid = PIDController(PIDGains(kp=1.0, ki=1.0, kd=1.0))
        assert pid.update(0.0, 0.01) == pytest.approx(0.0)

    @given(st.floats(min_value=-1.0, max_value=1.0))
    def test_proportional_output_opposes_error_sign(self, error):
        assume(abs(error) > 1e-6)
        pid = PIDController(PIDGains(kp=2.0, ki=0.0, kd=0.0))
        assert np.sign(pid.update(error, 0.01)) == np.sign(error)

    def test_integral_is_clamped(self):
        """Anti-windup matters here: during a friction stick the error persists
        while the body cannot move."""
        pid = PIDController(PIDGains(kp=0.0, ki=1.0, kd=0.0, integral_limit=0.5))
        for _ in range(1000):
            pid.update(1.0, 0.01)
        assert abs(pid.integral) <= 0.5 + 1e-12

    @given(st.floats(min_value=-20.0, max_value=20.0))
    def test_angular_controller_takes_the_short_way(self, error):
        """A yaw error of 359 deg must be treated as -1 deg."""
        pid = PIDController(PIDGains(kp=1.0, ki=0.0, kd=0.0), angular=True)
        direct = pid.update(error, 0.01)
        pid.reset()
        wrapped = pid.update(error + 2.0 * np.pi, 0.01)
        assert direct == pytest.approx(wrapped, abs=1e-9)

    def test_rejects_non_positive_dt(self):
        pid = PIDController(PIDGains(kp=1.0, ki=0.0, kd=0.0))
        with pytest.raises(ValueError):
            pid.update(1.0, 0.0)

    def test_reset_clears_state(self):
        pid = PIDController(PIDGains(kp=1.0, ki=1.0, kd=1.0))
        for _ in range(10):
            pid.update(1.0, 0.01)
        pid.reset()
        assert pid.integral == 0.0

    @given(st.lists(st.floats(min_value=-1.0, max_value=1.0),
                    min_size=1, max_size=60))
    def test_bounded_input_gives_bounded_output(self, errors):
        """BIBO stability: a bounded error sequence cannot produce unbounded
        control effort."""
        pid = PIDController(PIDGains(kp=1.0, ki=1.0, kd=1.0, integral_limit=1.0))
        for error in errors:
            output = pid.update(error, 0.02)
            assert np.isfinite(output)
            # |kp e| + |ki I| + |kd de/dt| with |e| <= 1 and dt = 0.02
            assert abs(output) <= 1.0 + 1.0 + 2.0 / 0.02 + 1e-9


class TestPoseController:
    def test_rejects_malformed_poses(self):
        controller = PoseController()
        with pytest.raises(ValueError):
            controller.update(np.zeros(2), np.zeros(3), 0.02)

    @given(poses)
    def test_on_setpoint_gives_no_wrench(self, pose):
        controller = PoseController()
        wrench = controller.update(np.array(pose), np.array(pose), 0.02)
        assert np.allclose(wrench, 0.0, atol=1e-12)

    @given(poses)
    def test_wrench_pushes_toward_the_setpoint(self, pose):
        pose = np.array(pose)
        assume(np.linalg.norm(pose[:2]) > 1e-3)
        controller = PoseController()
        wrench = controller.update(np.zeros(3), pose, 0.02)
        # Setpoint is the origin, so the force must oppose the displacement.
        assert np.dot(wrench[:2], pose[:2]) <= 0.0


class TestStateEstimator:
    def test_noise_free_measurement_is_exact(self):
        estimator = StateEstimator(noise_m=0.0, noise_rad=0.0)
        pose = np.array([0.1, -0.2, 0.05])
        assert np.allclose(estimator.measure(pose), pose, atol=1e-12)

    def test_recovers_a_constant_velocity(self):
        """Finite differencing plus moving average must converge to the true
        velocity, since Eq. 4 depends on this estimate."""
        estimator = StateEstimator(noise_m=0.0, noise_rad=0.0)
        dt, velocity = 0.02, np.array([0.01, -0.005, 0.0])
        pose = np.zeros(3)
        estimated = np.zeros(3)
        for _ in range(60):
            pose = pose + velocity * dt
            _, estimated = estimator.update(pose, dt)
        assert np.allclose(estimated, velocity, atol=1e-6)

    def test_rejects_invalid_window(self):
        with pytest.raises(ValueError):
            StateEstimator(window=0)

    def test_rejects_non_positive_dt(self):
        with pytest.raises(ValueError):
            StateEstimator().update(np.zeros(3), 0.0)


class TestResidualGravity:
    @given(st.floats(min_value=-2.0, max_value=2.0),
           st.floats(min_value=-3.5, max_value=3.5))
    def test_never_exceeds_the_stated_maximum(self, x, y):
        """Sec. 4.1: "The magnitude of the acceleration can reach 9.6 mm/s^2"."""
        field = ResidualGravityField()
        magnitude = np.linalg.norm(field.acceleration(np.array([x, y])))
        assert magnitude <= P.RESIDUAL_GRAVITY_MAX + 1e-12

    def test_varies_slowly_across_the_table(self):
        """Sec. 4.1: residual gravity "varies slowly across the table"; a
        rapidly varying field would be a different disturbance entirely."""
        field = ResidualGravityField()
        xs = np.linspace(-2.0, 2.0, 60)
        accelerations = np.array([field.acceleration(np.array([x, 0.0])) for x in xs])
        gradients = np.linalg.norm(np.diff(accelerations, axis=0), axis=1)
        assert np.max(gradients) < 0.1 * P.RESIDUAL_GRAVITY_MAX

    def test_direction_varies_across_the_table(self):
        """Sec. 4.1: "whilst the direction varies"."""
        field = ResidualGravityField()
        directions = []
        for x in np.linspace(-2.0, 2.0, 10):
            accel = field.acceleration(np.array([x, 0.0]))
            directions.append(np.arctan2(accel[1], accel[0]))
        assert np.ptp(directions) > 0.1

    def test_equivalent_tilt_is_physically_small(self):
        """The field must correspond to panel tilts of the order the paper
        reports (+/- 0.05 deg/m), not to a visibly sloped table."""
        field = ResidualGravityField()
        assert field.tilt_deg(np.array([0.5, 0.5])) < 0.1

    def test_force_scales_with_mass(self):
        field = ResidualGravityField()
        position = np.array([0.3, 0.2])
        assert np.allclose(field.force(position, mass=40.4),
                           2.0 * field.force(position, mass=20.2), atol=1e-12)


class TestNozzleFriction:
    @given(st.floats(min_value=0.0, max_value=1.0))
    def test_friction_opposes_travel(self, fraction):
        """Coulomb friction must always oppose motion, never assist it."""
        friction = NozzleFriction()
        direction = np.array([1.0, 0.0])
        wrench = friction.wrench(fraction, direction)
        assert np.dot(wrench[:2], direction) < 0.0

    @given(st.floats(min_value=0.0, max_value=1.0))
    def test_magnitude_is_always_at_least_the_baseline(self, fraction):
        friction = NozzleFriction()
        assert friction.magnitude(fraction) >= friction.baseline_n - 1e-12

    def test_snag_is_repeatable_in_position(self):
        """Sec. 5.1: "The point of maximum error was observed to occur at the
        same position for repeated experiments"."""
        friction = NozzleFriction()
        centre = P.NOZZLE_SNAG_PATH_FRACTIONS[0]
        assert friction.magnitude(centre) == friction.magnitude(centre)
        assert friction.magnitude(centre) > 3.0 * friction.baseline_n

    def test_snag_is_localised(self):
        friction = NozzleFriction()
        centre = P.NOZZLE_SNAG_PATH_FRACTIONS[0]
        far = friction.magnitude(centre + 0.3)
        assert far == pytest.approx(friction.baseline_n, rel=1e-3)

    def test_zero_travel_gives_no_friction(self):
        friction = NozzleFriction()
        assert np.allclose(friction.wrench(0.5, np.zeros(2)), 0.0)

    @given(st.floats(min_value=0.0, max_value=1.0))
    def test_offset_nozzle_generates_torque(self, fraction):
        """Friction at a 400 mm lever torques the body -- the mechanism behind
        the large uncompensated printing error.

        Travel is taken along y here, perpendicular to the default lever (which
        points along x). Torque is ``r x F``, so a friction force *parallel* to
        the lever produces no torque at all; only the perpendicular component
        does. Testing with parallel travel would assert something physically
        false.
        """
        friction = NozzleFriction()
        wrench = friction.wrench(fraction, np.array([0.0, 1.0]))
        assert abs(wrench[2]) > 0.0

    @given(st.floats(min_value=0.0, max_value=1.0))
    def test_travel_along_the_lever_produces_no_torque(self, fraction):
        """The complementary case: friction directed along the lever is pure
        force, no moment."""
        friction = NozzleFriction()
        wrench = friction.wrench(fraction, np.array([1.0, 0.0]))
        assert abs(wrench[2]) == pytest.approx(0.0, abs=1e-12)


class TestDisturbanceModel:
    def test_no_friction_when_not_printing(self):
        """The dry run has the printhead disabled and the platform removed."""
        model = DisturbanceModel(seed=0)
        dry = model.wrench(np.zeros(2), np.zeros(3), printing=False)
        model_b = DisturbanceModel(seed=0)
        wet = model_b.wrench(np.zeros(2), np.zeros(3), printing=True,
                             travel_direction=np.array([1.0, 0.0]))
        assert np.linalg.norm(wet[:2]) > np.linalg.norm(dry[:2])

    @given(st.floats(min_value=-1.0, max_value=1.0),
           st.floats(min_value=-1.0, max_value=1.0))
    def test_disturbance_is_always_finite(self, x, y):
        model = DisturbanceModel(seed=0)
        wrench = model.wrench(np.array([x, y]), np.array([0.01, 0.01, 0.001]))
        assert np.all(np.isfinite(wrench))

    def test_is_reproducible_for_a_given_seed(self):
        a = DisturbanceModel(seed=7).wrench(np.zeros(2), np.zeros(3))
        b = DisturbanceModel(seed=7).wrench(np.zeros(2), np.zeros(3))
        assert np.allclose(a, b, atol=1e-15)

    def test_bearing_drag_opposes_motion(self):
        model = DisturbanceModel(seed=0)
        velocity = np.array([0.02, 0.0, 0.0])
        moving = model.wrench(np.zeros(2), velocity)
        stationary = DisturbanceModel(seed=0).wrench(np.zeros(2), np.zeros(3))
        assert moving[0] < stationary[0]
