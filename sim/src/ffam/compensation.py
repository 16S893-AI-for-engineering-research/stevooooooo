"""The error correction algorithm of Sec. 4.2 -- Eqs. 3, 4 and 5.

This is the contribution the paper is really about. The free-flyer cannot hold
position better than a few mm, and because the TCP sits ~400 mm from the body
centre, yaw error is levered into further nozzle error (Sec. 4.1: "any
rotational error results in further position error for the nozzle ... the error
is up to 5.5 mm in the x direction. With a nozzle diameter of 1.2 mm, an error
of this magnitude would prevent structures being printed").

Rather than fight that with the thrusters, the arm absorbs it: the arm is
commanded to whatever joint configuration puts the nozzle at the correct point
in the *world* frame, given where the free-flyer currently is and where it is
predicted to be one step from now.

    Eq. 3:  {}^{TCP}_{BRF}T_{N+1} = {}^{TCP}_{W}T_{N+1} . ({}^{BRF}_{W}T_{N+1})^{-1}

    Eq. 4:  {}^{BRF}_{W}T_{N+1} = {}^{BRF}_{FF}T . (X^{FF}_N + dt . Xdot^{FF}_N)

    Eq. 5:  v = (X^{TCP}_{N+1} - X^{TCP}_N) / dt

Per Sec. 4.2, "As the time period between predictions was small (60 ms), and
the rotational velocity was low, the linear acceleration term, and all
rotational terms are omitted" -- so Eq. 4 is a first-order translational
extrapolation only.

Note on Eq. 3 as printed. Taken literally, the right-hand side composes in the
wrong order: with ``A_to_B`` matrices, obtaining base-to-TCP from world-to-TCP
and world-to-base requires ``(world_to_base)^{-1} . world_to_tcp``, i.e. the
inverse on the *left*. The published form puts it on the right, which for
non-commuting rigid transforms gives a different (and dimensionally
meaningless) result -- it only coincides when the free-flyer yaw is zero and
the two frames share an origin. This is almost certainly a notational
transpose in the paper rather than an error in their implementation, since
their reported results could not have been achieved with the literal form. This
module implements the physically correct composition and
``tests/test_compensation.py`` documents the distinction.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from . import params as P
from .transforms import compose, invert, transform, translation_of

__all__ = ["ArmCommand", "predict_base_frame", "compute_arm_command", "tcp_from_pose"]


@dataclass(frozen=True)
class ArmCommand:
    """A command sent to the robotic arm.

    Attributes:
        transform_brf_tcp: the 4x4 transform of Eq. 3, the arm's goal pose
            expressed in its own base reference frame.
        velocity: the commanded linear TCP speed from Eq. 5, m/s.
        target_world: the TCP position in the world frame the command aims at.
    """

    transform_brf_tcp: np.ndarray
    velocity: float
    target_world: np.ndarray

    @property
    def target_in_base(self) -> np.ndarray:
        """The commanded TCP position in the arm base frame, m."""
        return translation_of(self.transform_brf_tcp)


def free_flyer_transform(pose: np.ndarray) -> np.ndarray:
    """World-to-free-flyer transform for a planar pose [x, y, theta]."""
    pose = np.asarray(pose, dtype=float)
    return transform(x=float(pose[0]), y=float(pose[1]), theta=float(pose[2]))


def base_offset_transform(offset: float = P.TCP_OFFSET_M) -> np.ndarray:
    """The fixed free-flyer-to-arm-base transform, ``{}^{BRF}_{FF}T``.

    Sec. 4.2 calls this the transform "from the free-flyer to the base frame of
    the robot, which remains fixed". The arm base is mounted on the payload
    module, offset 400 mm from the geometric centre.

    Offset direction. The paper's Fig. 7 is not dimensioned, but the offset
    direction is recoverable from the reported error statistics. A yaw error
    ``dtheta`` displaces a nozzle at body offset ``d`` by ``d dtheta``
    perpendicular to the offset direction. The paper reports the uncompensated
    error as "over twice as large in the X direction than in the Y direction,
    with values of 0.68 and 0.28 mm" (Sec. 5.1) and "up to 5.5 mm in the x
    direction" (Sec. 4.1) -- i.e. yaw error shows up predominantly in *x*. That
    requires the offset to lie along the body **y** axis, since an offset along
    body x would push yaw error into y instead. The model therefore mounts the
    arm base along +y, which reproduces the observed asymmetry.
    """
    return transform(y=offset)


def predict_base_frame(pose: np.ndarray, velocity: np.ndarray, dt: float,
                       base_offset: np.ndarray | None = None) -> np.ndarray:
    """Eq. 4: predict the world-to-arm-base transform one step ahead.

    Args:
        pose: current free-flyer pose [x, y, theta] in the world frame.
        velocity: current free-flyer velocity [vx, vy, omega].
        dt: prediction horizon, s (the arm command period).
        base_offset: fixed free-flyer-to-base transform; defaults to
            :func:`base_offset_transform`.

    Returns:
        The predicted 4x4 world-to-base transform.

    The rotational terms are deliberately omitted, following the paper: yaw is
    extrapolated as zero-order (held at its current value) while position is
    extrapolated first-order. This is a real, documented approximation and is
    part of why the compensated error does not go to zero.
    """
    pose = np.asarray(pose, dtype=float)
    velocity = np.asarray(velocity, dtype=float)
    if base_offset is None:
        base_offset = base_offset_transform()

    predicted_pose = np.array([
        pose[0] + dt * velocity[0],
        pose[1] + dt * velocity[1],
        pose[2],  # rotational terms omitted, per Sec. 4.2
    ])
    return compose(free_flyer_transform(predicted_pose), base_offset)


def tcp_from_pose(pose: np.ndarray, arm_position_in_base: np.ndarray,
                  base_offset: np.ndarray | None = None) -> np.ndarray:
    """Forward kinematics: where the TCP actually is, in the world frame.

    Args:
        pose: true free-flyer pose [x, y, theta].
        arm_position_in_base: the arm's achieved TCP position in its base
            frame, m (3-vector).
        base_offset: fixed free-flyer-to-base transform.

    Returns:
        World-frame TCP position, 3-vector in metres.
    """
    if base_offset is None:
        base_offset = base_offset_transform()
    world_to_base = compose(free_flyer_transform(pose), base_offset)
    point = np.ones(4, dtype=float)
    point[:3] = np.asarray(arm_position_in_base, dtype=float)
    return (world_to_base @ point)[:3]


def compute_arm_command(target_world: np.ndarray,
                        previous_target_world: np.ndarray,
                        estimated_pose: np.ndarray,
                        estimated_velocity: np.ndarray,
                        dt: float,
                        compensate: bool = True,
                        base_offset: np.ndarray | None = None,
                        nominal_pose: np.ndarray | None = None) -> ArmCommand:
    """Build the arm command for the next trajectory point.

    Args:
        target_world: desired TCP position at step N+1 in the world frame, m.
        previous_target_world: desired TCP position at step N, for Eq. 5.
        estimated_pose: free-flyer pose estimate from the tracking system.
        estimated_velocity: free-flyer velocity estimate.
        dt: arm command period, s.
        compensate: if ``True``, apply Eqs. 3-4 using the measured pose. If
            ``False``, the arm is commanded against the *nominal* pose, which is
            the paper's uncompensated baseline -- the arm behaves as if the
            free-flyer were perfectly on station, so every bit of free-flyer
            error passes straight through to the nozzle.
        base_offset: fixed free-flyer-to-base transform.
        nominal_pose: the commanded station-keeping pose, used when
            ``compensate`` is ``False``. Defaults to the origin.

    Returns:
        The :class:`ArmCommand` to send.
    """
    if dt <= 0.0:
        raise ValueError("dt must be positive")
    if base_offset is None:
        base_offset = base_offset_transform()

    target_world = np.asarray(target_world, dtype=float)
    previous_target_world = np.asarray(previous_target_world, dtype=float)

    if compensate:
        world_to_base = predict_base_frame(estimated_pose, estimated_velocity, dt,
                                           base_offset=base_offset)
    else:
        pose = (np.zeros(3, dtype=float) if nominal_pose is None
                else np.asarray(nominal_pose, dtype=float))
        world_to_base = compose(free_flyer_transform(pose), base_offset)

    # Eq. 3, in the order that actually composes (see module docstring on the
    # transpose in the published equation).
    world_to_tcp = transform(x=float(target_world[0]), y=float(target_world[1]),
                             z=float(target_world[2]))
    brf_to_tcp = compose(invert(world_to_base), world_to_tcp)

    # Eq. 5.
    velocity = float(np.linalg.norm(target_world - previous_target_world) / dt)

    return ArmCommand(transform_brf_tcp=brf_to_tcp, velocity=velocity,
                      target_world=target_world.copy())
