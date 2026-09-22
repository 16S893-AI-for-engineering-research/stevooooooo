"""Free-flyer rigid-body dynamics, thruster allocation and lag.

Implements the plant described in Sec. 2 of Jonckers et al. (2022):

  * planar 3-DoF rigid body, pose q = [p_x, p_y, theta_z]^T
  * eight unidirectional propeller thrusters
  * wrench-to-thruster mapping  u = M f          (Eq. 1)
  * allocation via the pseudoinverse  f = 2 M^dagger u   (Eq. 2)
  * first-order thruster response (Sec. 2: "transfer functions corresponding
    to first-order systems ... longer rise and fall times ... compared to cold
    gas thrusters")
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from . import params as P

__all__ = [
    "FreeFlyerState",
    "ThrusterArray",
    "build_mapping_matrix",
    "allocate_thrusters",
    "rigid_body_step",
]


def build_mapping_matrix(positions: np.ndarray | None = None,
                         directions: np.ndarray | None = None) -> np.ndarray:
    """Build the 3xN mapping matrix M of Eq. 1, ``u = M f``.

    Row 0 and 1 are the x and y force contributions of each thruster; row 2 is
    the torque contribution r x d (the planar cross product).
    """
    if positions is None or directions is None:
        positions, directions = P.thruster_geometry()
    positions = np.asarray(positions, dtype=float)
    directions = np.asarray(directions, dtype=float)
    if positions.shape != directions.shape:
        raise ValueError("positions and directions must have the same shape")
    if positions.ndim != 2 or positions.shape[1] != 2:
        raise ValueError("expected (N, 2) arrays of planar vectors")

    torque = positions[:, 0] * directions[:, 1] - positions[:, 1] * directions[:, 0]
    return np.vstack([directions[:, 0], directions[:, 1], torque])


def allocate_thrusters(wrench: np.ndarray, mapping: np.ndarray,
                       max_force: float = P.THRUSTER_MAX_N,
                       scale: float = P.THRUSTER_SCALE,
                       deadband: float = 0.0,
                       pwm_levels: int = 0) -> np.ndarray:
    """Map a desired wrench to non-negative thruster forces (Eq. 2).

    ``f = 2 M^dagger u``, then clipped to [0, max_force] because the propeller
    thrusters are unidirectional -- which is precisely why the paper carries
    the factor of two (Sec. 2, citing Zappulla et al. 2017).

    Optionally applies the propeller drive non-idealities described in Sec. 2,
    where demanded force is "mapped linearly to the demanded rpm ... and a
    pulse-width modulation (PWM) signal is generated accordingly":

      * ``deadband``: commands below this force produce nothing, because the
        rotor does not spin up. This is what makes the closed loop limit-cycle.
      * ``pwm_levels``: quantise the duty cycle to an integer number of levels.

    Args:
        wrench: desired ``u = (f_x, f_y, tau)``.
        mapping: the matrix M from :func:`build_mapping_matrix`.
        max_force: per-thruster saturation.
        scale: the Eq. 2 factor of 2.
        deadband: minimum effective force, N. 0 disables.
        pwm_levels: PWM quantisation levels. 0 disables.

    Returns:
        Array of N non-negative thruster forces.
    """
    wrench = np.asarray(wrench, dtype=float)
    if wrench.shape != (3,):
        raise ValueError(f"wrench must have shape (3,), got {wrench.shape}")
    forces = scale * np.linalg.pinv(mapping) @ wrench
    forces = np.clip(forces, 0.0, max_force)

    if deadband > 0.0:
        forces = np.where(forces < deadband, 0.0, forces)

    if pwm_levels and pwm_levels > 0 and max_force > 0.0:
        step = max_force / pwm_levels
        forces = np.round(forces / step) * step
        forces = np.clip(forces, 0.0, max_force)

    return forces


@dataclass(frozen=True)
class FreeFlyerState:
    """Full state of the free-flyer.

    Attributes:
        pose: true pose [x, y, theta] in the world frame (m, m, rad).
        velocity: true velocity [vx, vy, omega] (m/s, m/s, rad/s).
        thruster_forces: current (lagged) force of each thruster, N.
        time: simulation time, s.
    """

    pose: np.ndarray
    velocity: np.ndarray
    thruster_forces: np.ndarray
    time: float = 0.0

    @classmethod
    def at_rest(cls, pose: np.ndarray | None = None,
                n_thrusters: int = P.N_THRUSTERS) -> "FreeFlyerState":
        """Create a state at rest with all thrusters off."""
        if pose is None:
            pose = np.zeros(3, dtype=float)
        return cls(
            pose=np.asarray(pose, dtype=float).copy(),
            velocity=np.zeros(3, dtype=float),
            thruster_forces=np.zeros(n_thrusters, dtype=float),
            time=0.0,
        )

    def with_(self, **changes) -> "FreeFlyerState":
        """Return a copy with the given fields replaced."""
        return replace(self, **changes)


class ThrusterArray:
    """Eight unidirectional thrusters with first-order response.

    The array owns both the geometry (mapping matrix M) and the lag state, so
    callers deal in wrenches while the physical asymmetry of unidirectional,
    saturating, laggy thrusters is handled here.
    """

    def __init__(self, tau: float = P.THRUSTER_TAU_S,
                 max_force: float = P.THRUSTER_MAX_N,
                 deadband: float = P.THRUSTER_DEADBAND_N,
                 pwm_levels: int = P.PWM_LEVELS,
                 positions: np.ndarray | None = None,
                 directions: np.ndarray | None = None) -> None:
        if tau <= 0.0:
            raise ValueError("thruster time constant must be positive")
        self.tau = float(tau)
        self.max_force = float(max_force)
        self.deadband = float(deadband)
        self.pwm_levels = int(pwm_levels)
        if positions is None or directions is None:
            positions, directions = P.thruster_geometry()
        self.positions = np.asarray(positions, dtype=float)
        self.directions = np.asarray(directions, dtype=float)
        self.mapping = build_mapping_matrix(self.positions, self.directions)

    @property
    def n_thrusters(self) -> int:
        return self.mapping.shape[1]

    def command(self, wrench: np.ndarray) -> np.ndarray:
        """Return the commanded (pre-lag) thruster forces for a wrench."""
        return allocate_thrusters(wrench, self.mapping, self.max_force,
                                  deadband=self.deadband,
                                  pwm_levels=self.pwm_levels)

    def step(self, current_forces: np.ndarray, wrench: np.ndarray,
             dt: float) -> tuple[np.ndarray, np.ndarray]:
        """Advance the thruster lag one step.

        Returns:
            (new_forces, achieved_wrench). The achieved wrench is ``M f``, which
            differs from the commanded wrench because of lag, saturation and
            unidirectionality -- this mismatch is a real part of the plant.
        """
        commanded = self.command(wrench)
        current_forces = np.asarray(current_forces, dtype=float)
        alpha = 1.0 - np.exp(-dt / self.tau)
        new_forces = current_forces + alpha * (commanded - current_forces)
        achieved = self.mapping @ new_forces
        return new_forces, achieved


def rigid_body_step(state: FreeFlyerState, wrench: np.ndarray,
                    disturbance_force: np.ndarray, dt: float,
                    mass: float = P.MASS_KG,
                    inertia: float = P.INERTIA_ZZ) -> FreeFlyerState:
    """Integrate the planar rigid body one step (semi-implicit Euler).

    Args:
        state: current state.
        wrench: achieved body wrench (f_x, f_y, tau) in the *world* frame.
        disturbance_force: external (f_x, f_y, tau) in the world frame, e.g.
            residual gravity and nozzle friction.
        dt: step, s.
        mass: free-flyer mass, kg.
        inertia: yaw inertia, kg m^2.

    Semi-implicit (symplectic) Euler is used rather than explicit Euler: it
    updates velocity first and integrates position with the *new* velocity,
    which does not inject energy into an oscillatory closed loop the way
    explicit Euler does.
    """
    wrench = np.asarray(wrench, dtype=float)
    disturbance_force = np.asarray(disturbance_force, dtype=float)
    total = wrench + disturbance_force

    accel = np.array([total[0] / mass, total[1] / mass, total[2] / inertia])
    velocity = state.velocity + accel * dt
    pose = state.pose + velocity * dt
    pose[2] = float(np.arctan2(np.sin(pose[2]), np.cos(pose[2])))

    return state.with_(pose=pose, velocity=velocity, time=state.time + dt)
