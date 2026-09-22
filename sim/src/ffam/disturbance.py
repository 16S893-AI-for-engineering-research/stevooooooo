"""Disturbance models for the ELISSA air bearing table.

Sec. 4.1 of Jonckers et al. (2022) enumerates the disturbances:

  * **Residual gravitational acceleration** -- the dominant system disturbance,
    "due to the panels making up the air bearing table not being perfectly
    perpendicular to the gravity vector". It "varies slowly across the table,
    as each panel has a maximum inclination variation of +/- 0.05 deg/m. The
    magnitude of the acceleration can reach 9.6 mm/s^2, whilst the direction
    varies."
  * **Aerodynamic forces** from the active air blowing system, plus air bearing
    friction: together "less than 10% of all residual accelerations", with a
    friction coefficient "< 10^-5".
  * **Dynamic coupling** from robotic arm motion, "expected to be small" due to
    slow arm speeds and the arm's low mass relative to the free-flyer.
  * **Nozzle contact friction** while printing -- not listed as a table
    disturbance, but identified in Sec. 5.1 as the cause of the large
    uncompensated printing error.
"""

from __future__ import annotations

import numpy as np

from . import params as P

__all__ = ["ResidualGravityField", "NozzleFriction", "DisturbanceModel"]


class ResidualGravityField:
    """Smooth, spatially varying residual gravity over the table.

    Implemented as a low-order Fourier surface in (x, y) for the tilt angles,
    which gives the two properties the paper specifies: the field varies
    *slowly* in space (so a station-keeping free-flyer sees a near-constant
    bias that the PID integral can reject), and its *direction* varies across
    the table. Magnitude is scaled so the worst case over the table equals the
    stated 9.6 mm/s^2.

    [INFERRED] The paper reports only the bound and the qualitative behaviour;
    the specific spatial pattern here is a model, not measured data.
    """

    def __init__(self, max_accel: float = P.RESIDUAL_GRAVITY_MAX,
                 table_size: tuple[float, float] = P.TABLE_SIZE_M,
                 seed: int = 0) -> None:
        self.max_accel = float(max_accel)
        self.table_size = table_size
        rng = np.random.default_rng(seed)
        # Random phases give a different but statistically similar table per
        # seed; two spatial harmonics keep the field smooth over metres.
        self._phase = rng.uniform(0.0, 2.0 * np.pi, size=4)
        self._weights = rng.uniform(0.5, 1.0, size=2)
        self._norm = self._compute_norm()

    def _raw(self, x: float, y: float) -> np.ndarray:
        """Unnormalised acceleration direction/magnitude at a point."""
        lx, ly = self.table_size
        ax = (self._weights[0] * np.sin(2.0 * np.pi * x / lx + self._phase[0])
              + 0.5 * np.cos(np.pi * y / ly + self._phase[1]))
        ay = (self._weights[1] * np.cos(2.0 * np.pi * y / ly + self._phase[2])
              + 0.5 * np.sin(np.pi * x / lx + self._phase[3]))
        return np.array([ax, ay])

    def _compute_norm(self) -> float:
        """Find the peak raw magnitude over the table, for scaling."""
        lx, ly = self.table_size
        xs = np.linspace(-lx / 2, lx / 2, 41)
        ys = np.linspace(-ly / 2, ly / 2, 41)
        peak = 0.0
        for x in xs:
            for y in ys:
                peak = max(peak, float(np.linalg.norm(self._raw(x, y))))
        return peak if peak > 0.0 else 1.0

    def acceleration(self, position: np.ndarray) -> np.ndarray:
        """Return the residual gravitational acceleration (m/s^2) at a point."""
        position = np.asarray(position, dtype=float)
        raw = self._raw(float(position[0]), float(position[1]))
        return raw * (self.max_accel / self._norm)

    def force(self, position: np.ndarray, mass: float = P.MASS_KG) -> np.ndarray:
        """Return the residual gravity force (N) on the free-flyer."""
        return mass * self.acceleration(position)

    def tilt_deg(self, position: np.ndarray) -> float:
        """Equivalent panel tilt at a point, deg. Sanity-checks the model
        against the paper's +/- 0.05 deg/m inclination figure."""
        accel = np.linalg.norm(self.acceleration(position))
        return float(np.degrees(np.arcsin(np.clip(accel / 9.81, -1.0, 1.0))))


class NozzleFriction:
    """Coulomb friction between the nozzle and the substrate while printing.

    This is the disturbance that dominates Sec. 5.1's uncompensated print run:
    "This increase in error was observed to be caused by friction between the
    nozzle and the print substrate, or already printed material. Any applied
    force disturbs the position of the free-flyer, which then requires some
    time to counteract the disturbance."

    Two components, both [INFERRED, CALIBRATED] -- the paper quantifies
    neither:

      * a baseline drag opposing the nozzle's motion along the print path;
      * a localised high-friction "snag" at a fixed path position, matching
        "The point of maximum error was observed to occur at the same position
        for repeated experiments, and was caused by particularly high friction
        between the nozzle and already printed structure."

    The snag is deterministic in path fraction, not random, precisely because
    the paper stresses its repeatability.
    """

    def __init__(self, baseline_n: float = P.NOZZLE_FRICTION_N,
                 snag_n: float = P.NOZZLE_SNAG_N,
                 snag_fractions: tuple[float, ...] = P.NOZZLE_SNAG_PATH_FRACTIONS,
                 snag_width: float = P.NOZZLE_SNAG_WIDTH,
                 tcp_offset: float = P.TCP_OFFSET_M) -> None:
        self.baseline_n = float(baseline_n)
        self.snag_n = float(snag_n)
        self.snag_fractions = tuple(snag_fractions)
        self.snag_width = float(snag_width)
        self.tcp_offset = float(tcp_offset)

    def magnitude(self, path_fraction: float) -> float:
        """Total friction force magnitude at a normalised path position."""
        total = self.baseline_n
        for centre in self.snag_fractions:
            delta = (path_fraction - centre) / self.snag_width
            total += self.snag_n * float(np.exp(-0.5 * delta * delta))
        return total

    def wrench(self, path_fraction: float, travel_direction: np.ndarray,
               nozzle_lever: np.ndarray | None = None) -> np.ndarray:
        """Return the world-frame wrench (f_x, f_y, tau) from nozzle friction.

        Friction opposes the direction of travel and, because it acts at the
        nozzle rather than the centre of mass, also applies a torque -- which
        is why a contact force at a 400 mm lever arm is so much more damaging
        to nozzle accuracy than an equal force at the centre.

        Args:
            path_fraction: normalised position along the print path, [0, 1].
            travel_direction: unit vector of nozzle travel in the world frame.
            nozzle_lever: vector from free-flyer centre to nozzle in the world
                frame. Defaults to ``[tcp_offset, 0]``.
        """
        travel_direction = np.asarray(travel_direction, dtype=float)
        norm = np.linalg.norm(travel_direction)
        if norm < 1e-12:
            return np.zeros(3, dtype=float)
        unit = travel_direction / norm

        force = -self.magnitude(path_fraction) * unit
        lever = (np.array([self.tcp_offset, 0.0]) if nozzle_lever is None
                 else np.asarray(nozzle_lever, dtype=float))
        torque = lever[0] * force[1] - lever[1] * force[0]
        return np.array([force[0], force[1], torque])


class DisturbanceModel:
    """Aggregate of all disturbances acting on the free-flyer."""

    def __init__(self, gravity: ResidualGravityField | None = None,
                 friction: NozzleFriction | None = None,
                 aero_fraction: float = P.AERO_FRACTION,
                 bearing_friction_coeff: float = P.BEARING_FRICTION_COEFF,
                 seed: int = 0) -> None:
        self.gravity = gravity if gravity is not None else ResidualGravityField(seed=seed)
        self.friction = friction if friction is not None else NozzleFriction()
        self.aero_fraction = float(aero_fraction)
        self.bearing_friction_coeff = float(bearing_friction_coeff)
        self.rng = np.random.default_rng(seed + 1)

    def wrench(self, position: np.ndarray, velocity: np.ndarray,
               printing: bool = False, path_fraction: float = 0.0,
               travel_direction: np.ndarray | None = None,
               nozzle_lever: np.ndarray | None = None,
               mass: float = P.MASS_KG) -> np.ndarray:
        """Total disturbance wrench (f_x, f_y, tau) in the world frame.

        Args:
            position: free-flyer position [x, y], m.
            velocity: free-flyer velocity [vx, vy, omega].
            printing: whether the nozzle is in contact with the substrate.
            path_fraction: normalised print path position, for the snag model.
            travel_direction: nozzle travel direction in the world frame.
            nozzle_lever: centre-to-nozzle vector in the world frame.
            mass: free-flyer mass, kg.
        """
        position = np.asarray(position, dtype=float)
        velocity = np.asarray(velocity, dtype=float)

        total = np.zeros(3, dtype=float)

        gravity_force = self.gravity.force(position[:2], mass=mass)
        total[:2] += gravity_force

        # Aerodynamic disturbance from the air blowing system: bounded by 10% of
        # the residual acceleration and fluctuating, so modelled as noise at
        # that scale rather than as a bias.
        aero_scale = self.aero_fraction * np.linalg.norm(gravity_force)
        total[:2] += self.rng.normal(0.0, aero_scale / 3.0, size=2)
        total[2] += self.rng.normal(0.0, aero_scale * 0.1 / 3.0)

        # Air bearing viscous drag: mu < 1e-5, i.e. all but frictionless.
        speed = np.linalg.norm(velocity[:2])
        if speed > 1e-12:
            drag = self.bearing_friction_coeff * mass * 9.81
            total[:2] -= drag * velocity[:2] / speed

        if printing and travel_direction is not None:
            total += self.friction.wrench(path_fraction, travel_direction,
                                          nozzle_lever=nozzle_lever)
        return total
