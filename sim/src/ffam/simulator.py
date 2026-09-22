"""The closed-loop simulator: free-flyer, controller, arm and disturbances.

Two timescales run concurrently, as in the real setup:

  * the free-flyer pose control loop (CONTROL_DT_S, [INFERRED] 20 ms);
  * the arm command loop (ARM_COMMAND_PERIOD_S, 60 ms -- [PAPER] Sec. 4.2);

both integrated on a finer physics step (SIM_DT_S, 2 ms).

The quantity of interest throughout is the *nozzle* error in the world frame,
which is what determines print quality, as opposed to the free-flyer body error.
Fig. 4 of the paper plots both, and the gap between them is the 400 mm lever arm
turning yaw error into position error.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from . import params as P
from .compensation import (base_offset_transform, compute_arm_command,
                           tcp_from_pose)
from .control import PoseController, StateEstimator
from .disturbance import DisturbanceModel, NozzleFriction
from .dynamics import FreeFlyerState, ThrusterArray, rigid_body_step
from .gcode import TrajectoryPoint
from .transforms import invert, compose, transform, translation_of, wrap_angle

__all__ = ["RunLog", "SimulationResult", "simulate_station_keeping",
           "simulate_print", "ARM_STANDOFF_M"]

ARM_STANDOFF_M = 0.150
"""[INFERRED] Distance from the arm base to the print region, m.

The Meca500 has a ~330 mm reach, and Fig. 5 shows the print area in front of
the arm rather than under the free-flyer. 150 mm keeps a 110 mm element
comfortably inside the workspace, matching Sec. 5.1's remark that this size
"could comfortably fit within the workspace of the robotic arm".

This is constrained, not free: Sec. 4.1 fixes the *total* centre-to-TCP
distance at 400 mm, so the arm base mount offset is the remainder,
``P.TCP_OFFSET_M - ARM_STANDOFF_M = 250 mm``. The lever that converts yaw error
into nozzle error is therefore the paper's 400 mm, however the two parts are
split.
"""

ARM_BASE_MOUNT_M = P.TCP_OFFSET_M - ARM_STANDOFF_M
"""[DERIVED] Free-flyer centre to arm base distance, m: 400 - 150 = 250 mm."""


@dataclass
class RunLog:
    """Time series recorded during a run. Lists are appended per physics step."""

    time: list[float] = field(default_factory=list)
    flyer_pose: list[np.ndarray] = field(default_factory=list)
    flyer_error: list[np.ndarray] = field(default_factory=list)
    nozzle_actual: list[np.ndarray] = field(default_factory=list)
    nozzle_desired: list[np.ndarray] = field(default_factory=list)
    nozzle_error: list[np.ndarray] = field(default_factory=list)
    thruster_forces: list[np.ndarray] = field(default_factory=list)
    disturbance: list[np.ndarray] = field(default_factory=list)
    printing: list[bool] = field(default_factory=list)

    def as_arrays(self) -> dict[str, np.ndarray]:
        """Convert the log to a dict of numpy arrays."""
        return {
            "time": np.asarray(self.time, dtype=float),
            "flyer_pose": np.asarray(self.flyer_pose, dtype=float),
            "flyer_error": np.asarray(self.flyer_error, dtype=float),
            "nozzle_actual": np.asarray(self.nozzle_actual, dtype=float),
            "nozzle_desired": np.asarray(self.nozzle_desired, dtype=float),
            "nozzle_error": np.asarray(self.nozzle_error, dtype=float),
            "thruster_forces": np.asarray(self.thruster_forces, dtype=float),
            "disturbance": np.asarray(self.disturbance, dtype=float),
            "printing": np.asarray(self.printing, dtype=bool),
        }


@dataclass
class SimulationResult:
    """Outcome of a run, with the error statistics the paper reports."""

    log: RunLog
    label: str

    @property
    def arrays(self) -> dict[str, np.ndarray]:
        if not hasattr(self, "_arrays"):
            self._arrays = self.log.as_arrays()
        return self._arrays

    def nozzle_error_mm(self) -> np.ndarray:
        """Per-sample nozzle position error magnitude, mm (in-plane)."""
        err = self.arrays["nozzle_error"]
        if err.size == 0:
            return np.zeros(0)
        return np.linalg.norm(err[:, :2], axis=1) * 1000.0

    def axis_error_mm(self, axis: int) -> np.ndarray:
        """Per-sample absolute nozzle error along one axis, mm."""
        err = self.arrays["nozzle_error"]
        if err.size == 0:
            return np.zeros(0)
        return np.abs(err[:, axis]) * 1000.0

    def flyer_error_mm(self) -> np.ndarray:
        """Per-sample free-flyer position error magnitude, mm."""
        err = self.arrays["flyer_error"]
        if err.size == 0:
            return np.zeros(0)
        return np.linalg.norm(err[:, :2], axis=1) * 1000.0

    def flyer_attitude_error_deg(self) -> np.ndarray:
        """Per-sample free-flyer yaw error, deg."""
        err = self.arrays["flyer_error"]
        if err.size == 0:
            return np.zeros(0)
        return np.degrees(err[:, 2])

    def stats(self) -> dict[str, float]:
        """Mean/max/RMS nozzle error in mm, plus per-axis means."""
        err = self.nozzle_error_mm()
        if err.size == 0:
            return {"mean_mm": 0.0, "max_mm": 0.0, "rms_mm": 0.0,
                    "mean_x_mm": 0.0, "mean_y_mm": 0.0}
        return {
            "mean_mm": float(np.mean(err)),
            "max_mm": float(np.max(err)),
            "rms_mm": float(np.sqrt(np.mean(err ** 2))),
            "mean_x_mm": float(np.mean(self.axis_error_mm(0))),
            "mean_y_mm": float(np.mean(self.axis_error_mm(1))),
        }


class _Simulator:
    """Shared machinery for station-keeping and printing runs."""

    def __init__(self, config: P.SimConfig | None = None,
                 nominal_pose: np.ndarray | None = None) -> None:
        self.cfg = config if config is not None else P.SimConfig()
        self.nominal_pose = (np.zeros(3, dtype=float) if nominal_pose is None
                             else np.asarray(nominal_pose, dtype=float).copy())

        self.thrusters = ThrusterArray(tau=self.cfg.thruster_tau,
                                       deadband=self.cfg.thruster_deadband,
                                       pwm_levels=self.cfg.pwm_levels)
        self.controller = PoseController(self.cfg.pid_xy, self.cfg.pid_theta)
        self.estimator = StateEstimator(
            noise_m=self.cfg.tracking_noise_m,
            noise_rad=self.cfg.tracking_noise_rad,
            rng=np.random.default_rng(self.cfg.seed),
        )
        self.disturbances = DisturbanceModel(
            friction=NozzleFriction(baseline_n=self.cfg.nozzle_friction_n,
                                    snag_n=self.cfg.nozzle_snag_n,
                                    tcp_offset=ARM_BASE_MOUNT_M + ARM_STANDOFF_M),
            seed=self.cfg.seed,
        )
        self.base_offset = base_offset_transform(ARM_BASE_MOUNT_M)

        self.state = FreeFlyerState.at_rest(pose=self.nominal_pose,
                                            n_thrusters=self.thrusters.n_thrusters)
        self.log = RunLog()

        # Arm state: commanded and achieved TCP position in the base frame.
        self._arm_target_base = np.zeros(3, dtype=float)
        self._arm_actual_base = np.zeros(3, dtype=float)

    # -- helpers ---------------------------------------------------------

    def _settle(self, duration: float = 12.0) -> None:
        """Run closed-loop with no printing so the integrator absorbs the
        residual-gravity bias before measurements start.

        Without this the first seconds of every run would be dominated by the
        initial transient rather than by steady-state station keeping, and the
        reported means would depend on run length.
        """
        steps = int(round(duration / self.cfg.dt))
        control_every = max(1, int(round(self.cfg.control_dt / self.cfg.dt)))
        wrench = np.zeros(3, dtype=float)
        for i in range(steps):
            if i % control_every == 0:
                pose_est, vel_est = self.estimator.update(self.state.pose,
                                                          self.cfg.control_dt)
                wrench = self.controller.update(self.nominal_pose, pose_est,
                                                self.cfg.control_dt)
            self._physics_step(wrench, printing=False)

    def _physics_step(self, wrench: np.ndarray, printing: bool,
                      path_fraction: float = 0.0,
                      travel_direction: np.ndarray | None = None) -> np.ndarray:
        """Advance thrusters, disturbances and the rigid body by one dt."""
        forces, achieved = self.thrusters.step(self.state.thruster_forces,
                                               wrench, self.cfg.dt)
        theta = float(self.state.pose[2])
        # Arm base is offset along the body +y axis (see
        # compensation.base_offset_transform for why) and the nozzle works at a
        # further standoff from the base. Their sum is the paper's 400 mm
        # centre-to-TCP distance, which is the lever that turns yaw error into
        # nozzle position error.
        lever_length = ARM_BASE_MOUNT_M + ARM_STANDOFF_M
        lever = lever_length * np.array([-np.sin(theta), np.cos(theta)])
        dist = self.disturbances.wrench(
            self.state.pose[:2], self.state.velocity, printing=printing,
            path_fraction=path_fraction, travel_direction=travel_direction,
            nozzle_lever=lever,
        )
        self.state = rigid_body_step(self.state.with_(thruster_forces=forces),
                                     achieved, dist, self.cfg.dt)
        return dist

    def _step_arm(self, dt: float) -> None:
        """First-order arm response toward its commanded TCP pose.

        The Meca500's own repeatability (5 um) is negligible here; what matters
        is the small lag between command and achievement, which sets how well
        the arm can chase a moving free-flyer.
        """
        alpha = 1.0 - np.exp(-dt / P.ARM_LAG_S)
        self._arm_actual_base = (self._arm_actual_base
                                 + alpha * (self._arm_target_base - self._arm_actual_base))

    def _nozzle_world(self) -> np.ndarray:
        """Current true nozzle position in the world frame."""
        return tcp_from_pose(self.state.pose, self._arm_actual_base,
                             base_offset=self.base_offset)


def simulate_station_keeping(duration: float = P.STATION_KEEPING_DURATION_S,
                             config: P.SimConfig | None = None,
                             nominal_pose: np.ndarray | None = None,
                             hold_tcp_base: np.ndarray | None = None
                             ) -> SimulationResult:
    """Reproduce Fig. 4: free-flyer and nozzle deviation while station keeping.

    The arm holds a fixed pose in its own base frame, so the nozzle's world
    position moves with the free-flyer's position *and* yaw. That is exactly the
    comparison Fig. 4 draws: the nozzle deviation exceeds the body deviation
    because of the 400 mm offset.

    Args:
        duration: run length, s (paper: 120 s).
        config: simulation configuration.
        nominal_pose: commanded pose to hold.
        hold_tcp_base: fixed arm TCP position in the base frame.

    Returns:
        A :class:`SimulationResult` whose nozzle error is measured against the
        nozzle position implied by the *nominal* free-flyer pose.
    """
    sim = _Simulator(config=config, nominal_pose=nominal_pose)
    if hold_tcp_base is None:
        # Hold the nozzle at its working standoff, so the logged nozzle
        # deviation reflects the real 400 mm + standoff lever arm that Fig. 4
        # compares against the body deviation.
        hold_tcp_base = np.array([0.0, ARM_STANDOFF_M,
                                  P.LAYER_HEIGHT_MM / 1000.0])
    sim._arm_target_base = np.asarray(hold_tcp_base, dtype=float).copy()
    sim._arm_actual_base = sim._arm_target_base.copy()

    sim._settle()

    nominal_nozzle = tcp_from_pose(sim.nominal_pose, sim._arm_actual_base,
                                   base_offset=sim.base_offset)

    steps = int(round(duration / sim.cfg.dt))
    control_every = max(1, int(round(sim.cfg.control_dt / sim.cfg.dt)))
    wrench = np.zeros(3, dtype=float)

    for i in range(steps):
        if i % control_every == 0:
            pose_est, vel_est = sim.estimator.update(sim.state.pose,
                                                     sim.cfg.control_dt)
            wrench = sim.controller.update(sim.nominal_pose, pose_est,
                                           sim.cfg.control_dt)
        dist = sim._physics_step(wrench, printing=False)
        sim._step_arm(sim.cfg.dt)

        nozzle = sim._nozzle_world()
        pose_error = sim.state.pose - sim.nominal_pose
        pose_error[2] = float(wrap_angle(pose_error[2]))

        sim.log.time.append(sim.state.time)
        sim.log.flyer_pose.append(sim.state.pose.copy())
        sim.log.flyer_error.append(pose_error)
        sim.log.nozzle_actual.append(nozzle)
        sim.log.nozzle_desired.append(nominal_nozzle.copy())
        sim.log.nozzle_error.append(nozzle - nominal_nozzle)
        sim.log.thruster_forces.append(sim.state.thruster_forces.copy())
        sim.log.disturbance.append(dist)
        sim.log.printing.append(False)

    return SimulationResult(log=sim.log, label="station keeping")


def simulate_print(trajectory: list[TrajectoryPoint],
                   compensate: bool = True,
                   printing: bool = True,
                   config: P.SimConfig | None = None,
                   nominal_pose: np.ndarray | None = None,
                   print_origin: np.ndarray | None = None) -> SimulationResult:
    """Run a trajectory, with or without compensation, printing or dry.

    This is the 2x2 experiment of Sec. 5.1: {dry run, print} x {no compensation,
    compensation}. The dry run has the printhead disabled and the platform
    removed, so there is no contact friction; the print run adds it.

    Args:
        trajectory: arm waypoints from :func:`ffam.gcode.build_trajectory`.
        compensate: apply the Eq. 3-5 correction.
        printing: whether the nozzle contacts the substrate (friction on).
        config: simulation configuration.
        nominal_pose: the station-keeping pose held during the print.
        print_origin: world-frame origin of the print, m. Defaults to the
            nominal nozzle position, so the commanded path starts where the
            nozzle already is.

    Returns:
        A :class:`SimulationResult` comparing the achieved nozzle path against
        the desired world-frame path.
    """
    sim = _Simulator(config=config, nominal_pose=nominal_pose)
    if not trajectory:
        return SimulationResult(log=sim.log, label="empty")

    # Start the arm at the first waypoint, expressed in the base frame.
    world_to_base_nominal = compose(
        transform(x=float(sim.nominal_pose[0]), y=float(sim.nominal_pose[1]),
                  theta=float(sim.nominal_pose[2])),
        sim.base_offset,
    )
    if print_origin is None:
        # Place the print so that the nozzle sits at its natural standoff from
        # the arm base, i.e. the arm is extended rather than folded back over
        # the body centre. This matters: if the print were centred on the
        # free-flyer's own centre of rotation, the arm would have to reach back
        # through the base offset, the effective centre-to-nozzle lever would
        # cancel to zero, and free-flyer yaw error would produce no nozzle
        # error at all -- losing the 400 mm lever effect that Sec. 4.1 is
        # explicitly about. The offset below puts the print region in front of
        # the arm base, as in Fig. 5.
        base_world = translation_of(world_to_base_nominal)
        print_origin = base_world + np.array([
            -P.SEGMENT_LENGTH_MM / 2000.0,   # centre the element on the base
            ARM_STANDOFF_M,                  # reach out along +y
            0.0,
        ])

    print_origin = np.asarray(print_origin, dtype=float)

    def to_world(position: np.ndarray) -> np.ndarray:
        """Desired TCP position in the world frame for a path point."""
        return print_origin + np.asarray(position, dtype=float)

    initial_world = to_world(trajectory[0].position)
    point = np.ones(4, dtype=float)
    point[:3] = initial_world
    sim._arm_target_base = (invert(world_to_base_nominal) @ point)[:3]
    sim._arm_actual_base = sim._arm_target_base.copy()

    sim._settle()

    previous_world = initial_world.copy()
    control_every = max(1, int(round(sim.cfg.control_dt / sim.cfg.dt)))
    wrench = np.zeros(3, dtype=float)
    step_counter = 0

    for waypoint in trajectory:
        target_world = to_world(waypoint.position)
        travel = target_world - previous_world
        travel_direction = (travel[:2] if np.linalg.norm(travel[:2]) > 1e-12
                            else np.array([1.0, 0.0]))

        pose_est, vel_est = sim.estimator.update(sim.state.pose, sim.cfg.control_dt)
        command = compute_arm_command(
            target_world=target_world,
            previous_target_world=previous_world,
            estimated_pose=pose_est,
            estimated_velocity=vel_est,
            dt=sim.cfg.arm_command_period,
            compensate=compensate,
            base_offset=sim.base_offset,
            nominal_pose=sim.nominal_pose,
        )
        sim._arm_target_base = command.target_in_base

        n_steps = max(1, int(round(waypoint.duration / sim.cfg.dt)))
        for _ in range(n_steps):
            if step_counter % control_every == 0:
                pose_c, vel_c = sim.estimator.update(sim.state.pose,
                                                      sim.cfg.control_dt)
                wrench = sim.controller.update(sim.nominal_pose, pose_c,
                                               sim.cfg.control_dt)
            step_counter += 1

            dist = sim._physics_step(wrench, printing=printing,
                                     path_fraction=waypoint.path_fraction,
                                     travel_direction=travel_direction)
            sim._step_arm(sim.cfg.dt)

            nozzle = sim._nozzle_world()
            pose_error = sim.state.pose - sim.nominal_pose
            pose_error[2] = float(wrap_angle(pose_error[2]))

            sim.log.time.append(sim.state.time)
            sim.log.flyer_pose.append(sim.state.pose.copy())
            sim.log.flyer_error.append(pose_error)
            sim.log.nozzle_actual.append(nozzle)
            sim.log.nozzle_desired.append(target_world.copy())
            sim.log.nozzle_error.append(nozzle - target_world)
            sim.log.thruster_forces.append(sim.state.thruster_forces.copy())
            sim.log.disturbance.append(dist)
            sim.log.printing.append(printing)

        previous_world = target_world

    label = ("print" if printing else "dry run") + \
            (", compensated" if compensate else ", uncompensated")
    return SimulationResult(log=sim.log, label=label)
