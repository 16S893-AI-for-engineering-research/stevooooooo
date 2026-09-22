"""Pose control: three decoupled PID controllers plus state estimation.

Sec. 2 and 4.1 of Jonckers et al. (2022): "Three distinct proportional-
integral-derivative (PID) controllers are used for position and attitude
control ... This control type was chosen as the system dynamics of the
free-flyer approximately correspond to that of a first-order system for which
PID control is well suited". The controllers output the wrench
``u = (f_x, f_y, tau)^T`` which is then mapped to the eight thrusters.

State estimation follows Sec. 2: "Finite differencing is used to obtain an
estimate of the free-flyer's velocity which is subsequently smoothed by a
moving average filter."
"""

from __future__ import annotations

from collections import deque

import numpy as np

from . import params as P
from .params import PIDGains
from .transforms import wrap_angle

__all__ = ["PIDController", "PoseController", "StateEstimator"]


class PIDController:
    """Single-axis PID controller with integral clamping.

    The integral term is what rejects the quasi-static residual gravity bias
    (Sec. 4.1: "the PID controller copes well with slowly changing
    disturbances such as the residual gravitational acceleration"), so it
    cannot be dropped -- but it is clamped, because during a friction stick
    event the error persists while the body cannot move, and an unclamped
    integrator would wind up and then overshoot on release.
    """

    def __init__(self, gains: PIDGains, angular: bool = False) -> None:
        self.gains = gains
        self.angular = angular
        self.reset()

    def reset(self) -> None:
        """Clear integral and derivative history."""
        self._integral = 0.0
        self._prev_error: float | None = None

    @property
    def integral(self) -> float:
        return self._integral

    def update(self, error: float, dt: float) -> float:
        """Advance the controller one step and return the control output.

        Args:
            error: setpoint minus measurement. Wrapped to [-pi, pi] if
                ``angular``, so the controller takes the short way round.
            dt: timestep, s.
        """
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        if self.angular:
            error = float(wrap_angle(error))

        self._integral += error * dt
        limit = self.gains.integral_limit
        if np.isfinite(limit):
            self._integral = float(np.clip(self._integral, -limit, limit))

        derivative = 0.0 if self._prev_error is None else (error - self._prev_error) / dt
        self._prev_error = error

        return (self.gains.kp * error
                + self.gains.ki * self._integral
                + self.gains.kd * derivative)


class PoseController:
    """Three PID controllers (x, y, theta) producing a body wrench.

    Matches Fig. 2 of the paper: desired pose X* is differenced with the
    tracked pose, the error drives the PIDs, and the resulting wrench u goes to
    the force allocation module.
    """

    def __init__(self, gains_xy: PIDGains | None = None,
                 gains_theta: PIDGains | None = None) -> None:
        gains_xy = gains_xy if gains_xy is not None else P.PID_XY
        gains_theta = gains_theta if gains_theta is not None else P.PID_THETA
        self.x = PIDController(gains_xy)
        self.y = PIDController(gains_xy)
        self.theta = PIDController(gains_theta, angular=True)

    def reset(self) -> None:
        self.x.reset()
        self.y.reset()
        self.theta.reset()

    def update(self, setpoint: np.ndarray, measured: np.ndarray,
               dt: float) -> np.ndarray:
        """Return the wrench ``u = (f_x, f_y, tau)`` for one control step.

        Args:
            setpoint: desired pose [x, y, theta].
            measured: estimated pose [x, y, theta] from the tracking system.
            dt: control period, s.
        """
        setpoint = np.asarray(setpoint, dtype=float)
        measured = np.asarray(measured, dtype=float)
        if setpoint.shape != (3,) or measured.shape != (3,):
            raise ValueError("poses must have shape (3,)")
        error = setpoint - measured
        return np.array([
            self.x.update(error[0], dt),
            self.y.update(error[1], dt),
            self.theta.update(error[2], dt),
        ])


class StateEstimator:
    """Optical tracking model: noisy pose, finite-differenced velocity.

    Reproduces the estimation chain in Sec. 2. The full estimated state
    ``[q, q_dot]^T`` is what the compensation algorithm consumes (Sec. 4.2,
    Eq. 4), so the velocity estimate quality directly limits how well the arm
    can predict the free-flyer's next pose. The moving-average filter is
    therefore a real contributor to the residual error, not cosmetic.
    """

    def __init__(self, window: int = P.VELOCITY_FILTER_WINDOW,
                 noise_m: float = P.TRACKING_NOISE_M,
                 noise_rad: float = P.TRACKING_NOISE_RAD,
                 rng: np.random.Generator | None = None) -> None:
        if window < 1:
            raise ValueError("filter window must be >= 1")
        self.window = int(window)
        self.noise_m = float(noise_m)
        self.noise_rad = float(noise_rad)
        self.rng = rng if rng is not None else np.random.default_rng(0)
        self.reset()

    def reset(self) -> None:
        self._history: deque[np.ndarray] = deque(maxlen=self.window)
        self._prev_pose: np.ndarray | None = None
        self._velocity = np.zeros(3, dtype=float)

    def measure(self, true_pose: np.ndarray) -> np.ndarray:
        """Return a noisy pose measurement of the true pose."""
        true_pose = np.asarray(true_pose, dtype=float)
        noise = np.array([
            self.rng.normal(0.0, self.noise_m),
            self.rng.normal(0.0, self.noise_m),
            self.rng.normal(0.0, self.noise_rad),
        ])
        measured = true_pose + noise
        measured[2] = float(wrap_angle(measured[2]))
        return measured

    def update(self, true_pose: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
        """Measure the pose and update the filtered velocity estimate.

        Returns:
            (estimated_pose, estimated_velocity).
        """
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        pose = self.measure(true_pose)

        if self._prev_pose is not None:
            delta = pose - self._prev_pose
            delta[2] = float(wrap_angle(delta[2]))
            self._history.append(delta / dt)
            self._velocity = np.mean(np.asarray(self._history), axis=0)

        self._prev_pose = pose
        return pose, self._velocity.copy()
