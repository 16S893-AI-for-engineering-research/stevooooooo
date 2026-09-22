"""G-code path definition and the Fig. 6 intermediate-point interpolation.

Sec. 4.2 of Jonckers et al. (2022): G-Code end points can be "tens of seconds"
apart, during which the free-flyer drifts, so intermediate points are generated
to give the arm a finer trajectory:

    "The distance between these intermediate points is determined by the
    desired velocity of the end effector, as well as the period between which
    commands could be sent to the robotic arm, in our case 3.33 mm/s and 60 ms,
    respectively. As the Cartesian distance travelled for each G-Code command
    is not exactly divisible by the distance the robotic arm would travel
    during the nominal time step, the length of the final interval was extended
    to accommodate the whole distance."

The worked example (Fig. 6) is the specification this module is tested against:
a 0.9 mm line at 3.33 mm/s with a 60 ms period moves ~0.2 mm per command, so
intermediate points fall every 0.2 mm; the final intermediate point would need
a 30 ms update period, which violates the 60 ms minimum, so it is eliminated
and the final interval becomes 90 ms.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from . import params as P

__all__ = ["GCodeMove", "TrajectoryPoint", "interpolate_move", "build_trajectory"]


@dataclass(frozen=True)
class GCodeMove:
    """A single linear G-code move between two points in 3D space (metres)."""

    start: np.ndarray
    end: np.ndarray
    extruding: bool = True

    @property
    def length(self) -> float:
        """Cartesian length of the move, m."""
        return float(np.linalg.norm(np.asarray(self.end) - np.asarray(self.start)))


@dataclass(frozen=True)
class TrajectoryPoint:
    """One commanded arm waypoint.

    Attributes:
        position: target TCP position in the print frame, m.
        duration: time allotted to reach it from the previous point, s. Always
            >= the arm command period.
        velocity: commanded linear speed, m/s (Eq. 5).
        extruding: whether material is being extruded on this move.
        path_fraction: normalised position along the whole trajectory, [0, 1].
    """

    position: np.ndarray
    duration: float
    velocity: float
    extruding: bool
    path_fraction: float = 0.0


def interpolate_move(move: GCodeMove,
                     speed: float = P.TCP_SPEED_MM_S / 1000.0,
                     period: float = P.ARM_COMMAND_PERIOD_S) -> list[TrajectoryPoint]:
    """Split one G-code move into arm commands, per Sec. 4.2 / Fig. 6.

    The nominal step is ``speed * period``. Points are placed at that interval;
    if the remaining distance to the end point would require an update period
    shorter than ``period``, the last intermediate point is dropped and the
    final interval absorbs the remainder. This guarantees every returned
    duration is >= ``period``, which is the hard constraint the paper is
    working around.

    Args:
        move: the G-code move to subdivide.
        speed: TCP speed, m/s.
        period: minimum arm command period, s.

    Returns:
        List of :class:`TrajectoryPoint`, ending exactly at ``move.end``. A
        zero-length move returns a single point at the end position.
    """
    if speed <= 0.0:
        raise ValueError("speed must be positive")
    if period <= 0.0:
        raise ValueError("period must be positive")

    start = np.asarray(move.start, dtype=float)
    end = np.asarray(move.end, dtype=float)
    total = move.length

    step = speed * period
    if total <= step:
        # Shorter than one command step: a single command, but never faster
        # than the minimum period allows.
        duration = max(total / speed, period)
        return [TrajectoryPoint(end.copy(), duration, total / duration if duration else 0.0,
                                move.extruding)]

    direction = (end - start) / total
    n_full = int(np.floor(total / step))
    remainder = total - n_full * step

    # If the leftover would need less than a full period, drop the last
    # intermediate point and let the final interval cover step + remainder.
    if remainder > 1e-12 and remainder < step:
        n_intermediate = n_full - 1
    else:
        n_intermediate = n_full - 1 if remainder <= 1e-12 else n_full

    points: list[TrajectoryPoint] = []
    for i in range(1, n_intermediate + 1):
        points.append(TrajectoryPoint(start + direction * (step * i), period,
                                      speed, move.extruding))

    covered = step * n_intermediate
    final_distance = total - covered
    final_duration = final_distance / speed
    points.append(TrajectoryPoint(end.copy(), final_duration,
                                  final_distance / final_duration, move.extruding))
    return points


def build_trajectory(moves: list[GCodeMove],
                     speed: float = P.TCP_SPEED_MM_S / 1000.0,
                     period: float = P.ARM_COMMAND_PERIOD_S) -> list[TrajectoryPoint]:
    """Interpolate a list of G-code moves into one arm trajectory.

    Also fills in ``path_fraction`` by cumulative distance, which the friction
    model uses to place the repeatable snag.
    """
    points: list[TrajectoryPoint] = []
    for move in moves:
        points.extend(interpolate_move(move, speed=speed, period=period))

    if not points:
        return points

    # Cumulative arc length -> normalised path fraction.
    positions = np.array([p.position for p in points])
    deltas = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    cumulative = np.concatenate([[0.0], np.cumsum(deltas)])
    total = cumulative[-1] if cumulative[-1] > 0.0 else 1.0

    return [
        TrajectoryPoint(p.position, p.duration, p.velocity, p.extruding,
                        float(c / total))
        for p, c in zip(points, cumulative)
    ]
