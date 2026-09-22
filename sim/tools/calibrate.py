"""Calibrate the [INFERRED] parameters against the paper's reported results.

Three quantities are not published and must be inferred (see params.py):

  1. ``THRUSTER_DEADBAND_N``  -- set by the station-keeping envelope,
     +/- 2 mm and +/- 0.5 deg (Sec. 4.1).
  2. ``NOZZLE_FRICTION_N``    -- set by the uncompensated print mean error,
     8.42 mm (Sec. 5.1).
  3. ``NOZZLE_SNAG_N``        -- set by the uncompensated print max error,
     55.45 mm (Sec. 5.1).

Each is scanned independently, in that order, because they are near-decoupled:
the deadband governs free-flyer wander with no printing at all, while the
friction terms only act during printing and do not feed back into the
station-keeping envelope.

Run:  python3 tools/calibrate.py
"""

from __future__ import annotations

import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from ffam import params as P  # noqa: E402
from ffam.gcode import build_trajectory  # noqa: E402
from ffam.simulator import simulate_print, simulate_station_keeping  # noqa: E402
from ffam.truss import segment_moves, segment_path  # noqa: E402


def dry_trajectory():
    """The Sec. 5.1 dry-run trajectory: a single 110 mm truss element."""
    path = segment_path(length=P.SEGMENT_LENGTH_MM / 1000.0)
    return build_trajectory(segment_moves(path))


def calibrate_deadband(candidates=None, seeds=(0, 1, 2)) -> float:
    """Scan the thruster deadband against the station-keeping envelope."""
    if candidates is None:
        candidates = np.linspace(0.0, 0.10, 21)

    print("\n=== 1. Thruster deadband vs station-keeping envelope ===")
    print("target: mean position error of order 1 mm, peak near +/- 2 mm,")
    print("        attitude within +/- 0.5 deg  (Sec. 4.1, Fig. 4)\n")
    print(f"{'deadband N':>11} {'mean mm':>9} {'p95 mm':>8} {'max mm':>8} "
          f"{'max deg':>8}")

    best, best_cost = None, np.inf
    for deadband in candidates:
        means, peaks, maxima, attitudes = [], [], [], []
        for seed in seeds:
            cfg = P.SimConfig(seed=seed, thruster_deadband=float(deadband))
            result = simulate_station_keeping(duration=40.0, config=cfg)
            err = result.flyer_error_mm()
            means.append(err.mean())
            peaks.append(np.percentile(err, 95))
            maxima.append(err.max())
            attitudes.append(np.abs(result.flyer_attitude_error_deg()).max())

        mean_mm = float(np.mean(means))
        p95_mm = float(np.mean(peaks))
        max_mm = float(np.mean(maxima))
        max_deg = float(np.mean(attitudes))
        print(f"{deadband:11.4f} {mean_mm:9.3f} {p95_mm:8.3f} {max_mm:8.3f} "
              f"{max_deg:8.4f}")

        # Aim for a 95th percentile near 2 mm -- "errors with a magnitude of
        # approximately +/- 2 mm" reads as the envelope, not the mean -- while
        # respecting the attitude limit.
        cost = abs(p95_mm - P.STATION_KEEPING_POS_TOL_MM)
        if max_deg > P.STATION_KEEPING_ATT_TOL_DEG:
            cost += 100.0
        if cost < best_cost:
            best, best_cost = float(deadband), cost

    print(f"\n-> deadband = {best:.4f} N")
    return best


def calibrate_friction(deadband: float, candidates=None, seed: int = 0) -> float:
    """Scan baseline nozzle friction against the 8.42 mm uncompensated mean."""
    if candidates is None:
        candidates = np.linspace(0.02, 0.40, 20)

    target = P.REPORTED["print_uncompensated"].mean_mm
    print("\n=== 2. Nozzle friction vs uncompensated print mean error ===")
    print(f"target: mean = {target} mm  (Sec. 5.1)\n")
    print(f"{'friction N':>11} {'mean mm':>9} {'max mm':>9}")

    trajectory = dry_trajectory()
    best, best_cost = None, np.inf
    for friction in candidates:
        cfg = P.SimConfig(seed=seed, thruster_deadband=deadband,
                          nozzle_friction_n=float(friction), nozzle_snag_n=0.0)
        result = simulate_print(trajectory, compensate=False, printing=True,
                                config=cfg)
        stats = result.stats()
        print(f"{friction:11.4f} {stats['mean_mm']:9.3f} {stats['max_mm']:9.3f}")
        cost = abs(stats["mean_mm"] - target)
        if cost < best_cost:
            best, best_cost = float(friction), cost

    print(f"\n-> baseline friction = {best:.4f} N")
    return best


def calibrate_snag(deadband: float, friction: float, candidates=None,
                   seed: int = 0) -> float:
    """Scan the snag force against the 55.45 mm uncompensated maximum."""
    if candidates is None:
        candidates = np.linspace(0.0, 2.0, 21)

    target = P.REPORTED["print_uncompensated"].max_mm
    print("\n=== 3. Snag force vs uncompensated print maximum error ===")
    print(f"target: max = {target} mm  (Sec. 5.1)\n")
    print(f"{'snag N':>9} {'mean mm':>9} {'max mm':>9}")

    trajectory = dry_trajectory()
    best, best_cost = None, np.inf
    for snag in candidates:
        cfg = P.SimConfig(seed=seed, thruster_deadband=deadband,
                          nozzle_friction_n=friction, nozzle_snag_n=float(snag))
        result = simulate_print(trajectory, compensate=False, printing=True,
                                config=cfg)
        stats = result.stats()
        print(f"{snag:9.3f} {stats['mean_mm']:9.3f} {stats['max_mm']:9.3f}")
        cost = abs(stats["max_mm"] - target)
        if cost < best_cost:
            best, best_cost = float(snag), cost

    print(f"\n-> snag force = {best:.4f} N")
    return best


def main() -> None:
    print(__doc__)
    deadband = calibrate_deadband()
    friction = calibrate_friction(deadband)
    snag = calibrate_snag(deadband, friction)

    print("\n" + "=" * 62)
    print("Calibrated values to write into params.py:")
    print(f"  THRUSTER_DEADBAND_N = {deadband:.4f}")
    print(f"  NOZZLE_FRICTION_N   = {friction:.4f}")
    print(f"  NOZZLE_SNAG_N       = {snag:.4f}")
    print("=" * 62)


if __name__ == "__main__":
    main()
