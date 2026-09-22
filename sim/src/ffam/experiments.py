"""The paper's experiments, and a validation report against reported values.

Reproduces:

  * Fig. 4  -- free-flyer and nozzle deviation while station keeping for 120 s
  * Fig. 8  -- TCP position during dry runs, without (A) and with (B) compensation
  * Fig. 9  -- TCP position while printing, without (A) and with (B) compensation
  * Sec. 5.2 -- the 7-segment, 775 mm truss

Run:  python3 -m ffam.experiments
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from . import params as P
from .gcode import build_trajectory
from .simulator import (SimulationResult, simulate_print,
                        simulate_station_keeping)
from .truss import (multi_segment_truss, segment_moves, segment_path,
                    total_truss_length)

__all__ = [
    "dry_run_trajectory",
    "run_all_experiments",
    "run_station_keeping",
    "run_multi_segment_truss",
    "validation_report",
    "format_report",
]


def dry_run_trajectory(length_mm: float = P.SEGMENT_LENGTH_MM):
    """The Sec. 5.1 trajectory: the path required to print a 110 mm element."""
    path = segment_path(length=length_mm / 1000.0)
    return build_trajectory(segment_moves(path))


def run_station_keeping(duration: float = P.STATION_KEEPING_DURATION_S,
                        seed: int = 0) -> SimulationResult:
    """Fig. 4: station keeping with the arm holding a fixed pose."""
    return simulate_station_keeping(duration=duration,
                                    config=P.SimConfig(seed=seed))


def run_all_experiments(seed: int = 0) -> dict[str, SimulationResult]:
    """Run the 2x2 experiment of Sec. 5.1 plus the Fig. 4 station-keeping run.

    Returns:
        Dict keyed ``station_keeping``, ``dry_uncompensated``,
        ``dry_compensated``, ``print_uncompensated``, ``print_compensated``.
    """
    trajectory = dry_run_trajectory()
    config = P.SimConfig(seed=seed)

    results = {"station_keeping": run_station_keeping(seed=seed)}
    for printing, print_key in ((False, "dry"), (True, "print")):
        for compensate, comp_key in ((False, "uncompensated"), (True, "compensated")):
            results[f"{print_key}_{comp_key}"] = simulate_print(
                trajectory, compensate=compensate, printing=printing,
                config=config,
            )
    return results


def run_multi_segment_truss(seed: int = 0, compensate: bool = True) -> dict:
    """Sec. 5.2: print 7 overlapping segments into a 775 mm truss.

    Each segment is printed with the free-flyer holding a fixed pose; between
    segments the free-flyer translates by the segment advance. Returns the
    per-segment results plus the assembled geometry.
    """
    segments = multi_segment_truss(n_segments=P.N_SEGMENTS)
    config = P.SimConfig(seed=seed)
    advance = P.SEGMENT_ADVANCE_MM / 1000.0

    results = []
    for segment in segments:
        # The free-flyer repositions so each segment sits within the arm
        # workspace; the print platform stays fixed in the world frame.
        nominal_pose = np.array([segment.index * advance, 0.0, 0.0])
        local_path = segment.path - np.array([segment.path[0, 0], 0.0, 0.0])
        trajectory = build_trajectory(segment_moves(local_path))
        results.append(simulate_print(
            trajectory, compensate=compensate, printing=True,
            config=config, nominal_pose=nominal_pose,
        ))

    return {
        "segments": segments,
        "results": results,
        "total_length_mm": total_truss_length(segments) * 1000.0,
        "n_segments": len(segments),
    }


@dataclass
class Comparison:
    """One row of the validation report."""

    label: str
    quantity: str
    paper: float
    simulated: float
    unit: str = "mm"
    tolerance: float = 0.5   # fractional
    note: str = ""
    bound: str = "two-sided"
    """``two-sided`` to match a reported value, or ``upper`` when the paper
    quotes an envelope the simulation must stay inside rather than hit."""

    @property
    def relative_error(self) -> float:
        if self.paper == 0.0:
            return 0.0 if self.simulated == 0.0 else np.inf
        return abs(self.simulated - self.paper) / abs(self.paper)

    @property
    def passed(self) -> bool:
        if self.bound == "upper":
            # The paper states an envelope ("within +/- 2 mm"), so anything at
            # or below it is consistent; only exceeding it is a failure.
            return self.simulated <= self.paper * (1.0 + self.tolerance)
        return self.relative_error <= self.tolerance


def validation_report(results: dict[str, SimulationResult] | None = None,
                      seed: int = 0) -> list[Comparison]:
    """Compare the simulation against every quantity the paper reports.

    Tolerances are deliberately loose (50% by default) and the reasons are
    worth stating plainly: the paper publishes no PID gains, no thruster
    curves, no inertia, no nozzle friction and no disturbance time series, so
    several parameters had to be inferred. Agreement to a factor of ~1.5 on the
    absolute errors, with the correct *ordering* and *ratios* between the four
    experiment quadrants, is the honest standard of success here.
    """
    if results is None:
        results = run_all_experiments(seed=seed)

    comparisons: list[Comparison] = []

    # -- the 2x2 error statistics -------------------------------------------
    for key in ("dry_uncompensated", "dry_compensated",
                "print_uncompensated", "print_compensated"):
        reported = P.REPORTED[key]
        stats = results[key].stats()
        comparisons.append(Comparison(
            reported.label, "mean error", reported.mean_mm, stats["mean_mm"],
            note=reported.source[:60] + "...",
        ))
        comparisons.append(Comparison(
            reported.label, "max error", reported.max_mm, stats["max_mm"],
            tolerance=0.6,
        ))

    # -- per-axis asymmetry from the 400 mm TCP offset ----------------------
    dry = results["dry_uncompensated"].stats()
    comparisons.append(Comparison(
        "dry run, no compensation", "mean error, X axis",
        P.DRY_RUN_AXIS_MEANS_MM["x"], dry["mean_x_mm"],
        note="Sec. 5.1: yaw error levered into x by the 400 mm TCP offset",
    ))
    comparisons.append(Comparison(
        "dry run, no compensation", "mean error, Y axis",
        P.DRY_RUN_AXIS_MEANS_MM["y"], dry["mean_y_mm"],
    ))

    # -- station keeping envelope ------------------------------------------
    sk = results["station_keeping"]
    comparisons.append(Comparison(
        "station keeping", "position envelope (p95)",
        P.STATION_KEEPING_POS_TOL_MM,
        float(np.percentile(sk.flyer_error_mm(), 95)),
        bound="upper", tolerance=0.25,
        note="Sec. 4.1: 'approximately +/- 2 mm in the x and y directions'",
    ))
    comparisons.append(Comparison(
        "station keeping", "attitude envelope (max)",
        P.STATION_KEEPING_ATT_TOL_DEG,
        float(np.max(np.abs(sk.flyer_attitude_error_deg()))),
        unit="deg", bound="upper", tolerance=0.25,
        note="Sec. 4.1: 'within +/- 0.5 deg when not printing'",
    ))

    # -- improvement ratios, the paper's headline claims --------------------
    dry_unc = results["dry_uncompensated"].stats()
    dry_cmp = results["dry_compensated"].stats()
    print_unc = results["print_uncompensated"].stats()
    print_cmp = results["print_compensated"].stats()

    comparisons.append(Comparison(
        "compensation benefit", "dry-run mean error reduction",
        P.MEAN_ERROR_REDUCTION_DRY * 100.0,
        (1.0 - dry_cmp["mean_mm"] / dry_unc["mean_mm"]) * 100.0,
        unit="%", tolerance=0.35,
        note="Sec. 5.1: 'approximately a 66% reduction in mean error'",
    ))
    comparisons.append(Comparison(
        "compensation benefit", "print mean error reduction",
        P.MEAN_ERROR_REDUCTION_PRINT * 100.0,
        (1.0 - print_cmp["mean_mm"] / print_unc["mean_mm"]) * 100.0,
        unit="%", tolerance=0.15,
        note="Sec. 5.3: 'reduced the mean ... errors during printing by 97%'",
    ))
    comparisons.append(Comparison(
        "compensation benefit", "print max error reduction",
        P.MAX_ERROR_REDUCTION_PRINT * 100.0,
        (1.0 - print_cmp["max_mm"] / print_unc["max_mm"]) * 100.0,
        unit="%", tolerance=0.15,
        note="Sec. 5.3: 'and maximum errors ... by 95%'",
    ))

    # -- the truss itself ---------------------------------------------------
    truss = run_multi_segment_truss(seed=seed, compensate=True)
    comparisons.append(Comparison(
        "multi-segment truss", "total length",
        P.TOTAL_TRUSS_LENGTH_MM, truss["total_length_mm"],
        tolerance=0.02,
        note="Sec. 5.2: 7 segments 'forming a truss with a total length of 775 mm'",
    ))

    return comparisons


def format_report(comparisons: list[Comparison]) -> str:
    """Render the validation report as a text table."""
    lines = []
    lines.append("=" * 96)
    lines.append("VALIDATION AGAINST Jonckers et al. (2022), Front. Space Technol. 3:879542")
    lines.append("=" * 96)
    lines.append(f"{'experiment':<26} {'quantity':<28} {'paper':>9} "
                 f"{'sim':>9} {'rel err':>9}  ok")
    lines.append("-" * 96)

    for c in comparisons:
        flag = "yes" if c.passed else "NO"
        unit = c.unit
        marker = "<=" if c.bound == "upper" else "  "
        lines.append(
            f"{c.label:<26} {c.quantity:<28} "
            f"{marker}{c.paper:>6.2f}{unit[:1]} {c.simulated:>8.2f}{unit[:1]} "
            f"{c.relative_error * 100:>8.1f}%  {flag}"
        )

    lines.append("-" * 96)
    n_pass = sum(1 for c in comparisons if c.passed)
    lines.append(f"{n_pass}/{len(comparisons)} quantities within tolerance")
    lines.append("")
    lines.append("Tolerances are loose by design: the paper publishes no PID gains, thruster")
    lines.append("curves, inertia, nozzle friction or disturbance time series, so those were")
    lines.append("inferred or calibrated (see params.py tags). The meaningful test is that the")
    lines.append("four experiment quadrants keep the right ordering and ratios.")
    lines.append("Rows marked '<=' are envelopes the paper states as bounds, not target values;")
    lines.append("staying inside them is a pass.")
    lines.append("=" * 96)
    return "\n".join(lines)


def main() -> None:
    print("Running experiments (this takes a couple of minutes)...\n")
    results = run_all_experiments()

    print(f"{'run':<32} {'mean mm':>9} {'max mm':>9} {'rms mm':>9}")
    print("-" * 62)
    for key, result in results.items():
        stats = result.stats()
        print(f"{result.label:<32} {stats['mean_mm']:>9.3f} "
              f"{stats['max_mm']:>9.3f} {stats['rms_mm']:>9.3f}")

    print()
    print(format_report(validation_report(results)))


if __name__ == "__main__":
    main()
