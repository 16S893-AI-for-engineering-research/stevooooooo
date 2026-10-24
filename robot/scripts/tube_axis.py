#!/usr/bin/env python3
"""AA-16: Tube-axis calibration (PCA line fit through attachment-point centres).

Procedure
---------
This script implements AC#3 of AA-16.  In **dry-run** mode (default) it prints
the procedure instructions; in **live** mode (``--live``) it collects
attachment-point centre positions (measured with calipers) and runs the PCA
line fit (``inchworm.calibration.fit_tube_axis``).

The tube is movable: the script records a single set of measurements at one
tube position, then expresses the axis relative to both the chassis frame and
the FrameModel lattice index.  The pitch estimate is compared to the configured
``ATTACHMENT_PITCH_MM`` as a sanity check.

Instruments needed
------------------
* Digital calipers (≥ 0.1 mm resolution)
* At least 3 attachment-point centres measured in the chassis frame.
  (Use a reference flat surface and measure from the chassis datum.)

Safety invariants
-----------------
* **Dry-run by default** — ``--live`` must be passed explicitly.
* No hardware is opened in dry-run mode.
* No motion is commanded.

Usage
-----
Dry-run (default — no hardware)::

    uv run python scripts/tube_axis.py

Live (REQUIRES owner supervision)::

    uv run python scripts/tube_axis.py --live
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

_repo_root = Path(__file__).resolve().parents[1]
_src_path = _repo_root / "src"
if str(_src_path) not in sys.path:
    sys.path.insert(0, str(_src_path))

logging.basicConfig(level=logging.INFO, format="%(levelname)s  %(name)s  %(message)s")
logger = logging.getLogger("tube_axis")

_DRY_RUN_TEXT = """
================================================================================
  AA-16 Tube-Axis Calibration  —  DRY-RUN (no hardware)
================================================================================

  Pass --live to collect real measurements (no hardware motion is commanded).

  Overview
  --------
  The tube can be at any position along the chassis.  This procedure fits a
  line through the measured attachment-point centre positions to determine the
  tube axis direction and the position of lattice index 0 in the chassis frame.

  The result is stored in calibration.tube_axis in hardware.yaml.

  Instruments needed (see docs/calibration_procedure.md)
  -------------------------------------------------------
  1. Digital calipers (>= 0.1 mm resolution, >= 200 mm reach).
  2. A chassis reference datum (e.g. a bolt hole or the chassis corner).
  3. The tube must be in a STABLE position during measurement (do not move it).

  Procedure (live)
  ----------------
  1. Hold the tube stationary (clamp if needed).

  2. For each attachment point you can physically access (minimum 3,
     recommend 5–8):
     a. Identify the attachment-point index k (count from the end you define
        as index 0; record the convention).
     b. Measure the centre position of the attachment hole in the chassis frame
        with calipers:  x_mm, y_mm, z_mm  (relative to chassis datum).
     c. Enter the lattice index and position when prompted.

  3. After collecting >= 3 points, type 'done'.
     The script runs the PCA line fit and prints:
     - Axis direction (unit vector in chassis frame)
     - Lattice origin (chassis-frame position of index 0)
     - Estimated pitch (compare to ATTACHMENT_PITCH_MM in constants.py)
     - RMS deviation of measured points from the fitted axis

  4. Paste the result into hardware.yaml under calibration.tube_axis.

  Sanity check
  ------------
  The pitch estimate should match ATTACHMENT_PITCH_MM ± ~1 mm.
  A large deviation indicates a measurement error or wrong index assignment.

================================================================================
[DRY-RUN complete — no hardware commands issued]
================================================================================
"""


def _run_dry() -> None:
    print(_DRY_RUN_TEXT)


def _run_live() -> None:
    """Collect attachment-point measurements and fit the tube axis."""
    import numpy as np
    from inchworm.calibration import fit_tube_axis
    from inchworm.constants import ATTACHMENT_PITCH_MM

    centers: list[np.ndarray] = []
    indices: list[int] = []

    print()
    print("=== Live tube-axis calibration ===")
    print(f"Reference: ATTACHMENT_PITCH_MM = {ATTACHMENT_PITCH_MM:.3f} mm")
    print("Collect >= 3 attachment-point centre positions.")
    print("Type 'done' when finished, 'abort' to quit.")
    print()

    i = 0
    while True:
        cmd = input(f"\n--- Point {i + 1} --- Press Enter to enter data (or 'done'/'abort'): ").strip().lower()
        if cmd == "abort":
            sys.exit(0)
        if cmd == "done":
            if len(centers) < 3:
                print(f"Need >= 3 points; only have {len(centers)}. Continue.")
                continue
            break

        raw_idx = input("  Enter lattice index k (integer): ").strip()
        try:
            k = int(raw_idx)
        except ValueError:
            print("  Invalid index. Skipping.")
            continue

        raw_pos = input("  Enter chassis position x_mm,y_mm,z_mm: ").strip()
        try:
            parts = [float(v.strip()) for v in raw_pos.split(",")]
            if len(parts) != 3:
                raise ValueError("need 3 values")
            pos = np.array(parts) / 1000.0  # mm → m
        except ValueError as e:
            print(f"  Invalid position ({e}). Skipping.")
            continue

        centers.append(pos)
        indices.append(k)
        print(f"  Recorded index={k}, pos={parts} mm. ({len(centers)} total)")
        i += 1

    pts = np.vstack(centers)
    idx = np.array(indices, dtype=np.int64)

    result = fit_tube_axis(pts, idx, notes="manual caliper measurement, live")

    pitch_mm = result.pitch_estimate_m * 1000.0
    pitch_diff_mm = abs(pitch_mm - ATTACHMENT_PITCH_MM)
    origin_mm = result.lattice_origin_in_chassis * 1000.0

    import datetime
    ts = datetime.datetime.now(datetime.timezone.utc).isoformat()

    print()
    print("=" * 70)
    print("  TUBE-AXIS CALIBRATION COMPLETE")
    print("=" * 70)
    print(f"  RMS deviation from axis: {result.residual_m * 1000:.2f} mm  (n={result.n_samples})")
    print(f"  Axis direction (chassis): {result.axis_direction.tolist()}")
    print(f"  Lattice origin (index 0): {origin_mm.tolist()} mm")
    print(f"  Estimated pitch: {pitch_mm:.2f} mm  (config: {ATTACHMENT_PITCH_MM:.3f} mm, diff: {pitch_diff_mm:.2f} mm)")
    if pitch_diff_mm > 2.0:
        print(f"  WARNING: pitch estimate differs from config by {pitch_diff_mm:.2f} mm — check indices/measurements.")
    print()
    print("  Paste into hardware.yaml under calibration.tube_axis:")
    print()
    print("  tube_axis:")
    print(f"    axis_direction: {result.axis_direction.tolist()}")
    print(f"    axis_point: {result.axis_point.tolist()}")
    print(f"    residual_m: {result.residual_m:.6f}")
    print(f"    lattice_origin_in_chassis: {result.lattice_origin_in_chassis.tolist()}")
    print(f"    pitch_estimate_m: {result.pitch_estimate_m:.6f}")
    print(f"    n_samples: {result.n_samples}")
    print(f"    ts_iso: '{ts}'")
    print(f"    method: '{result.method}'")
    print(f"    unverified: {'true' if pitch_diff_mm > 2.0 else 'false'}")
    print("=" * 70)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(
        prog="tube_axis.py",
        description=(
            "AA-16: Tube-axis calibration via PCA line fit.\n"
            "DRY-RUN by default. Pass --live for measurement (owner supervision)."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--live", action="store_true", default=False,
                   help="Collect real measurements (no motion commanded; owner supervision).")
    args = p.parse_args(argv)

    if not args.live:
        _run_dry()
        return 0

    logger.warning(
        "LIVE MODE: tube_axis.py will collect caliper measurements. "
        "Owner supervision required."
    )
    _run_live()
    return 0


if __name__ == "__main__":
    sys.exit(main())
