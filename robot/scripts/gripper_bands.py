#!/usr/bin/env python3
"""Gripper band measurement guide (AA-17, AC#1).

Assists the owner in empirically measuring the servo position bands for the
custom SO-101 gripper.  These bands distinguish the three operating regions:

  - **unlocked**: jaw open, latch fully retracted.
  - **engaged_but_unlocked**: jaw contacting attachment point, latch not seated.
  - **locked**: latch positively seated and holding.

Default behaviour (no flags): **dry-run mode**.  No serial port is opened; no
hardware is touched.  The script prints measurement guidance and exits.

Pass ``--live`` to perform a guided position sweep on real hardware, with the
owner supervising each step.  The script reads ``present_position_deg`` only;
it never writes to the servo during measurement (read-only sweep guidance).

The owner must then copy the observed band boundaries into
``robot/config/hardware.yaml`` under the ``gripper_lock`` key, set
``unverified: false`` for each arm, and run the test suite to confirm the
new config validates.

Usage::

    # Dry-run (default) – print guidance, exit
    uv run python scripts/gripper_bands.py

    # Live (owner-supervised) – poll position while owner moves servo by hand
    uv run python scripts/gripper_bands.py --live --arm arm_left

Safety notes
------------
- This script is **read-only** in measurement mode: it never commands the
  servo to any position.
- The owner moves the gripper servo to each band region manually (torque
  disabled or back-driven with care) and observes the readback.
- Never run ``--live`` without the owner present.
- After measurement, both arms must still pass the existing test suite before
  live use.

HARDWARE-ONLY: this script requires real SO-101 hardware.  AC#1 cannot be
checked via this script alone; the owner must record observations and update
hardware.yaml in a supervised hardware session.
"""

from __future__ import annotations

import argparse
import sys
import time

# ---------------------------------------------------------------------------
# Guidance text (always printed)
# ---------------------------------------------------------------------------

_GUIDANCE = """
=============================================================================
AA-17 Gripper Band Measurement Guide
=============================================================================

PURPOSE
-------
Measure the STS3215 present_position_deg values corresponding to the three
operating bands of the custom gripper servo:

  1. UNLOCKED band        : jaw open, latch fully retracted.
  2. ENGAGED-BUT-UNLOCKED : jaw touching attachment point, latch not seated.
  3. LOCKED band          : latch positively seated and holding.

EQUIPMENT NEEDED
----------------
- Robot powered on, both SO-101 arms connected via USB.
- INCHWORM_BACKEND=hardware environment variable set.
- Owner present to back-drive or command the gripper servo by hand.

PROCEDURE
---------
Step 1: Disable torque on the gripper servo (or use a minimal torque).
        This allows back-driving the jaw by hand.

Step 2: Move the gripper to the FULLY OPEN (unlocked) position.
        Note the present_position_deg readback from the console.
        Record: unlocked_low_deg ≈ <observed_low>
                unlocked_high_deg ≈ <observed_high>

Step 3: Move the gripper until the jaw just contacts the tube attachment
        (engaged-but-unlocked: jaw touching, latch not yet clicking).
        Note the present_position_deg.
        Record: engaged_low_deg ≈ <observed_low>
                engaged_high_deg ≈ <observed_high>

Step 4: Continue closing until the latch positively seats (click).
        Note the present_position_deg.
        Record: locked_low_deg ≈ <observed_low>
                locked_high_deg ≈ <observed_high>

Step 5: Also note the load_raw and current_raw readings at each stage.
        These set the verification thresholds:
          jam_load_threshold_raw  : load when the servo jams (latch missed)
          latch_load_max_raw      : maximum load for a clean latch
          latch_current_max_raw   : maximum current for a clean latch

Step 6: Update robot/config/hardware.yaml under the gripper_lock section.
        Set unverified: false for each arm once values are confirmed.

Step 7: Run the test suite: cd robot && uv run pytest

YAML TEMPLATE (copy and fill in measured values)
-------------------------------------------------
gripper_lock:
  arm_left:
    unlocked_low_deg: <MEASURED>
    unlocked_high_deg: <MEASURED>
    engaged_low_deg: <MEASURED>
    engaged_high_deg: <MEASURED>
    locked_low_deg: <MEASURED>
    locked_high_deg: <MEASURED>
    unverified: false   # <-- set false only after measurement confirmed
  arm_right:
    unlocked_low_deg: <MEASURED>
    unlocked_high_deg: <MEASURED>
    engaged_low_deg: <MEASURED>
    engaged_high_deg: <MEASURED>
    locked_low_deg: <MEASURED>
    locked_high_deg: <MEASURED>
    unverified: false
  drift_threshold_deg: <MEASURED>  # e.g. 2.0
  verification:
    jam_load_threshold_raw: <MEASURED>
    latch_load_max_raw: <MEASURED>
    latch_current_max_raw: <MEASURED>
    unverified: false

=============================================================================
"""

_LIVE_WARNING = """
WARNING: --live mode opens a real serial port and polls servo position.
  - No write commands are sent; this is READ-ONLY.
  - The owner must be present and supervising.
  - Press Ctrl-C to stop polling at any time.
"""


# ---------------------------------------------------------------------------
# Live measurement loop (read-only position poll)
# ---------------------------------------------------------------------------


def _run_live(arm_role: str, poll_hz: float = 5.0) -> None:
    """Poll gripper present_position and print it until Ctrl-C.

    This is a read-only measurement aid.  No position commands are sent.

    Parameters
    ----------
    arm_role:
        ``"arm_left"`` or ``"arm_right"``.
    poll_hz:
        Polling rate in Hz (default 5 Hz = 0.2 s per sample).
    """
    print(_LIVE_WARNING)
    print(f"Opening hardware backend for role: {arm_role}")

    try:
        from inchworm.backends import HardwareBackend
        from inchworm.config import load_config
        from inchworm.servo_bus import ArmServoBus, ServoError
    except ImportError as exc:
        print(f"ERROR: cannot import inchworm modules: {exc}", file=sys.stderr)
        sys.exit(1)

    try:
        config = load_config()
    except Exception as exc:  # noqa: BLE001
        print(f"ERROR: cannot load config: {exc}", file=sys.stderr)
        sys.exit(1)

    # Build joint ID map from config (may be all-null until scanned)
    joints_cfg = (
        config.arms.left.servo_joints
        if arm_role == "arm_left"
        else config.arms.right.servo_joints
    )
    joint_id_map = {
        k: v
        for k, v in {
            "shoulder_pan": joints_cfg.shoulder_pan,
            "shoulder_lift": joints_cfg.shoulder_lift,
            "elbow_flex": joints_cfg.elbow_flex,
            "wrist_flex": joints_cfg.wrist_flex,
            "wrist_roll": joints_cfg.wrist_roll,
            "gripper": joints_cfg.gripper,
        }.items()
        if v is not None
    }

    if "gripper" not in joint_id_map:
        print(
            "ERROR: gripper servo ID is null in hardware.yaml. "
            "Run scan_servos.py --live first to populate servo IDs.",
            file=sys.stderr,
        )
        sys.exit(1)

    try:
        backend = HardwareBackend(config)
        transport = backend.open_serial(arm_role)
    except Exception as exc:  # noqa: BLE001
        print(f"ERROR: cannot open serial for {arm_role}: {exc}", file=sys.stderr)
        sys.exit(1)

    bus = ArmServoBus(
        transport,
        role=arm_role,
        joint_id_map=joint_id_map,
        dry_run=False,  # read-only, but not dry-run (we need real reads)
    )

    gripper_servo_id: int = joint_id_map["gripper"]

    print(f"Polling gripper servo ID {gripper_servo_id} at {poll_hz} Hz.")
    print("Move the gripper by hand and observe the readbacks.")
    print("Press Ctrl-C to stop.\n")
    print(f"{'TIME':>8}  {'POS_DEG':>9}  {'LOAD_RAW':>10}  {'CURRENT_RAW':>12}")
    print("-" * 46)

    interval = 1.0 / poll_hz
    t0 = time.monotonic()
    try:
        while True:
            try:
                js = bus.read_joint_state(gripper_servo_id)
                elapsed = time.monotonic() - t0
                print(
                    f"{elapsed:8.2f}  "
                    f"{js.position_deg:9.3f}  "
                    f"{js.load_raw:10d}  "
                    f"{js.current_raw:12d}"
                )
            except ServoError as exc:
                print(f"  [servo error: {exc}]")
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\nStopped by user.")

    print("\nDone. Record the observed band boundaries in hardware.yaml.")
    print("See the guidance above for the YAML template.")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--live",
        action="store_true",
        default=False,
        help=(
            "Open a real serial port and poll the gripper servo position "
            "(read-only). OWNER SUPERVISION REQUIRED. "
            "Default (without --live) prints guidance and exits."
        ),
    )
    parser.add_argument(
        "--arm",
        choices=["arm_left", "arm_right"],
        default="arm_left",
        help="Arm role to measure (default: arm_left). Only used with --live.",
    )
    parser.add_argument(
        "--poll-hz",
        type=float,
        default=5.0,
        metavar="HZ",
        help="Polling rate in Hz (default 5 Hz). Only used with --live.",
    )
    args = parser.parse_args(argv)

    print(_GUIDANCE)

    if not args.live:
        print("(Dry-run mode: no hardware touched.  Pass --live for real measurements.)")
        return 0

    _run_live(arm_role=args.arm, poll_hz=args.poll_hz)
    return 0


if __name__ == "__main__":
    sys.exit(main())
