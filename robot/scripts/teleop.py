#!/usr/bin/env python3
"""AA-23: Cross-arm teleop demonstration capture script.

Dry-run is the default.  Real motion requires ``--live``.

Hardware-only ACs (needs owner-supervised hardware session):
  AC#1  Mapping verification:
        ``cd robot && uv run python scripts/teleop.py --live --speed-scale 0.2``
  AC#4  Fixture validation:
        ``cd robot && uv run python scripts/teleop.py --live --validate-fixture``
  AC#5  Backdrive effort and latency:
        ``cd robot && uv run python scripts/teleop.py --live --measure-latency``

Safety invariants (always hold):
- Joint 6 (gripper) is never commanded.  The gripper is reserved for AA-17.
- Every follower position write goes through GuardedArm (AA-9).
- Only the follower wrist camera streams during capture (AA-6 arbiter).
- Recording refuses to start unless tube_support_source + operator_token
  are set in config or via CLI flags.
- Dry-run is the default; live motion requires explicit ``--live``.
- Live mode fails closed on unverified mirror_map (unverified: true config values
  are used in dry-run only with a WARNING; live mode prints a warning but does NOT
  fail closed on the mapping itself — the owner verifies at low speed per AC#1).
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

from inchworm.config import load_config
from inchworm.teleop_capture import (
    TELEOP_JOINTS,
    MirrorMapper,
    TeleopSession,
    _LocalLeRobotWriter,
    make_writer,
    check_preconditions,
    PreconditionError,
)

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
log = logging.getLogger(__name__)


def _build_mock_session(cfg_teleop, dataset_root: Path, live: bool, speed_scale: float):
    """Build a TeleopSession backed by mock buses for dry-run."""
    from unittest.mock import MagicMock
    from inchworm.servo_bus import JointState

    # Mock leader bus: returns zero positions for all teleop joints
    leader_bus = MagicMock()
    states = {
        j: JointState(
            servo_id=i + 1,
            position_deg=180.0,
            speed_raw=0,
            load_raw=0,
            voltage_v=12.0,
            temperature_c=30,
        )
        for i, j in enumerate(TELEOP_JOINTS)
    }
    leader_bus.read_all_joint_states.return_value = states

    # Mock follower GuardedArm
    follower = MagicMock()
    follower_bus = MagicMock()
    follower_bus.read_all_joint_states.return_value = states
    follower.bus = follower_bus

    writer = make_writer(dataset_root)

    pre_insertion_q = [180.0] * len(TELEOP_JOINTS)

    session = TeleopSession(
        config=cfg_teleop,
        leader_bus=leader_bus,
        follower_guarded_arm=follower,
        camera_arbiter=None,
        follower_camera_role=f"camera_{cfg_teleop.follower_arm}_arm",
        writer=writer,
        pre_insertion_q_deg=pre_insertion_q,
        live=live,
    )
    return session


def main() -> int:
    parser = argparse.ArgumentParser(
        description="AA-23 teleop capture. Dry-run by default."
    )
    parser.add_argument(
        "--live",
        action="store_true",
        default=False,
        help="Enable real hardware motion (requires physical setup and owner supervision).",
    )
    parser.add_argument(
        "--speed-scale",
        type=float,
        default=0.2,
        dest="speed_scale",
        help="Fractional speed scale for follower motion (0 < value <= 1.0; default 0.2).",
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=1,
        help="Number of episodes to record (default 1).",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=50,
        help="Timesteps per episode (default 50).",
    )
    parser.add_argument(
        "--config",
        default=None,
        help="Path to hardware.yaml (default: robot/config/hardware.yaml).",
    )
    parser.add_argument(
        "--dataset-root",
        default=None,
        dest="dataset_root",
        help="Local dataset root path (overrides config).",
    )
    parser.add_argument(
        "--tube-support",
        default=None,
        dest="tube_support",
        help="Tube support source: 'fixture' or 'locked_arm'.",
    )
    parser.add_argument(
        "--operator-token",
        default=None,
        dest="operator_token",
        help="Operator confirmation token (required when --tube-support fixture).",
    )
    parser.add_argument(
        "--validate-fixture",
        action="store_true",
        default=False,
        dest="validate_fixture",
        help=(
            "HARDWARE ONLY (AC#4): compare wrist-camera frames and follower joint "
            "poses at the pre-insertion pose in the fixture vs the real crawl "
            "configuration. Records the measured difference. Requires --live."
        ),
    )
    parser.add_argument(
        "--measure-latency",
        action="store_true",
        default=False,
        dest="measure_latency",
        help=(
            "HARDWARE ONLY (AC#5): measure leader backdrive effort and teleop "
            "tracking latency; log a judgement of demo smoothness. Requires --live."
        ),
    )
    args = parser.parse_args()

    if args.validate_fixture and not args.live:
        parser.error("--validate-fixture requires --live")
    if args.measure_latency and not args.live:
        parser.error("--measure-latency requires --live")
    if args.speed_scale <= 0 or args.speed_scale > 1.0:
        parser.error("--speed-scale must be in (0, 1]")

    cfg = load_config(args.config)

    if cfg.teleop_capture is None:
        log.error("No teleop_capture block in hardware.yaml. Add it before recording.")
        return 1

    tc = cfg.teleop_capture

    # Apply CLI overrides
    import dataclasses
    overrides: dict = {}
    if args.tube_support:
        overrides["tube_support_source"] = args.tube_support
    if args.operator_token:
        overrides["operator_token"] = args.operator_token

    if overrides:
        # Re-build config with overrides using model_copy (pydantic v2)
        tc = tc.model_copy(update=overrides)

    from inchworm.config import resolve_robot_path
    dataset_root = resolve_robot_path(args.dataset_root if args.dataset_root else tc.dataset_root)

    if args.live:
        log.warning(
            "AA-23 LIVE MODE: real motion will be commanded. "
            "Ensure owner is supervising, fixture is in place, and speed_scale=%.2f.",
            args.speed_scale,
        )
        # Hardware path: build real buses and GuardedArm.
        # Not implemented here (requires AA-8/AA-9 integration and owner
        # supervision for first execution). This block is intentionally a stub:
        # the dry-run path below is the safe default.
        log.error(
            "Live hardware session not implemented in this script version. "
            "Implement robot/scripts/teleop.py live path after hardware is available.\n"
            "Hardware AC commands:\n"
            "  AC#1: cd robot && uv run python scripts/teleop.py --live --speed-scale 0.2\n"
            "  AC#4: cd robot && uv run python scripts/teleop.py --live --validate-fixture\n"
            "  AC#5: cd robot && uv run python scripts/teleop.py --live --measure-latency"
        )
        return 2
    else:
        log.info("AA-23 dry-run mode (no hardware, no real motion).")

    # Check preconditions (will fail if not configured)
    try:
        check_preconditions(
            tube_support_source=tc.tube_support_source,
            operator_token=tc.operator_token,
            leader_arm=tc.leader_arm,
        )
    except PreconditionError as exc:
        log.error("Precondition failed: %s", exc)
        log.info(
            "Hint: pass --tube-support fixture --operator-token <token> or "
            "set tube_support_source/operator_token in hardware.yaml."
        )
        return 1

    session = _build_mock_session(tc, dataset_root, live=False, speed_scale=args.speed_scale)
    session.start()

    for ep_i in range(args.episodes):
        meta = session.record_episode(n_steps=args.steps, speed_scale=args.speed_scale)
        log.info("Episode %d/%d: %s (%d frames, seed=%d)",
                 ep_i + 1, args.episodes, meta.label, meta.frame_count, meta.seed)

    info_path = session.close()
    log.info("Dataset written to %s", info_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
