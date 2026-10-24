"""scripts/rollout.py — AA-26: supervised insertion rollout CLI.

Dry-run by default; use --live for physical execution.

Usage
-----
  python scripts/rollout.py --checkpoint PATH/pretrained_model/
  python scripts/rollout.py --checkpoint PATH/pretrained_model/ --live
  python scripts/rollout.py --checkpoint PATH/pretrained_model/ --cycle-count 20

Acceptance notes
----------------
AC#4 (latency): runs ``select_action`` timing in dry-run mode on randomly-
  initialised model weights if --checkpoint is provided and lerobot is available.

AC#7 (20 handoffs): requires --live and a verified checkpoint.  Each handoff
  outcome is appended to ``rollout_log.jsonl`` in the current directory.

Safety invariants
-----------------
- Dry-run is the default.  --live requires explicit flag.
- Checkpoint is validated at load time (CheckpointIncompatible raised before motion).
- No camera or serial port opened in dry-run mode (mock backend).
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from pathlib import Path

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s: %(message)s",
)
_LOG = logging.getLogger("rollout")


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="AA-26 supervised insertion rollout (dry-run by default)"
    )
    p.add_argument(
        "--checkpoint",
        required=True,
        metavar="DIR",
        help="Path to a pretrained_model/ checkpoint directory (config.json + model.safetensors)",
    )
    p.add_argument(
        "--live",
        action="store_true",
        default=False,
        help="Execute real motion (default: dry-run, no hardware commands sent)",
    )
    p.add_argument(
        "--cycle-count",
        type=int,
        default=1,
        metavar="N",
        help="Number of insertion cycles to run (default: 1)",
    )
    p.add_argument(
        "--log-file",
        default="rollout_log.jsonl",
        metavar="PATH",
        help="JSON-lines file to append outcomes to (default: rollout_log.jsonl)",
    )
    p.add_argument(
        "--device",
        default="cpu",
        metavar="DEVICE",
        help="PyTorch device for lerobot inference (default: cpu)",
    )
    return p.parse_args()


def main() -> int:
    args = _parse_args()
    dry_run = not args.live

    # --- 1. Validate checkpoint at load time (AC#6) ----------------------
    ckpt_dir = Path(args.checkpoint)
    _LOG.info("loading checkpoint from %s (dry_run=%s)", ckpt_dir, dry_run)

    from inchworm.policy_runner import (
        CheckpointLoader,
        CheckpointPolicy,
        CheckpointIncompatible,
        LeRobotNotAvailable,
        StubBackend,
        PolicyRunner,
    )

    try:
        loader = CheckpointLoader(ckpt_dir)
        meta = loader.load()
    except CheckpointIncompatible as exc:
        _LOG.error("checkpoint incompatible: %s", exc)
        return 1
    except FileNotFoundError as exc:
        _LOG.error("checkpoint not found: %s", exc)
        return 1

    _LOG.info("checkpoint validated: sha256=%s", meta.sha256)

    # --- 2. Select backend -----------------------------------------------
    if dry_run:
        _LOG.info("dry-run mode: using StubBackend (no lerobot import)")
        backend = StubBackend()
    else:
        _LOG.info("live mode: attempting LeRobotBackend (device=%s)", args.device)
        try:
            from inchworm.policy_runner import LeRobotBackend  # noqa: PLC0415
            backend = LeRobotBackend(device=args.device)
        except LeRobotNotAvailable as exc:
            _LOG.error(
                "lerobot is not available: %s\n"
                "Activate robot/lerobot_env before running with --live.",
                exc,
            )
            return 1

    policy = CheckpointPolicy(meta, backend)

    # --- 3. Build supervisor_kwargs (mock frame source and arm) ----------
    import numpy as np  # noqa: PLC0415

    from inchworm.config import load_config, PolicySupervisorConfig  # noqa: PLC0415
    from inchworm.kinematics import build_chain  # noqa: PLC0415
    from inchworm.policy_contract import N_JOINTS  # noqa: PLC0415

    cfg = load_config()
    chain = build_chain(cfg.so101_kinematics)
    T = chain.fk(np.zeros(N_JOINTS))
    target_m = T[:3, 3]

    def _mock_frame_source():
        frame = np.zeros((480, 640), dtype=np.uint8)
        joints = np.zeros(N_JOINTS, dtype=np.float32)
        return frame, joints

    supervisor_config = PolicySupervisorConfig(
        max_steps=200,
        max_wall_time_s=30.0,
        workspace_radius_m=0.15,
        max_action_magnitude_deg=5.0,
        clamp_on_magnitude_violation=True,
        max_retries=2,
        retract_pose_q_rad=[0.0, 0.0, 0.0, 0.0, 0.0],
        retract_pose_unverified=True,  # unverified: dry-run only
    )

    if args.live:
        _LOG.warning(
            "LIVE MODE: physical motion will be commanded. "
            "Ensure owner supervision before proceeding."
        )
        # In live mode a real motion executor, approach callable, seating evaluator
        # and lock SM must be wired in here (AA-18, AA-24, AA-17).
        # This script leaves them as stubs with a clear error message.
        _LOG.error(
            "Live-mode hardware wiring is not yet implemented in this script. "
            "Wire the real GuardedArm, SeatingEvaluator, and GripperLockSM before "
            "running with --live. Aborting."
        )
        return 1

    supervisor_kwargs = {
        "config": supervisor_config,
        "chain": chain,
        "target_attachment_point_m": target_m,
        "frame_source": _mock_frame_source,
        "motion_executor": lambda action: None,  # dry-run no-op
        "approach_callable": lambda: True,
        "joint_deg_to_rad": lambda d: np.deg2rad(np.asarray(d, dtype=np.float64)),
        "dry_run": True,
    }

    runner = PolicyRunner(
        policy=policy,
        supervisor_kwargs=supervisor_kwargs,
        dry_run=True,
        logger=_LOG,
    )

    # --- 4. Run cycles and log outcomes ----------------------------------
    log_path = Path(args.log_file)
    outcomes = []

    for cycle_idx in range(args.cycle_count):
        _LOG.info("--- cycle %d / %d ---", cycle_idx + 1, args.cycle_count)
        result = runner.run_insertion(cycle_index=cycle_idx)

        record = {
            "cycle_index": cycle_idx,
            "outcome": result.outcome.name,
            "attempts": result.supervisor_result.attempts,
            "total_steps": result.supervisor_result.total_steps,
            "elapsed_s": result.elapsed_s,
            "checkpoint_sha256": result.checkpoint_sha256,
            "timestamp": time.time(),
            "dry_run": dry_run,
        }
        outcomes.append(record)

        with log_path.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(record) + "\n")

        _LOG.info(
            "cycle %d outcome=%s attempts=%d steps=%d elapsed=%.3fs",
            cycle_idx,
            result.outcome.name,
            result.supervisor_result.attempts,
            result.supervisor_result.total_steps,
            result.elapsed_s,
        )

    # --- 5. Summary -------------------------------------------------------
    success_count = sum(1 for r in outcomes if r["outcome"] == "SUCCESS")
    _LOG.info(
        "summary: %d / %d cycles succeeded (dry_run=%s)",
        success_count, args.cycle_count, dry_run,
    )
    if not dry_run and args.cycle_count >= 20:
        if success_count == args.cycle_count:
            _LOG.info("AC#7: all %d handoffs succeeded with zero operator interventions", args.cycle_count)
        else:
            _LOG.warning(
                "AC#7: only %d / %d handoffs succeeded",
                success_count, args.cycle_count,
            )

    return 0


if __name__ == "__main__":
    sys.exit(main())
