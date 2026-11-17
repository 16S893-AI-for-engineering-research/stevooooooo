"""scripts/rollout.py — AA-26: supervised insertion rollout CLI.

Dry-run by default; use --live for physical execution.

Usage
-----
  python scripts/rollout.py --checkpoint PATH/pretrained_model/
  python scripts/rollout.py --checkpoint PATH/pretrained_model/ --live
  python scripts/rollout.py --checkpoint PATH/pretrained_model/ --cycle-count 20

Dataset-replay dry-run (robot/lerobot_env; real policy, no hardware opened)::

  cd robot/lerobot_env
  uv run python ../scripts/rollout.py --checkpoint OUT/checkpoints/last/pretrained_model \
      --dataset ../datasets/insert_left_<UTC> --episode 0 --device cuda

  The trained checkpoint runs through LeRobotBackend (LeRobot 0.6.1 API, the
  checkpoint's normalisation) inside the unchanged supervisor; observations
  (wrist frame + joints, raw servo degrees) are replayed from the episode. The
  workspace target is the episode's last (seated) pose. Each step's policy target
  is compared with the recorded action; nothing is commanded.

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
    p.add_argument(
        "--dataset",
        default=None,
        metavar="DIR",
        help="Dry-run only: replay observations from this recorded LeRobot dataset and run "
             "the real checkpoint (LeRobotBackend) instead of the stub",
    )
    p.add_argument("--episode", type=int, default=0, help="Dataset episode to replay (default: 0)")
    p.add_argument("--follower", default="left", choices=("left", "right"),
                   help="Arm the checkpoint drives (calibration for the workspace check)")
    p.add_argument("--max-steps", type=int, default=200, metavar="N",
                   help="Supervisor step limit per attempt (default: 200)")
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

    if args.dataset and not dry_run:
        _LOG.error("--dataset is a dry-run replay; it cannot be combined with --live")
        return 1

    # --- 2. Select backend -----------------------------------------------
    replay = None
    if dry_run and args.dataset:
        _LOG.info("dry-run dataset replay: LeRobotBackend on %s episode %d", args.dataset, args.episode)
        try:
            from inchworm.policy_runner import DatasetReplaySource, LeRobotBackend  # noqa: PLC0415
            backend = LeRobotBackend(device=args.device, follower=args.follower)
            replay = DatasetReplaySource(args.dataset, args.episode)
        except LeRobotNotAvailable as exc:
            _LOG.error("lerobot is not available: %s\nRun from robot/lerobot_env.", exc)
            return 1
    elif dry_run:
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

    try:
        policy = CheckpointPolicy(meta, backend)
    except CheckpointIncompatible as exc:
        _LOG.error("checkpoint incompatible: %s", exc)
        return 1

    # --- 3. Build supervisor_kwargs (mock frame source and arm) ----------
    import numpy as np  # noqa: PLC0415

    from inchworm.config import load_config, PolicySupervisorConfig  # noqa: PLC0415
    from inchworm.kinematics import build_chain  # noqa: PLC0415
    from inchworm.policy_contract import JOINT_ORDER, N_JOINTS  # noqa: PLC0415

    cfg = load_config()
    chain = build_chain(cfg.so101_kinematics)
    T = chain.fk(np.zeros(N_JOINTS))
    target_m = T[:3, 3]

    def _mock_frame_source():
        frame = np.zeros((480, 640), dtype=np.uint8)
        joints = np.zeros(N_JOINTS, dtype=np.float32)
        return frame, joints

    joint_deg_to_rad = lambda d: np.deg2rad(np.asarray(d, dtype=np.float64))  # noqa: E731
    frame_source = _mock_frame_source
    step_errors = []
    if replay is not None:
        from inchworm.policy_runner import raw_servo_deg_to_q_rad  # noqa: PLC0415
        joint_deg_to_rad = raw_servo_deg_to_q_rad(cfg, args.follower)
        _, last_state, _ = replay.item(replay.length - 1)
        target_m = chain.fk(joint_deg_to_rad(last_state))[:3, 3]
        frame_source = replay
        inner = policy.step

        def _compare(obs):
            action = inner(obs)
            rec = replay.last_action_deg
            step_errors.append(max(abs(action[j] - float(rec[i])) for i, j in enumerate(JOINT_ORDER)))
            return action

        policy.step = _compare

    supervisor_config = PolicySupervisorConfig(
        max_steps=args.max_steps,
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
        "frame_source": frame_source,
        "motion_executor": lambda action: None,  # dry-run no-op
        "approach_callable": lambda: True,
        "joint_deg_to_rad": joint_deg_to_rad,
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
        if replay is not None:
            record.update({
                "dataset": str(args.dataset), "episode": args.episode,
                "steps_compared": len(step_errors),
                "max_abs_err_vs_recorded_deg": round(max(step_errors), 3) if step_errors else None,
                "median_abs_err_vs_recorded_deg":
                    round(float(np.median(step_errors)), 3) if step_errors else None,
            })
            _LOG.info("replay: %d steps, |policy - recorded action| max %s deg, median %s deg",
                      len(step_errors), record["max_abs_err_vs_recorded_deg"],
                      record["median_abs_err_vs_recorded_deg"])
            step_errors.clear()
            replay.index = 0
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
