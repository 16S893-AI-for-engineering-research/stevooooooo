"""AA-25 offline evaluation: held-out action error of an ACT checkpoint (no hardware).

For every frame t of each requested episode the policy predicts its action chunk from the
recorded observation (wrist image + joint state, exactly as training saw them). Reported
per joint, in raw servo degrees:

- ``step1``: |chunk[0] - action[t]|  (the action that would be sent next)
- ``exec``:  |chunk[k] - action[t+k]| for k < n_action_steps (the part that is executed
  before the policy re-plans), padded frames past the episode end excluded
- baseline ``hold``: |state[t] - action[t+k]| over the same window (= "stay where you
  are"); a policy that does not beat it has learned nothing useful about the motion
- ``final``: |chunk[0] - action[last]| on the last 10 % of the episode (end-pose accuracy)

Usage (from robot/lerobot_env)::

  uv run python eval_offline.py --checkpoint ../out/train/X/checkpoints/020000/pretrained_model \
      --dataset ../datasets/X --episodes 26,27,28 [--json out.json]
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import torch

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")


def _load(ckpt: Path, device: str):
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.policies.factory import make_pre_post_processors

    pcfg = PreTrainedConfig.from_pretrained(str(ckpt))
    pcfg.device = device
    policy = ACTPolicy.from_pretrained(str(ckpt), config=pcfg).to(device).eval()
    pre, post = make_pre_post_processors(
        pcfg, pretrained_path=str(ckpt),
        preprocessor_overrides={"device_processor": {"device": device}})
    return policy, pre, post, pcfg


def evaluate(ckpt: Path, dataset: Path, episodes, device: str = "cuda", batch: int = 16):
    from lerobot.datasets import LeRobotDataset

    policy, pre, post, pcfg = _load(ckpt, device)
    n_exec = int(pcfg.n_action_steps)
    fps = 30
    out = {"checkpoint": str(ckpt), "dataset": str(dataset), "n_action_steps": n_exec, "episodes": {}}
    for ep in episodes:
        ds = LeRobotDataset(f"local/{dataset.name}", root=dataset, episodes=[ep], video_backend="pyav",
                            delta_timestamps={"action": [k / fps for k in range(n_exec)]})
        n = len(ds)
        last_action = ds[n - 1]["action"][0].numpy()
        step1, exec_err, hold_err, final = [], [], [], []
        t0 = time.time()
        for s in range(0, n, batch):
            items = [ds[i] for i in range(s, min(n, s + batch))]
            obs = {"observation.state": torch.stack([x["observation.state"] for x in items]),
                   "observation.images.wrist": torch.stack([x["observation.images.wrist"] for x in items])}
            with torch.inference_mode():
                norm = pre(obs)
                chunk = policy.predict_action_chunk(norm)[:, :n_exec]          # (B, n_exec, 5) normalised
                pred = post(chunk).detach().cpu().numpy()                      # unnormalised degrees
            for b, x in enumerate(items):
                rec = x["action"].numpy()                                      # (n_exec, 5)
                pad = x["action_is_pad"].numpy()
                state = x["observation.state"].numpy()
                valid = ~pad
                step1.append(np.abs(pred[b, 0] - rec[0]))
                exec_err.append(np.abs(pred[b][valid] - rec[valid]))
                hold_err.append(np.abs(state[None, :] - rec[valid]))
                if s + b >= int(0.9 * n):
                    final.append(np.abs(pred[b, 0] - last_action))
        step1 = np.stack(step1)
        exec_err = np.concatenate(exec_err)
        hold_err = np.concatenate(hold_err)
        final = np.stack(final)

        def stats(a):
            return {"mean": [round(float(v), 3) for v in a.mean(0)],
                    "p95": [round(float(v), 3) for v in np.percentile(a, 95, axis=0)],
                    "max": [round(float(v), 3) for v in a.max(0)],
                    "mean_all": round(float(a.mean()), 3)}

        out["episodes"][ep] = {"frames": n, "seconds": round(n / fps, 1),
                               "step1": stats(step1), "exec": stats(exec_err),
                               "hold_baseline": stats(hold_err), "final": stats(final),
                               "eval_s": round(time.time() - t0, 1)}
    allx = list(out["episodes"].values())
    w = np.array([e["frames"] for e in allx], dtype=float)
    out["overall"] = {k: round(float(np.average([e[k]["mean_all"] for e in allx], weights=w)), 3)
                      for k in ("step1", "exec", "hold_baseline", "final")}
    return out


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--checkpoint", required=True, type=Path)
    p.add_argument("--dataset", required=True, type=Path)
    p.add_argument("--episodes", required=True, help="comma-separated episode indices")
    p.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    p.add_argument("--batch", type=int, default=16)
    p.add_argument("--json", type=Path, default=None)
    a = p.parse_args()
    res = evaluate(a.checkpoint, a.dataset, [int(e) for e in a.episodes.split(",")], a.device, a.batch)
    print(f"checkpoint {a.checkpoint}")
    print(f"joints {', '.join(JOINTS)} (raw servo deg); exec = first {res['n_action_steps']} chunk steps")
    for ep, e in res["episodes"].items():
        print(f"ep {ep}: {e['frames']} frames ({e['seconds']} s)")
        for k in ("step1", "exec", "hold_baseline", "final"):
            print(f"  {k:13s} mean {e[k]['mean']}  p95 {e[k]['p95']}  max {e[k]['max']}  all {e[k]['mean_all']}")
    print("overall mean |err| deg:", res["overall"])
    if a.json:
        a.json.parent.mkdir(parents=True, exist_ok=True)
        a.json.write_text(json.dumps(res, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
