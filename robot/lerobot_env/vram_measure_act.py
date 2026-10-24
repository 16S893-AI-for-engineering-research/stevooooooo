"""AA-22 AC#6: measure peak GPU memory of the REAL LeRobot ACTPolicy
(ResNet-18 backbone + transformer, default config) for one training step
(forward + backward + optimizer step) on synthetic mono-as-3ch images.

Orchestrator 2026-10-24: replaces the earlier toy-CNN estimate, which did not
measure ACT. Run: uv run python vram_measure_act.py
"""
import gc, sys, time
import torch
from lerobot.configs.types import FeatureType, PolicyFeature, NormalizationMode
from lerobot.policies.act.configuration_act import ACTConfig
from lerobot.policies.act.modeling_act import ACTPolicy


def run(h, w, bs, amp):
    cfg = ACTConfig(
        input_features={
            "observation.state": PolicyFeature(type=FeatureType.STATE, shape=(5,)),
            "observation.images.wrist": PolicyFeature(type=FeatureType.VISUAL, shape=(3, h, w)),
        },
        output_features={"action": PolicyFeature(type=FeatureType.ACTION, shape=(5,))},
        device="cuda",
    )
    pol = ACTPolicy(cfg).cuda().train()
    opt = torch.optim.AdamW(pol.parameters(), lr=1e-5)
    k = cfg.chunk_size
    batch = {
        "observation.state": torch.randn(bs, 5, device="cuda"),
        "observation.images.wrist": torch.rand(bs, 3, h, w, device="cuda"),
        "action": torch.randn(bs, k, 5, device="cuda"),
        "action_is_pad": torch.zeros(bs, k, dtype=torch.bool, device="cuda"),
    }
    torch.cuda.reset_peak_memory_stats(); torch.cuda.synchronize()
    t = time.perf_counter()
    with torch.autocast("cuda", dtype=torch.bfloat16, enabled=amp):
        loss, _ = pol.forward(batch)
    loss.backward(); opt.step(); opt.zero_grad(set_to_none=True)
    torch.cuda.synchronize()
    dt = time.perf_counter() - t
    peak = torch.cuda.max_memory_allocated() / 2**20
    del pol, opt, batch, loss; gc.collect(); torch.cuda.empty_cache()
    return peak, dt


if __name__ == "__main__":
    import lerobot
    print(f"lerobot {lerobot.__version__}  torch {torch.__version__}  GPU {torch.cuda.get_device_name(0)}  "
          f"total {torch.cuda.get_device_properties(0).total_memory/2**20:.0f} MB")
    print(f"{'res':>10} {'bs':>3} {'amp':>5} {'peak_MB':>8} {'step_s':>7}")
    for (h, w) in [(240, 320), (480, 640)]:
        for amp in (False, True):
            for bs in (1, 2, 4, 8):
                try:
                    peak, dt = run(h, w, bs, amp)
                    print(f"{h}x{w:>4} {bs:>3} {str(amp):>5} {peak:8.0f} {dt:7.2f}", flush=True)
                except torch.cuda.OutOfMemoryError:
                    print(f"{h}x{w:>4} {bs:>3} {str(amp):>5}      OOM", flush=True)
                    gc.collect(); torch.cuda.empty_cache()
                    break
