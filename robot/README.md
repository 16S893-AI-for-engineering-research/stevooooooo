# inchworm-robot

Control software for the inchworm print-crawl robot: two SO-101 arms and a
Rotrics DexArm on a fixed chassis, moving a printed tube. Hardware facts live in
backlog doc-1; this package must not restate them in code.

## Quick start

```bash
cd robot
uv run pytest            # all tests run against the mock backend, no hardware
```

## Layout

| Path | What it holds |
|---|---|
| `config/hardware.yaml` | Declarative hardware description: device identities, geometry, laser design values, USB constraint. Data, not code. Calibration tasks write results back here. |
| `src/inchworm/config.py` | Typed, versioned loader (`load_config`). Rejects unknown fields, missing fields and wrong types with a `ConfigError` naming the field path. |
| `src/inchworm/constants.py` | The system constants (attachment pitch, laser design values, single-camera limit), read once from the config. Import them from here; never restate the literals. |
| `src/inchworm/backends.py` | Driver backend abstraction. `create_backend(config)` returns a `MockBackend` or `HardwareBackend`. |
| `src/inchworm/units.py` | Unit type aliases (`Millimeters`, `Degrees`, `MillimetersPerPixel`). |

## Units convention

**Millimeters and degrees at every interface boundary**: function arguments,
return values, config fields, logged values and anything crossing a module or
driver boundary.

Enforced two ways:

1. **Naming.** Every dimensional config field ends in its unit: `_mm`, `_deg`,
   `_mm_per_px`. `tests/test_config.py::test_dimensional_fields_carry_unit_suffix`
   fails if a float field in the schema lacks one.
2. **Type annotations.** Use `inchworm.units.Millimeters` / `Degrees` /
   `MillimetersPerPixel` on interface signatures and constants.

A module that needs meters or radians internally (for example a kinematics
solver) converts at its own boundary and documents the internal unit where it
is used. Radians or meters must never leak out of a public function.

## Backend selection

The mock backend serves in-memory serial and camera transports, so arm, DexArm,
relay and camera logic runs with zero hardware attached.

- Config: `backend: mock` or `backend: hardware` in `config/hardware.yaml`.
- Override: `INCHWORM_BACKEND=mock|hardware` takes precedence over the config.
- Config file override: `INCHWORM_CONFIG=/path/to/hardware.yaml`.

`HardwareBackend` currently refuses to open any device. Real transports arrive
with device discovery (AA-5) and the per-device drivers (AA-8, AA-12, AA-14,
AA-29). Camera transports must go through the AA-6 single-camera arbiter.

## Device identity

Devices are keyed on `vendor_id:product_id` plus `usb_serial` wherever the
device reports one. The relay (CH340) and the right-arm camera report no USB
serial, so they are marked `identify_by: port_path` and must be resolved by
where they are plugged in. `windows_busid_observed` is informational only:
usbipd BusIds change across reconnects.

<!-- --- AA-22 --- -->
## AA-22: LeRobot environment and SO-101 adapter

### Environment setup (lerobot_env)

LeRobot 0.6.1 requires Python >= 3.12 which is incompatible with the main
package (`requires-python >= 3.10`). A **separate uv project** lives in
`robot/lerobot_env/`.

**Recreate from scratch:**

```bash
cd robot/lerobot_env
uv venv --python 3.12 --seed    # create .venv with Python 3.12
uv sync                         # install lerobot 0.6.1 + inchworm (editable)
uv run python smoke.py          # verify versions and CUDA
uv run pytest tests/ -v         # run LeRobot integration tests
uv run python vram_measure_act.py   # measure VRAM for AC#6 (real ACTPolicy)
```

The `uv.lock` is committed alongside `pyproject.toml`; `uv sync` reproduces
the exact pinned environment. The `.venv/` directory is gitignored.

**Pinned versions (as of 2026-10-24):**
- Python: 3.12.15
- lerobot: 0.6.1 (newest 0.6.x; 0.6.2 does not exist on PyPI)
- PyTorch: 2.11.0+cu130
- CUDA: 13.0 (RTX 3050 Ti Laptop, sm_86)

### Adapter (`inchworm/lerobot_adapter.py`)

The adapter lives in the **main package** and is importable without lerobot
installed (all `lerobot.*` imports are guarded).

| Feature | Detail |
|---|---|
| Observation | `shoulder_pan.pos` … `wrist_roll.pos` (5 floats) + `wrist_cam` `(H, W, 1)` uint8 mono |
| Action | Same 5 joint keys only; gripper **structurally absent** (AC#4) |
| Camera path | `frame_source` callable injected by caller (must hold AA-6 arbiter lock) (AC#3) |
| Camera device | Resolved via AA-5 `require_device` (usbipd BusId); never OpenCV index (AC#9) |
| Joint writes | Via `GuardedArm.apply_joint_command` (AA-9 enforcement) |
| Contract check | `N_JOINTS==5` and `JOINT_ORDER` validated at construction time (AC#5) |

**Usage example (lerobot_env, with real hardware):**

```python
from inchworm.config import load_config
from inchworm.backends import create_backend
from inchworm.camera_arbiter import CameraArbiter
from inchworm.camera_capture import open_capture
from inchworm.discovery import require_device
from inchworm.lerobot_adapter import _make_adapter_class, _make_config_class

config = load_config()
backend = create_backend(config)
arbiter = CameraArbiter(backend)

# AC#9: camera device resolved via AA-5 (usbipd BusId), never OpenCV index
cam_role = "camera_left_arm"  # usbipd BusId resolved internally

with arbiter.acquire(cam_role) as transport:
    capture = open_capture(transport, role=cam_role)
    capture.apply_settings(width=640, height=480, fps=30)
    frame_source = lambda: capture.grab().data  # (H, W) uint8 mono

    Config = _make_config_class()
    cfg = Config(arm_role="arm_left", frame_height=480, frame_width=640)
    Adapter = _make_adapter_class()
    robot = Adapter(
        cfg,
        guarded_arm=guarded_arm,   # AA-9 GuardedArm
        frame_source=frame_source, # AC#3: through arbiter
        camera_wh=(640, 480),      # AC#9: from AA-5 discovery
    )
    with robot:
        obs = robot.get_observation()
        # obs keys: shoulder_pan.pos … wrist_roll.pos, wrist_cam (H,W,1)
```

### VRAM feasibility (AC#6)

Measured by the orchestrator, 2026-10-24, with the **real LeRobot `ACTPolicy`**
(lerobot 0.6.1, default config: ResNet-18 ImageNet backbone, transformer,
chunk_size 100) for one full training step (forward + backward + AdamW step),
on synthetic 3-channel images (mono replicated to 3 channels for the
pretrained backbone), 5-D state and 5-D action. The earlier toy-CNN estimate was
discarded because it did not measure ACT.

```
lerobot 0.6.1  torch 2.11.0+cu130  GPU NVIDIA GeForce RTX 3050 Ti Laptop GPU  total 4096 MB
       res  bs   amp  peak_MB  step_s
240x 320   1 False     1005    0.68
240x 320   2 False     1010    0.21
240x 320   4 False     1018    0.19
240x 320   8 False     1021    0.16
240x 320   1  True     1007    0.38
240x 320   2  True     1010    0.20
240x 320   4  True     1011    0.19
240x 320   8  True     1019    0.19
480x 640   1 False     1017    0.14
480x 640   2 False     1013    0.14
480x 640   4 False     1025    0.21
480x 640   8 False     1724    0.36
480x 640   1  True     1014    0.18
480x 640   2  True     1016    0.20
480x 640   4  True     1025    0.19
480x 640   8  True     1677    0.26
```

Peak use is about 1.0 GB at batch 1-4 and about 1.7 GB at 480x640, batch 8, so
**ACT training fits the 4 GB RTX 3050 Ti** at 480x640 with batch 8, leaving
headroom for the CUDA context and data loading. Recommendation: 480x640 (or
downscale to 240x320 if the dataloader is CPU-bound), batch 8, AMP optional.
Re-measure: `cd robot/lerobot_env && uv run python vram_measure_act.py`.
<!-- --- AA-22 end --- -->
