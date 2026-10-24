#!/usr/bin/env python3
"""AA-22 smoke test: report lerobot, torch, and Python versions + CUDA availability.

Run from the lerobot_env directory:
    uv run python smoke.py
"""
# --- AA-22 ---

import sys

print(f"Python:          {sys.version}")
print(f"Python version:  {sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}")

try:
    import torch

    print(f"PyTorch version: {torch.__version__}")
    cuda_available = torch.cuda.is_available()
    print(f"CUDA available:  {cuda_available}")
    if cuda_available:
        print(f"CUDA device:     {torch.cuda.get_device_name(0)}")
        print(f"CUDA capability: {torch.cuda.get_device_capability(0)}")
        print(f"CUDA version:    {torch.version.cuda}")
    else:
        print("CUDA device:     (not available)")
except ImportError as exc:
    print(f"PyTorch:         IMPORT ERROR: {exc}")

try:
    import lerobot

    print(f"LeRobot version: {lerobot.__version__}")
except ImportError as exc:
    print(f"LeRobot:         IMPORT ERROR: {exc}")

try:
    import inchworm

    print(f"inchworm:        importable (from {inchworm.__file__})")
except ImportError as exc:
    print(f"inchworm:        IMPORT ERROR: {exc}")

try:
    from inchworm.lerobot_adapter import (
        ADAPTER_JOINT_ORDER,
        ACTION_JOINT_KEYS,
        CAMERA_KEY,
        CAMERA_SHAPE,
    )

    print(f"lerobot_adapter: importable")
    print(f"  ADAPTER_JOINT_ORDER = {ADAPTER_JOINT_ORDER}")
    print(f"  CAMERA_KEY = {CAMERA_KEY!r}, CAMERA_SHAPE = {CAMERA_SHAPE}")
except Exception as exc:
    print(f"lerobot_adapter: ERROR: {exc}")

# --- AA-22 end ---
