"""
inchworm.system_model
====================
Full-system geometric model of the as-built inchworm robot, parsed from
``CAD/Assembly 1.step``.

Sub-modules
-----------
step_parser   – pure-Python streaming STEP AP242 placement extractor
arm_placer    – matches CAD Base_SO101 instances to the vendored URDF
model_builder – tessellates non-arm parts and assembles the full model
viewer        – meshcat browser viewer + offscreen PNG snapshots
"""
# Public re-exports used by tests without importing heavy deps
from .step_parser import StepPlacement, parse_step_placements  # noqa: F401
