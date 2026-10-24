"""Tests for inchworm.constants: values come from config, defined once in source."""

from __future__ import annotations

import re
from pathlib import Path

from inchworm import constants
from inchworm.config import load_config

SRC_DIR = Path(constants.__file__).resolve().parent
CONFIG_FILE = SRC_DIR.parents[1] / "config" / "hardware.yaml"


def test_constants_match_doc1_design_values():
    assert constants.ATTACHMENT_PITCH_MM == 183.676
    assert constants.LASER_TO_CAMERA_ANGLE_DEG == 40.0
    assert constants.LASER_BASELINE_OFFSET_MM == 67.0
    assert constants.LASER_STANDOFF_MM == 80.0
    assert constants.LASER_HEIGHT_SENSITIVITY_MM_PER_PX == 0.1
    assert constants.MAX_CONCURRENT_CAMERA_STREAMS == 1


def test_constants_are_sourced_from_config():
    config = load_config()
    assert constants.ATTACHMENT_PITCH_MM == config.geometry.attachment_pitch_mm
    assert constants.LASER_STANDOFF_MM == config.laser.design.standoff_mm
    assert constants.MAX_CONCURRENT_CAMERA_STREAMS == config.usb.max_concurrent_camera_streams


# Literal forms of the doc-1 values that must not be restated in source code.
# 0.1 and 1 are too generic to scan for; they are pinned by name above instead.
_DESIGN_LITERALS = {
    "attachment pitch": r"183\.676",
    "laser angle": r"(?<![\w.])40(?:\.0*)?(?![\w.])",
    "baseline offset": r"(?<![\w.])67(?:\.0*)?(?![\w.])",
    "standoff": r"(?<![\w.])80(?:\.0*)?(?![\w.])",
}


def _code_lines(path: Path):
    """Source lines with trailing ``#`` comments stripped."""
    for lineno, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        code = line.split("#", 1)[0]
        if code.strip():
            yield lineno, code


def test_no_duplicated_design_literals_in_src():
    offenders = []
    for path in sorted(SRC_DIR.rglob("*.py")):
        for lineno, code in _code_lines(path):
            for label, pattern in _DESIGN_LITERALS.items():
                if re.search(pattern, code):
                    offenders.append(f"{path.name}:{lineno} {label}: {code.strip()}")
    assert not offenders, "design values restated outside config:\n" + "\n".join(offenders)


def test_design_values_defined_once_in_config_file():
    text = CONFIG_FILE.read_text(encoding="utf-8")
    assert len(re.findall(r"183\.676", text)) == 1
