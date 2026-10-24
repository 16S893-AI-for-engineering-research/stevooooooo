"""Units convention for the robot interface layer.

Every value that crosses a module or driver boundary is in millimeters or
degrees. The aliases below make that explicit in type annotations, and every
config field carrying a dimension also ends in its unit (``_mm``, ``_deg``,
``_mm_per_px``). Any internal use of meters or radians must be converted back
before it leaves the function that uses it, and documented there.
"""

from __future__ import annotations

from typing import Annotated

Millimeters = Annotated[float, "mm"]
Degrees = Annotated[float, "deg"]
MillimetersPerPixel = Annotated[float, "mm/px"]

# Field-name suffixes that declare a unit; enforced on the config schema by tests.
UNIT_SUFFIXES = ("_mm", "_deg", "_mm_per_px")

# --- AA-29 ---
# Add seconds suffix so max_continuous_on_s passes the unit-suffix check.
UNIT_SUFFIXES = UNIT_SUFFIXES + ("_s",)  # type: ignore[assignment]
# --- AA-29 end ---
