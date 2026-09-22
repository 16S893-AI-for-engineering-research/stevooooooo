"""Free-form truss geometry and segmentation.

Sec. 3 and 5 of Jonckers et al. (2022). The truss is printed free-form -- the
material is "extruded and cooled in 3 dimensional space" rather than deposited
on previous layers -- so the geometry is constrained by the nozzle:

  * height <= 5.5 mm and diagonals shallower than 45 deg from horizontal,
    else the printhead recontacts already printed material (Sec. 3, Fig. 3);
  * each segment must fit the arm workspace: a 110 mm element "could
    comfortably fit within the workspace of the robotic arm" (Sec. 5.1);
  * segments overlap so they fuse into one structure (Sec. 5.2, Fig. 12);
  * 7 segments give a 775 mm truss (Sec. 5.2).

Note the internal consistency check this affords: 775 / 7 = 110.7 mm of new
structure per segment, against the 110 mm quoted for the single element.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from . import params as P
from .gcode import GCodeMove

__all__ = [
    "TrussSegment",
    "segment_path",
    "segment_moves",
    "multi_segment_truss",
    "check_freeform_limits",
]


@dataclass(frozen=True)
class TrussSegment:
    """One printable truss segment.

    Attributes:
        index: 0-based segment number.
        origin_x: x offset of this segment in the world frame, m.
        path: (N, 3) array of path vertices in the print frame, m.
    """

    index: int
    origin_x: float
    path: np.ndarray

    @property
    def length(self) -> float:
        """Total path length of the segment, m."""
        return float(np.sum(np.linalg.norm(np.diff(self.path, axis=0), axis=1)))

    @property
    def extent_x(self) -> float:
        """x extent of the segment, m."""
        return float(self.path[:, 0].max() - self.path[:, 0].min())


def segment_path(length: float = P.SEGMENT_LENGTH_MM / 1000.0,
                 depth: float = P.TRUSS_DEPTH_MM / 1000.0,
                 height: float = P.LAYER_HEIGHT_MM / 1000.0,
                 bays: int = P.TRUSS_BAYS_PER_SEGMENT,
                 origin_x: float = 0.0) -> np.ndarray:
    """Build the vertex path of one truss segment as a continuous print path.

    Sec. 5.2: "each of which were printed as a continuous print path", so this
    returns a single polyline the nozzle can trace without lifting -- a zigzag
    web between two chords, which is the classic Warren truss topology visible
    in Figs. 10-11.

    Args:
        length: x extent of the segment, m.
        depth: y separation of the two chords, m.
        height: z height of the printed layer, m.
        bays: number of zigzag bays.
        origin_x: x offset of the segment, m.

    Returns:
        (N, 3) array of vertices in metres.
    """
    if bays < 1:
        raise ValueError("a segment needs at least one bay")
    if length <= 0.0 or depth <= 0.0:
        raise ValueError("length and depth must be positive")

    pitch = length / bays
    vertices: list[np.ndarray] = []

    # Outbound: near chord with diagonals up to the far chord and back.
    for i in range(bays):
        x0 = origin_x + i * pitch
        vertices.append(np.array([x0, 0.0, height]))
        vertices.append(np.array([x0 + pitch / 2.0, depth, height]))
    vertices.append(np.array([origin_x + length, 0.0, height]))

    # Return pass along the far chord, closing the truss into one structure.
    vertices.append(np.array([origin_x + length, depth, height]))
    vertices.append(np.array([origin_x, depth, height]))

    return np.array(vertices, dtype=float)


def segment_moves(path: np.ndarray, extruding: bool = True) -> list[GCodeMove]:
    """Convert a vertex path into a list of G-code moves."""
    path = np.asarray(path, dtype=float)
    if path.ndim != 2 or path.shape[1] != 3:
        raise ValueError("path must be an (N, 3) array")
    return [GCodeMove(path[i], path[i + 1], extruding) for i in range(len(path) - 1)]


def multi_segment_truss(n_segments: int = P.N_SEGMENTS,
                        advance: float = P.SEGMENT_ADVANCE_MM / 1000.0,
                        overlap: float = P.SEGMENT_OVERLAP_MM / 1000.0,
                        **kwargs) -> list[TrussSegment]:
    """Build the multi-segment truss of Sec. 5.2.

    Each segment is printed with the free-flyer stationary; between segments the
    free-flyer translates by ``advance`` while the print platform stays put
    (Sec. 5.2: "the free-flyer was moved a known distance whilst the print
    platform remained stationary"). Consecutive segments overlap so material is
    deposited onto the previous segment and the two fuse.

    Geometry note. The paper's two element sizes are not the same design, and
    the arithmetic proves it: 7 elements of 110 mm overlapping by any positive
    amount cannot span 775 mm (7 x 110 - 6 x 15 = 680 mm). Sec. 5.1's 110 mm
    element is the *dry-run* element, while Sec. 5.2 states "A design for a
    second truss element was created, such that it would overlap, and therefore
    join to a previously printed truss." So the model takes the *advance* per
    segment as the paper-anchored quantity, 775 / 7 = 110.71 mm, and makes each
    printed element ``advance + overlap`` long so that it laps back onto its
    predecessor. The total extent is then exactly n x advance = 775 mm, and the
    element length (125.7 mm) stays within the arm workspace implied by the
    110 mm element "comfortably" fitting.

    The paper's own caveat is inherited: "it was assumed that the previous
    segment had been printed without errors ... For a spacecraft in orbit, these
    assumptions may not be valid". Segment placement here is open-loop on
    commanded position, with no sensing of where the previous segment actually
    ended up.

    Returns:
        List of :class:`TrussSegment`, positioned along x.
    """
    if n_segments < 1:
        raise ValueError("need at least one segment")
    if overlap < 0.0:
        raise ValueError("overlap must be non-negative")
    if overlap >= advance:
        raise ValueError("overlap must be smaller than the segment advance")

    segments: list[TrussSegment] = []
    for i in range(n_segments):
        origin = i * advance
        # The first segment starts at the origin; every later one starts back
        # inside its predecessor by the overlap, so its first bay prints onto
        # existing material. Each spans up to (i + 1) * advance.
        start_x = origin if i == 0 else origin - overlap
        printed_length = (origin + advance) - start_x
        path = segment_path(length=printed_length, origin_x=start_x, **kwargs)
        segments.append(TrussSegment(index=i, origin_x=origin, path=path))
    return segments


def total_truss_length(segments: list[TrussSegment]) -> float:
    """Overall x extent of a multi-segment truss, m."""
    if not segments:
        return 0.0
    lo = min(float(s.path[:, 0].min()) for s in segments)
    hi = max(float(s.path[:, 0].max()) for s in segments)
    return hi - lo


def check_freeform_limits(path: np.ndarray,
                          max_height: float = P.MAX_FREEFORM_HEIGHT_MM / 1000.0,
                          max_angle_deg: float = P.MAX_FREEFORM_ANGLE_DEG) -> dict:
    """Check a path against the nozzle-geometry limits of Sec. 3 / Fig. 3.

    Returns:
        Dict with ``max_height``, ``max_angle_deg``, ``height_ok``,
        ``angle_ok`` and ``ok``. Angles are measured from the horizontal, and
        only segments with a vertical component are considered -- a purely
        horizontal move has no free-form overhang constraint.
    """
    path = np.asarray(path, dtype=float)
    heights = path[:, 2]
    deltas = np.diff(path, axis=0)

    horizontal = np.linalg.norm(deltas[:, :2], axis=1)
    vertical = np.abs(deltas[:, 2])
    angles = np.where(horizontal > 1e-12,
                      np.degrees(np.arctan2(vertical, horizontal)),
                      np.where(vertical > 1e-12, 90.0, 0.0))

    max_height = float(max_height)
    peak_height = float(np.max(np.abs(heights))) if len(heights) else 0.0
    peak_angle = float(np.max(angles)) if len(angles) else 0.0

    return {
        "max_height": peak_height,
        "max_angle_deg": peak_angle,
        "height_ok": peak_height <= max_height + 1e-12,
        "angle_ok": peak_angle <= max_angle_deg + 1e-9,
        "ok": (peak_height <= max_height + 1e-12) and (peak_angle <= max_angle_deg + 1e-9),
    }
