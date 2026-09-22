"""Generate the figures, as standalone SVG (no matplotlib available offline).

Produces recreations of:

  * Fig. 4 -- free-flyer and nozzle deviation while station keeping for 120 s
  * Fig. 8 -- dry-run TCP path, without (A) and with (B) compensation
  * Fig. 9 -- printing TCP path, without (A) and with (B) compensation
  * a summary bar chart of mean/max error across the four experiment quadrants

matplotlib cannot be installed in this sandbox (PyPI is firewalled), so the SVG
is emitted directly. That turns out to be an advantage for the website: the
output is small, scales crisply, and can be themed with CSS to match the page.

Run:  python3 tools/make_figures.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from ffam import params as P  # noqa: E402
from ffam.experiments import (dry_run_trajectory, run_all_experiments,  # noqa: E402
                              run_multi_segment_truss)
from ffam.simulator import simulate_print  # noqa: E402

OUT = ROOT / "out"

# Palette matched to the custom properties in css/style.css, so the figures sit
# naturally inside the site's deep-space theme.
BG = "#0a0a1c"        # --bg-soft
PANEL = "#14142d"     # --panel, flattened (this SVG avoids alpha)
GRID = "#2a2a52"
TEXT = "#e8e8f5"      # --text
MUTED = "#a9a9c7"     # --text-dim
BLUE = "#4cc9f0"      # --accent-2
RED = "#ff5c8a"
GREEN = "#57e3a0"
AMBER = "#ffc857"
VIOLET = "#7c5cff"    # --accent


class SVG:
    """A tiny SVG builder: just enough for line plots and bar charts."""

    def __init__(self, width: int, height: int, title: str = "") -> None:
        self.width, self.height = width, height
        self.parts: list[str] = []
        self.parts.append(
            f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" '
            f'width="100%" height="auto" font-family="ui-monospace,SFMono-Regular,'
            f'Menlo,monospace" role="img" aria-label="{_esc(title)}">'
        )
        if title:
            self.parts.append(f"<title>{_esc(title)}</title>")
        self.parts.append(f'<rect width="{width}" height="{height}" fill="{BG}"/>')

    def rect(self, x, y, w, h, fill=PANEL, stroke="none", rx=0, opacity=1.0):
        self.parts.append(
            f'<rect x="{x:.2f}" y="{y:.2f}" width="{w:.2f}" height="{h:.2f}" '
            f'fill="{fill}" stroke="{stroke}" rx="{rx}" opacity="{opacity}"/>'
        )

    def line(self, x1, y1, x2, y2, stroke=GRID, width=1.0, dash=None, opacity=1.0):
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
        self.parts.append(
            f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
            f'stroke="{stroke}" stroke-width="{width}"{dash_attr} opacity="{opacity}"/>'
        )

    def polyline(self, points, stroke=BLUE, width=1.2, opacity=1.0, dash=None):
        if len(points) < 2:
            return
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
        # One decimal place is well below a pixel at these sizes and roughly
        # halves the file size versus two.
        coords = " ".join(f"{x:.1f},{y:.1f}" for x, y in points)
        self.parts.append(
            f'<polyline points="{coords}" fill="none" stroke="{stroke}" '
            f'stroke-width="{width}" stroke-linejoin="round" '
            f'stroke-linecap="round"{dash_attr} opacity="{opacity}"/>'
        )

    def text(self, x, y, content, size=11, fill=TEXT, anchor="start",
             weight="normal", rotate=None):
        transform = f' transform="rotate({rotate} {x:.2f} {y:.2f})"' if rotate else ""
        self.parts.append(
            f'<text x="{x:.2f}" y="{y:.2f}" font-size="{size}" fill="{fill}" '
            f'text-anchor="{anchor}" font-weight="{weight}"{transform}>'
            f"{_esc(content)}</text>"
        )

    def save(self, path: Path) -> None:
        self.parts.append("</svg>")
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("\n".join(self.parts), encoding="utf-8")
        print(f"wrote {path.relative_to(ROOT)}  ({path.stat().st_size // 1024} KB)")


def _esc(text: str) -> str:
    return (str(text).replace("&", "&amp;").replace("<", "&lt;")
            .replace(">", "&gt;"))


class Axes:
    """Maps data coordinates to SVG pixels inside a plot box."""

    def __init__(self, svg: SVG, x0, y0, w, h, xlim, ylim,
                 xlabel="", ylabel="", title=""):
        self.svg = svg
        self.x0, self.y0, self.w, self.h = x0, y0, w, h
        self.xlim, self.ylim = xlim, ylim
        svg.rect(x0, y0, w, h, fill=PANEL, rx=3)
        if title:
            svg.text(x0, y0 - 8, title, size=11, fill=TEXT, weight="bold")
        if xlabel:
            svg.text(x0 + w / 2, y0 + h + 34, xlabel, size=10, fill=MUTED,
                     anchor="middle")
        if ylabel:
            svg.text(x0 - 40, y0 + h / 2, ylabel, size=10, fill=MUTED,
                     anchor="middle", rotate=-90)

    def px(self, x):
        lo, hi = self.xlim
        return self.x0 + (x - lo) / (hi - lo) * self.w if hi > lo else self.x0

    def py(self, y):
        lo, hi = self.ylim
        return self.y0 + self.h - (y - lo) / (hi - lo) * self.h if hi > lo else self.y0

    def grid(self, xticks, yticks, xfmt="{:g}", yfmt="{:g}"):
        for x in xticks:
            gx = self.px(x)
            self.svg.line(gx, self.y0, gx, self.y0 + self.h, stroke=GRID, width=0.7)
            self.svg.text(gx, self.y0 + self.h + 16, xfmt.format(x), size=9,
                          fill=MUTED, anchor="middle")
        for y in yticks:
            gy = self.py(y)
            self.svg.line(self.x0, gy, self.x0 + self.w, gy, stroke=GRID, width=0.7)
            self.svg.text(self.x0 - 8, gy + 3, yfmt.format(y), size=9,
                          fill=MUTED, anchor="end")

    def plot(self, xs, ys, simplify=0.35, **kwargs):
        points = [(self.px(x), self.py(y)) for x, y in zip(xs, ys)]
        # Clip to the box so a spike cannot escape the panel.
        points = [(min(max(x, self.x0), self.x0 + self.w),
                   min(max(y, self.y0), self.y0 + self.h)) for x, y in points]
        if simplify and len(points) > 3:
            px = [p[0] for p in points]
            py = [p[1] for p in points]
            points = _simplify(px, py, simplify)
        self.svg.polyline(points, **kwargs)

    def hline(self, y, **kwargs):
        gy = self.py(y)
        self.svg.line(self.x0, gy, self.x0 + self.w, gy, **kwargs)


def _decimate(array, target=1400):
    """Thin a series for SVG size without losing visible structure."""
    if len(array) <= target:
        return np.arange(len(array))
    return np.linspace(0, len(array) - 1, target).astype(int)


def _simplify(xs, ys, tolerance):
    """Drop points that lie within ``tolerance`` of the running direction.

    A cheap perpendicular-distance filter. The traces here are dense (tens of
    thousands of samples at 2 ms) and mostly smooth, so this removes most points
    while preserving every visible excursion -- which matters because the whole
    point of Figs. 8-9 is the shape of the deviation.
    """
    if len(xs) < 3:
        return list(zip(xs, ys))

    kept = [(xs[0], ys[0])]
    anchor_x, anchor_y = xs[0], ys[0]
    previous = (xs[0], ys[0])

    for x, y, in zip(xs[1:], ys[1:]):
        dx, dy = x - anchor_x, y - anchor_y
        length = np.hypot(dx, dy)
        if length < 1e-9:
            previous = (x, y)
            continue
        # Perpendicular distance of the previous point from the anchor->current
        # chord; if it is significant, the previous point carries shape.
        px, py = previous[0] - anchor_x, previous[1] - anchor_y
        deviation = abs(px * dy - py * dx) / length
        if deviation > tolerance:
            kept.append(previous)
            anchor_x, anchor_y = previous
        previous = (x, y)

    kept.append((xs[-1], ys[-1]))
    return kept


def figure_station_keeping(result, path: Path) -> None:
    """Recreation of Fig. 4: free-flyer vs nozzle deviation over 120 s."""
    svg = SVG(880, 430, "Station keeping deviation over 120 s")
    svg.text(24, 28, "Station keeping: free-flyer vs nozzle deviation",
             size=14, weight="bold")
    svg.text(24, 46,
             "Recreation of Figure 4, Jonckers et al. (2022). The nozzle sits "
             "400 mm from the body centre, so yaw error is levered into "
             "position error.",
             size=10, fill=MUTED)

    arrays = result.arrays
    time = arrays["time"] - arrays["time"][0]
    index = _decimate(time)

    flyer = arrays["flyer_error"][:, :2] * 1000.0
    nozzle = arrays["nozzle_error"][:, :2] * 1000.0
    limit = max(np.abs(flyer).max(), np.abs(nozzle).max()) * 1.15

    for column, (label, series) in enumerate([
        ("x deviation", (flyer[:, 0], nozzle[:, 0])),
        ("y deviation", (flyer[:, 1], nozzle[:, 1])),
    ]):
        axes = Axes(svg, 70 + column * 420, 90, 350, 260,
                    xlim=(0, time[-1]), ylim=(-limit, limit),
                    xlabel="time (s)", ylabel="deviation (mm)", title=label)
        ticks = np.linspace(-limit, limit, 5)
        axes.grid(np.linspace(0, time[-1], 5), ticks, xfmt="{:.0f}", yfmt="{:.1f}")
        axes.hline(0.0, stroke=MUTED, width=0.8, dash="3,3", opacity=0.7)
        axes.plot(time[index], series[1][index], stroke=RED, width=1.0,
                  opacity=0.95, simplify=0.12)
        axes.plot(time[index], series[0][index], stroke=BLUE, width=1.2,
                  simplify=0.12)

    legend_y = 392
    svg.line(70, legend_y, 100, legend_y, stroke=BLUE, width=2)
    svg.text(106, legend_y + 4, "free-flyer body", size=10, fill=TEXT)
    svg.line(240, legend_y, 270, legend_y, stroke=RED, width=2)
    svg.text(276, legend_y + 4, "nozzle (TCP)", size=10, fill=TEXT)
    svg.text(430, legend_y + 4,
             f"body max {result.flyer_error_mm().max():.2f} mm   "
             f"nozzle max {result.nozzle_error_mm().max():.2f} mm   "
             f"yaw max {np.abs(result.flyer_attitude_error_deg()).max():.2f} deg",
             size=9, fill=MUTED)
    svg.save(path)


def figure_paths(results, keys, titles, path: Path, heading: str,
                 subtitle: str) -> None:
    """Recreation of Figs. 8 and 9: desired vs achieved TCP path."""
    svg = SVG(880, 470, heading)
    svg.text(24, 28, heading, size=14, weight="bold")
    svg.text(24, 46, subtitle, size=10, fill=MUTED)

    # Common limits so the two panels are directly comparable.
    all_x, all_y = [], []
    for key in keys:
        arrays = results[key].arrays
        for field in ("nozzle_desired", "nozzle_actual"):
            all_x.append(arrays[field][:, 0] * 1000.0)
            all_y.append(arrays[field][:, 1] * 1000.0)
    xs = np.concatenate(all_x)
    ys = np.concatenate(all_y)

    pad = 6.0
    xlim = (xs.min() - pad, xs.max() + pad)
    ylim = (ys.min() - pad, ys.max() + pad)

    for column, (key, title) in enumerate(zip(keys, titles)):
        arrays = results[key].arrays
        stats = results[key].stats()
        axes = Axes(svg, 70 + column * 420, 100, 350, 280,
                    xlim=xlim, ylim=ylim,
                    xlabel="x (mm)", ylabel="y (mm)", title=title)
        axes.grid(np.linspace(xlim[0], xlim[1], 5),
                  np.linspace(ylim[0], ylim[1], 5),
                  xfmt="{:.0f}", yfmt="{:.0f}")

        index = _decimate(arrays["time"], target=2200)
        axes.plot(arrays["nozzle_desired"][index, 0] * 1000.0,
                  arrays["nozzle_desired"][index, 1] * 1000.0,
                  stroke=TEXT, width=1.6, opacity=0.85, simplify=0.2)
        # A tight tolerance on the achieved path: its wobble *is* the result,
        # so it must not be smoothed away.
        axes.plot(arrays["nozzle_actual"][index, 0] * 1000.0,
                  arrays["nozzle_actual"][index, 1] * 1000.0,
                  stroke=RED, width=1.0, opacity=0.9, simplify=0.06)

        reported = P.REPORTED[key]
        svg.text(70 + column * 420, 412,
                 f"sim  mean {stats['mean_mm']:.2f} mm   max {stats['max_mm']:.2f} mm",
                 size=9, fill=TEXT)
        svg.text(70 + column * 420, 428,
                 f"paper mean {reported.mean_mm:.2f} mm   max {reported.max_mm:.2f} mm",
                 size=9, fill=MUTED)

    svg.line(70, 452, 100, 452, stroke=TEXT, width=2)
    svg.text(106, 456, "desired path", size=10, fill=TEXT)
    svg.line(240, 452, 270, 452, stroke=RED, width=2)
    svg.text(276, 456, "achieved nozzle path", size=10, fill=TEXT)
    svg.save(path)


def figure_summary(results, path: Path) -> None:
    """Bar chart: simulated vs reported mean and max error, all four quadrants."""
    svg = SVG(880, 430, "Simulated vs reported error")
    svg.text(24, 28, "Error statistics: recreation vs published values",
             size=14, weight="bold")
    svg.text(24, 46,
             "Log scale. Four experiment quadrants from Section 5.1: dry run "
             "and printing, each without and with arm compensation.",
             size=10, fill=MUTED)

    keys = ["dry_uncompensated", "dry_compensated",
            "print_uncompensated", "print_compensated"]
    labels = ["dry\nno comp.", "dry\ncompensated",
              "print\nno comp.", "print\ncompensated"]

    lo, hi = 0.1, 100.0

    def to_y(value):
        value = max(value, lo)
        frac = (np.log10(value) - np.log10(lo)) / (np.log10(hi) - np.log10(lo))
        return 330 - frac * 230

    for decade in (0.1, 1.0, 10.0, 100.0):
        y = to_y(decade)
        svg.line(80, y, 830, y, stroke=GRID, width=0.7)
        svg.text(72, y + 3, f"{decade:g}", size=9, fill=MUTED, anchor="end")
    svg.text(40, 215, "error (mm)", size=10, fill=MUTED, anchor="middle",
             rotate=-90)

    group_width = 180
    for i, (key, label) in enumerate(zip(keys, labels)):
        stats = results[key].stats()
        reported = P.REPORTED[key]
        x0 = 100 + i * group_width

        bars = [
            ("sim mean", stats["mean_mm"], BLUE),
            ("paper mean", reported.mean_mm, MUTED),
            ("sim max", stats["max_mm"], AMBER),
            ("paper max", reported.max_mm, MUTED),
        ]
        for j, (_, value, colour) in enumerate(bars):
            bx = x0 + j * 27
            by = to_y(value)
            opacity = 0.55 if colour == MUTED else 1.0
            svg.rect(bx, by, 22, 330 - by, fill=colour, rx=2, opacity=opacity)
            svg.text(bx + 11, by - 5, f"{value:.2f}", size=8, fill=TEXT,
                     anchor="middle")

        for line_no, part in enumerate(label.split("\n")):
            svg.text(x0 + 54, 352 + line_no * 13, part, size=10, fill=TEXT,
                     anchor="middle")

    legend = [("simulated mean", BLUE), ("simulated max", AMBER),
              ("published (Jonckers et al. 2022)", MUTED)]
    x = 100
    for name, colour in legend:
        svg.rect(x, 396, 14, 10, fill=colour, rx=2,
                 opacity=0.55 if colour == MUTED else 1.0)
        svg.text(x + 20, 405, name, size=9, fill=TEXT)
        x += len(name) * 5.6 + 46
    svg.save(path)


def figure_truss(path: Path) -> None:
    """The 7-segment, 775 mm truss of Sec. 5.2."""
    truss = run_multi_segment_truss(seed=0, compensate=True)
    segments = truss["segments"]

    svg = SVG(880, 300, "Seven-segment 775 mm truss")
    svg.text(24, 28, "Multi-segment truss: 7 segments, 775 mm",
             size=14, weight="bold")
    svg.text(24, 46,
             "Section 5.2. Each segment is printed with the free-flyer "
             "stationary, then it translates by 110.7 mm; segments overlap so "
             "they fuse into one structure.",
             size=10, fill=MUTED)

    all_x = np.concatenate([s.path[:, 0] for s in segments]) * 1000.0
    all_y = np.concatenate([s.path[:, 1] for s in segments]) * 1000.0
    xlim = (all_x.min() - 20, all_x.max() + 20)
    ylim = (all_y.min() - 14, all_y.max() + 14)

    axes = Axes(svg, 70, 100, 760, 120, xlim=xlim, ylim=ylim,
                xlabel="x (mm)", ylabel="y (mm)")
    axes.grid(np.linspace(0, 775, 6), np.linspace(ylim[0], ylim[1], 3),
              xfmt="{:.0f}", yfmt="{:.0f}")

    palette = [BLUE, GREEN, AMBER, VIOLET]
    for segment in segments:
        colour = palette[segment.index % len(palette)]
        axes.plot(segment.path[:, 0] * 1000.0, segment.path[:, 1] * 1000.0,
                  stroke=colour, width=1.4, opacity=0.9)

    means = [r.stats()["mean_mm"] for r in truss["results"]]
    svg.text(70, 258,
             f"total length {truss['total_length_mm']:.1f} mm "
             f"(paper: {P.TOTAL_TRUSS_LENGTH_MM:.0f} mm)   |   "
             f"per-segment mean error {np.mean(means):.2f} mm   |   "
             f"nozzle diameter {P.NOZZLE_DIAMETER_MM} mm",
             size=10, fill=TEXT)
    svg.text(70, 276,
             "Colour indicates print order; overlaps at the joins are where "
             "material is deposited onto the previous segment.",
             size=9, fill=MUTED)
    svg.save(path)


def main() -> None:
    print("Running experiments for figures (a few minutes)...\n")
    results = run_all_experiments(seed=0)

    figure_station_keeping(results["station_keeping"],
                           OUT / "fig4_station_keeping.svg")
    figure_paths(
        results,
        ["dry_uncompensated", "dry_compensated"],
        ["A  without compensation", "B  with compensation"],
        OUT / "fig8_dry_run.svg",
        "Dry run: nozzle path with and without error correction",
        "Recreation of Figure 8. Printhead disabled and platform removed, so "
        "there is no contact friction -- only free-flyer pose error.",
    )
    figure_paths(
        results,
        ["print_uncompensated", "print_compensated"],
        ["A  without compensation", "B  with compensation"],
        OUT / "fig9_printing.svg",
        "Printing: nozzle path with and without error correction",
        "Recreation of Figure 9. Nozzle friction against the substrate now "
        "disturbs the free-flyer, and without correction the path is unusable.",
    )
    figure_summary(results, OUT / "summary_errors.svg")
    figure_truss(OUT / "truss_775mm.svg")

    print("\nFigures written to", OUT.relative_to(ROOT))


if __name__ == "__main__":
    main()
