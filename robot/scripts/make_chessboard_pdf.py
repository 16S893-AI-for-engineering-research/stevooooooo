#!/usr/bin/env python3
"""Generate the calibration chessboard PDF (AA-30): 9x6 inner corners, 25 mm squares.

The board is 10 x 7 squares = 250 x 175 mm on an A4 landscape page (297 x 210 mm),
centred, so it also prints on US Letter at 100 % ("actual size", NOT "fit to page").
A 100 mm scale bar is printed below the board so the owner can check the print scale
with calipers/ruler before use.

    uv run python scripts/make_chessboard_pdf.py            # writes assets/chessboard_9x6_25mm.pdf

No third-party dependency: the PDF is written directly.
"""

from __future__ import annotations

import argparse
from pathlib import Path

MM = 72.0 / 25.4  # PDF points per mm
PAGE_W_MM, PAGE_H_MM = 297.0, 210.0


def build_pdf(inner_cols: int = 9, inner_rows: int = 6, square_mm: float = 25.0) -> bytes:
    sq_x, sq_y = inner_cols + 1, inner_rows + 1  # squares across / down
    board_w, board_h = sq_x * square_mm, sq_y * square_mm
    x0 = (PAGE_W_MM - board_w) / 2.0
    y0 = (PAGE_H_MM - board_h) / 2.0  # board bottom edge (PDF origin is bottom-left)

    ops = ["0 g"]
    for r in range(sq_y):
        for c in range(sq_x):
            if (r + c) % 2 == 0:  # top-left square black, like OpenCV's reference board
                x = x0 + c * square_mm
                y = y0 + (sq_y - 1 - r) * square_mm
                ops.append(f"{x*MM:.3f} {y*MM:.3f} {square_mm*MM:.3f} {square_mm*MM:.3f} re f")
    # 100 mm scale bar under the board.
    bar_y = y0 - 10.0
    ops.append(
        f"0.6 w {x0*MM:.3f} {bar_y*MM:.3f} m {(x0+100)*MM:.3f} {bar_y*MM:.3f} l S"
    )
    for xt in (x0, x0 + 100):
        ops.append(f"{xt*MM:.3f} {(bar_y-1.5)*MM:.3f} m {xt*MM:.3f} {(bar_y+1.5)*MM:.3f} l S")
    label = (
        f"Chessboard {inner_cols}x{inner_rows} inner corners, {square_mm:g} mm squares. "
        "Print at 100% (actual size). Bar = 100 mm; one square = 25 mm: verify with calipers."
    )
    ops.append(
        f"BT /F1 8 Tf {x0*MM:.3f} {(bar_y-7)*MM:.3f} Td ({label}) Tj ET"
    )
    content = "\n".join(ops).encode("latin-1")

    objs = [
        b"<< /Type /Catalog /Pages 2 0 R >>",
        b"<< /Type /Pages /Kids [3 0 R] /Count 1 >>",
        (
            f"<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {PAGE_W_MM*MM:.3f} {PAGE_H_MM*MM:.3f}] "
            "/Contents 4 0 R /Resources << /Font << /F1 5 0 R >> >> >>"
        ).encode(),
        b"<< /Length %d >>\nstream\n" % len(content) + content + b"\nendstream",
        b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>",
    ]
    out = bytearray(b"%PDF-1.4\n")
    offsets = []
    for i, body in enumerate(objs, start=1):
        offsets.append(len(out))
        out += f"{i} 0 obj\n".encode() + body + b"\nendobj\n"
    xref = len(out)
    out += f"xref\n0 {len(objs)+1}\n0000000000 65535 f \n".encode()
    for off in offsets:
        out += f"{off:010d} 00000 n \n".encode()
    out += f"trailer\n<< /Size {len(objs)+1} /Root 1 0 R >>\nstartxref\n{xref}\n%%EOF\n".encode()
    return bytes(out)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", type=Path, default=Path(__file__).resolve().parents[1] / "assets" / "chessboard_9x6_25mm.pdf")
    args = ap.parse_args()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_bytes(build_pdf())
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
