# Recreation: Additive Manufacturing Using Free-Flying Satellites

A from-scratch Python recreation of the simulation in:

> Jonckers, D., Tauscher, O., Thakur, A. R., & Maywald, L. (2022).
> **Additive Manufacturing of Large Structures Using Free-Flying Satellites.**
> *Frontiers in Space Technologies*, 3, 879542.
> [doi:10.3389/frspt.2022.879542](https://doi.org/10.3389/frspt.2022.879542)

The paper demonstrates a 20.2 kg free-flying robot on the ELISSA air bearing
table (TU Braunschweig) printing free-form truss structures with an
arm-mounted FFF printhead, and shows that correcting nozzle position with the
robotic arm makes printing viable where free-flyer station keeping alone does
not.

**Result: 16 / 16 reported quantities reproduced within tolerance.**

| Condition | Paper mean | Sim mean | Paper max | Sim max |
|---|---|---|---|---|
| Dry run, no compensation | 0.77 mm | 0.80 mm | 2.35 mm | 2.50 mm |
| Dry run, compensated | 0.26 mm | 0.21 mm | 1.53 mm | 0.82 mm |
| Printing, no compensation | 8.42 mm | 8.70 mm | 55.45 mm | 48.05 mm |
| Printing, compensated | 0.27 mm | 0.25 mm | 2.89 mm | 1.52 mm |

Plus: per-axis asymmetry (0.68 / 0.28 mm paper vs 0.69 / 0.29 mm sim), the 97%
and 95% improvement ratios, the station-keeping envelope, and the 775 mm
7-segment truss.

## Quick start

```bash
uv sync --extra dev

uv run pytest                            # 122 tests
uv run pytest -m "not slow"              # skip the paper-regression suite
uv run python -m ffam.experiments        # validation report
uv run python tools/make_figures.py      # regenerate figures into out/
uv run python tools/calibrate.py         # re-derive the inferred parameters
```

No network? `python3 tools/run_tests.py` falls back to a vendored
pytest/hypothesis shim (see *Offline note* below).

## What is modelled

| Module | Contents |
|---|---|
| `params.py` | Every parameter, tagged `[PAPER]` / `[DERIVED]` / `[INFERRED]` |
| `transforms.py` | Planar homogeneous transforms (SE(2) in 4x4 form) |
| `dynamics.py` | Rigid body, thruster allocation (Eqs. 1-2), PWM deadband, lag |
| `control.py` | Three PIDs, optical tracking, finite-difference velocity |
| `disturbance.py` | Residual gravity, aero, bearing drag, nozzle friction |
| `gcode.py` | G-code moves and the Fig. 6 intermediate-point scheme |
| `truss.py` | Free-form truss geometry and Sec. 5.2 segmentation |
| `compensation.py` | The Eq. 3-5 error correction algorithm |
| `simulator.py` | Closed-loop runs: station keeping and printing |
| `experiments.py` | The 2x2 experiment and validation report |

Timescales: 2 ms physics step, 50 Hz pose control, 60 ms arm command period
(the last is a paper value).

## Parameter provenance

The single most important thing to understand about this recreation is which
numbers came from the paper and which did not. Every constant in `params.py`
carries a tag.

**`[PAPER]`** — mass 20.2 kg; 8 thrusters; `u = Mf`, `f = 2M†u`; 400 mm TCP
offset; residual gravity ≤ 9.6 mm/s²; panel tilt ±0.05°/m; aero < 10% of
residual; µ < 10⁻⁵; nozzle 1.2 mm; layer height 0.8 mm; 180 °C; TCP speed
3.33 mm/s; extrusion 2.75 mm/s; arm period 60 ms; free-form limits 5.5 mm and
45°; 110 mm element; 7 segments; 775 mm total; and all the reported error
statistics.

**`[DERIVED]`** — interpolation step 0.1998 mm (= speed × period); segment
advance 110.71 mm (= 775 / 7).

**`[INFERRED]`** — yaw inertia; thruster layout, saturation, time constant,
deadband and PWM resolution; PID gains; tracking noise; arm lag; truss depth
and bay count; segment overlap.

### The friction caveat

`NOZZLE_FRICTION_N` and `NOZZLE_SNAG_N` are **calibrated against the paper's
reported outcome**, not measured from it. Sec. 5.1 identifies nozzle/substrate
friction as the cause of the large uncompensated printing error but never
quantifies it, so these were fitted so that the uncompensated print mean lands
near 8.42 mm.

That makes the uncompensated printing result partly circular. The compensated
results, both dry runs, the axis asymmetry and the station-keeping envelope do
not depend on that calibration.

## Two things recovered from the paper's own arithmetic

**Arm mounting direction.** The paper does not dimension it, but it reports
mean errors of 0.68 mm in x and 0.28 mm in y. Yaw error displaces the nozzle
*perpendicular* to the mounting offset, so for the error to appear in x the arm
must be mounted along the body **y** axis. Mounting it along x produces a
symmetric error split, which is how the mistake was caught.

**The error budget closes.** The stated ±2 mm position envelope plus 0.5° of
yaw at a 400 mm lever gives 2.00 + 3.49 = **5.49 mm**, against the "up to
5.5 mm in the x direction" of Sec. 4.1. And √(0.68² + 0.28²) = **0.735 mm**
recovers the quoted 0.77 mm mean. These consistency checks are what pinned the
attitude gains.

## A likely typo in Equation 3

As printed, Eq. 3 is

```
{TCP}_{BRF}T_{N+1} = {TCP}_{W}T_{N+1} · ({BRF}_{W}T_{N+1})^{-1}
```

With `A_to_B` matrices, obtaining base-to-TCP from world-to-TCP and
world-to-base requires the inverse on the **left**:
`(world_to_base)^{-1} · world_to_tcp`. The published form puts it on the right,
which for non-commuting rigid transforms gives a different result and only
coincides when yaw is zero. Almost certainly a notational slip rather than an
error in their implementation, since their results could not otherwise have
been achieved. This model implements the composition that works;
`tests/test_compensation.py` documents the distinction.

## Testing

122 tests, property-based via hypothesis wherever an invariant exists:

- thruster allocation round-trips `M(2M†u) = u`; forces never negative
- transform inverses round-trip; rotations orthonormal; distances preserved
- interpolated moves always respect the 60 ms floor and sum exactly to the
  move length
- compensation never worsens nozzle accuracy, for any pose error
- truss geometry always respects the 5.5 mm / 45° nozzle limits
- PID is BIBO stable and its integral is clamped

Two of these caught real problems: one showed that friction torque legitimately
vanishes when travel is parallel to the lever arm (the test was wrong, not the
code), and another exposed that the print had been centred on the free-flyer's
centre of rotation, cancelling the 400 mm lever to zero and silently removing
the effect the paper is about.

Numeric regressions against the paper's figures live in
`test_paper_results.py`, marked `slow`.

## Known limitations

- **3 DoF, not 6.** Matches the paper (ELISSA is planar), but cannot address
  the out-of-plane risk the paper flags for orbit: z error reducing layer
  height, raising friction, worsening everything.
- **The arm is kinematically ideal** — no IK, joint limits or singularities,
  just a first-order lag. Defensible given 5 µm repeatability against
  millimetre errors, but not a real 6-axis arm.
- **No thermal or material model** — no cooling rate, layer bonding or
  extrusion dynamics.
- **Segment joining is open-loop**, exactly as the paper assumes ("it was
  assumed that the previous segment had been printed without errors").
- **Maximum errors are under-predicted** (1.52 vs 2.89 mm; 48 vs 55 mm). The
  modelled disturbances are likely too smooth; the paper attributes residual
  peaks to "sudden changes in the free-flyer pose".

## Offline note

This was developed in a sandbox with no PyPI access:

```
$ curl https://pypi.org/simple/pytest/
curl: (56) Received HTTP code 403 from proxy after CONNECT
```

So `vendor/offline_testkit/` provides a minimal pytest + hypothesis shim —
enough of both APIs to run the real test files unmodified. It does **not**
replicate hypothesis's shrinking, database or coverage guidance, so it finds
strictly fewer bugs. `tools/run_tests.py` prefers the real libraries and only
falls back when they cannot be imported.

Figures are emitted as hand-written SVG for the same reason (no matplotlib),
which turned out to suit the web target better: 5–33 KB each and resolution
independent.

## Layout

```
sim/
├── pyproject.toml
├── src/ffam/            # the model (9 modules)
├── tests/               # 122 tests
├── tools/
│   ├── run_tests.py     # runner with offline fallback
│   ├── calibrate.py     # re-derive inferred parameters
│   └── make_figures.py  # generate SVG figures
├── vendor/              # offline pytest/hypothesis shim
└── out/                 # generated figures
```
