"""Recreation of the simulation from Jonckers et al. (2022).

"Additive Manufacturing of Large Structures Using Free-Flying Satellites",
Declan Jonckers, Oliver Tauscher, Aditya R. Thakur, Lasse Maywald,
Institut fuer Raumfahrtsysteme, TU Braunschweig.
Front. Space Technol. 3:879542. doi:10.3389/frspt.2022.879542

The package models a 20.2 kg free-flying robot on the ELISSA air bearing table
printing free-form truss structures with an arm-mounted FFF printhead, and
reproduces the paper's central result: correcting nozzle position with the
robotic arm (Eqs. 3-5) makes printing viable where free-flyer station keeping
alone does not.

Modules:
    params         All parameters, each tagged [PAPER] / [DERIVED] / [INFERRED].
    transforms     Planar homogeneous transforms.
    dynamics       Rigid body, thruster allocation (Eqs. 1-2), thruster lag.
    control        Three PIDs, force allocation, tracking/velocity estimation.
    disturbance    Residual gravity, aero, bearing drag, nozzle friction.
    gcode          G-code moves and the Fig. 6 intermediate-point scheme.
    truss          Free-form truss geometry and Sec. 5.2 segmentation.
    compensation   The Eq. 3-5 error correction algorithm.
    simulator      Closed-loop runs: station keeping and printing.
    experiments    The paper's 2x2 experiment and validation report.
"""

from __future__ import annotations

from . import (compensation, control, disturbance, dynamics, gcode, params,
               simulator, transforms, truss)

__version__ = "0.1.0"

PAPER_CITATION = (
    "Jonckers, D., Tauscher, O., Thakur, A. R., & Maywald, L. (2022). "
    "Additive Manufacturing of Large Structures Using Free-Flying Satellites. "
    "Frontiers in Space Technologies, 3, 879542. "
    "https://doi.org/10.3389/frspt.2022.879542"
)

__all__ = [
    "compensation",
    "control",
    "disturbance",
    "dynamics",
    "gcode",
    "params",
    "simulator",
    "transforms",
    "truss",
    "PAPER_CITATION",
    "__version__",
]
