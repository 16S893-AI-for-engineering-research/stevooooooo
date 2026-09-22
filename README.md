# Portfolio site — Stephen Timperley (16.S893)

**Live site:** [16s893-ai-for-engineering-research.github.io/stevooooooo](https://16s893-ai-for-engineering-research.github.io/stevooooooo/)

Static, no-build multi-page site for Assignment 1 of 16.S893 (AI Agents for
Engineering Research).

Requirements:
- Multi-page with navigation
- Who I am + a project outline
- Something animated
- An Easter egg

## Structure

```
index.html      Home — who I am
project.html    Project outline — truss-climbing 3D-printing "spider bot"
simulation.html Paper recreation — free-flying satellite AM simulation
devlog.html     Dev log — agent session notes and activity
css/style.css   Deep-space theme, shared across all pages
js/stars.js     Twinkling starfield background
js/truss.js     Builds the truss + animates the inchworm robot (crawl + wave easter egg)
assets/sim/     Generated SVG figures for the simulation write-up
sim/            Python recreation of Jonckers et al. (2022) — see sim/README.md
papers/         Source papers
```

## Simulation recreation

`sim/` is a from-scratch Python recreation of the simulation in Jonckers,
Tauscher, Thakur & Maywald (2022), *"Additive Manufacturing of Large Structures
Using Free-Flying Satellites"*, Front. Space Technol. 3:879542
([doi](https://doi.org/10.3389/frspt.2022.879542)).

It models a 20.2 kg free-flying robot on an air bearing table printing free-form
truss structures with an arm-mounted extruder, and reproduces the paper's
central result: using the robotic arm to correct nozzle position makes printing
viable where free-flyer station keeping alone does not. **16 of 16 reported
quantities land within tolerance.** Write-up and figures on the
[Simulation](simulation.html) page; full detail in [`sim/README.md`](sim/README.md).

```bash
cd sim
uv sync --extra dev
uv run pytest                        # 122 tests (pytest + hypothesis)
uv run python -m ffam.experiments    # validation report
```

## Project

A concept for a 3D printer that moves along the structure it prints, using
robotic arms to climb, anchor, and extrude — enabling a small satellite to
build megastructures (trusses, solar arrays, habitat frames) far larger than
itself. Full write-up on the [Project](project.html) page.

## Run locally

No build step. From this folder:

```bash
python3 -m http.server 8000
```

Then open http://localhost:8000/.

## Status

Built with the `pi` coding agent, reviewed by hand before publishing.
