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
devlog.html     Dev log — agent session notes and activity
css/style.css   Deep-space theme, shared across all pages
js/stars.js     Twinkling starfield background
js/truss.js     Builds the truss + animates the inchworm robot (crawl + wave easter egg)
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
