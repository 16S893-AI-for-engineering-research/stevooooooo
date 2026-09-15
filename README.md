# Stephen's Portfolio — In-Space Manufacturing

A small deep-space-themed portfolio site.

## Pages

- **`index.html`** — Home / who I am
- **`project.html`** — Outline of my current project: a truss-climbing 3D-printing "spider bot" for building megastructures in space
- **`lab.html`** — Lab notes / research context

## Features

- Multi-page site with shared navigation
- Animated inchworm-style robot crawling along a truss spanning the screen (built and animated in `js/truss.js`)
- Twinkling starfield background (`js/stars.js`)
- Easter egg: click the crawling robot — it pauses, waves, and keeps going

## Running locally

No build step needed. Just serve the folder statically, e.g.:

```bash
python3 -m http.server 8000
```

Then open `http://localhost:8000`.

## Deploying with GitHub Pages

1. Push this repo to GitHub.
2. In the repo settings, enable **GitHub Pages** from the `main` branch (root).
3. The site will be live at `https://<username>.github.io/<repo-name>/`.
