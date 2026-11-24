# Blender render of the inchworm crawl

Renders the validated AA-45 crawl cycle (`out/inchworm/timeline.jsonl`) from the
cell URDF, from the tube's point of view: the tube stays still (extended with
older printed sections) and the robot crawls one pitch along it. All parts are
white plastic; no DexArm claw gripper. A fixed 3/4 side camera zooms in over the
first 2 s. Backdrop: low Earth orbit (star map, textured Earth, clouds, blue limb).

## Textures (not committed; download once into `out/blender/tex/`)

```bash
mkdir -p out/blender/tex && cd out/blender/tex
for f in 8k_earth_daymap.jpg 8k_earth_nightmap.jpg 8k_earth_clouds.jpg 8k_stars_milky_way.jpg; do
  curl -sSLO -A "Mozilla/5.0" "https://www.solarsystemscope.com/textures/download/$f"; done
cd ../../..
```

Source: Solar System Scope (CC BY 4.0, based on NASA imagery). Credit it if the
video is published.

## Render

```bash
cd robot
uv run python scripts/blender_crawl_export.py            # -> out/blender/crawl_*.{npz,json}
B="/mnt/c/Program Files/Blender Foundation/Blender 5.3/blender.exe"
"$B" -b --python "$(wslpath -w blender/blender_crawl_scene.py)" -- --data "$(wslpath -w out/blender)"
# -> out/blender/inchworm_crawl.blend + inchworm_crawl.mp4 (~6 min, EEVEE 1280x720 30 fps, 11.5 s)
# --still N renders one PNG frame; --no-render only saves the .blend;
# --earth-yaw / --earth-tilt (deg) choose which part of the globe is in view.
```
