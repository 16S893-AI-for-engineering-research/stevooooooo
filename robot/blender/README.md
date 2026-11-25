# Blender render of the inchworm crawl

Renders the validated AA-45 crawl cycle (`out/inchworm/timeline.jsonl`) from the
cell URDF, from the tube's point of view: the tube stays still (extended with
older printed sections) and the robot crawls one pitch along it. Storyline: the
DexArm finishes the final layer on the tube's newest face (during the 2 s
zoom-in), stows, the arms regrip-crawl one pitch, then the DexArm starts the next
section on the same face. The prints are pretend spiral beads followed by DexArm
position IK (visual only: no G-code, not collision-checked). Printed parts are
white plastic, STS3215 servos black; no DexArm claw gripper. Fixed 3/4 side
camera. Backdrop: orbit (star map, textured Earth on the left of the frame,
clouds, thin limb band on the sunlit side).

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
# -> out/blender/inchworm_crawl.blend + inchworm_crawl.mp4 (EEVEE 1280x720 30 fps, 13.4 s)
# --still N renders one PNG frame; --no-render only saves the .blend;
# --earth-lat / --earth-lon (deg, default 12 / -25 = tropical Atlantic) choose the
# region shown next to the limb.
```
