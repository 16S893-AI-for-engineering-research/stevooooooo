# Blender render of the inchworm crawl

Renders the validated AA-45 crawl cycle (`out/inchworm/timeline.jsonl`) from the
cell URDF, shown from the tube's point of view: the tube stays still (extended
with older printed sections) and the robot crawls one pitch along it. All parts
are white plastic on a starfield; a fixed 3/4 camera; no DexArm claw gripper.

```bash
cd robot
uv run python scripts/blender_crawl_export.py            # -> out/blender/crawl_*.{npz,json}
B="/mnt/c/Program Files/Blender Foundation/Blender 5.3/blender.exe"
"$B" -b --python "$(wslpath -w blender/blender_crawl_scene.py)" -- --data "$(wslpath -w out/blender)"
# -> out/blender/inchworm_crawl.blend and inchworm_crawl.mp4 (~3 min, EEVEE 1280x720 30 fps)
# add --still 150 to render one PNG frame, or --no-render to only save the .blend
```
