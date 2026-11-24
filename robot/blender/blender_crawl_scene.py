"""Blender scene for the inchworm crawl (run INSIDE Blender, not with uv).

Input: ``out/blender/crawl_data.npz`` + ``crawl_frames.json`` from
``blender_crawl_export.py``. Builds the cell (all links in white plastic), a
long stationary tube (older printed sections behind the robot), a procedural
starfield world, a fixed 3/4 elevated camera, and keys
the robot motion from the validated AA-45 timeline. Saves a .blend and,
unless --no-render, renders an MP4 (EEVEE, 1280x720, 30 fps).

    blender.exe -b --python blender_crawl_scene.py -- --data <dir> [--still N] [--no-render]
"""
import json
import math
import os
import sys

import bpy
import numpy as np
from mathutils import Matrix, Vector

argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []


def arg(name, default=None):
    return argv[argv.index(name) + 1] if name in argv else default


DATA = arg("--data")
STILL = arg("--still")
NO_RENDER = "--no-render" in argv
FPS = 30
HOLD = 15            # frames held at start and end
DURATION = 300       # 10 s total

meta = json.load(open(os.path.join(DATA, "crawl_frames.json")))
arr = np.load(os.path.join(DATA, "crawl_data.npz"))
frames = meta["frames"]
n = len(frames)

# --- clean scene -----------------------------------------------------------
bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
scene.render.fps = FPS
scene.frame_start, scene.frame_end = 1, DURATION

# --- material: white plastic ----------------------------------------------
mat = bpy.data.materials.new("WhitePlastic")
mat.use_nodes = True
bsdf = mat.node_tree.nodes.get("Principled BSDF")
bsdf.inputs["Base Color"].default_value = (0.86, 0.86, 0.85, 1)
bsdf.inputs["Roughness"].default_value = 0.38
for k, v in (("Coat Weight", 0.25), ("Coat Roughness", 0.15),
             ("Subsurface Weight", 0.05), ("Specular IOR Level", 0.5)):
    if k in bsdf.inputs:
        bsdf.inputs[k].default_value = v


def make_mesh(name, link):
    v = arr[f"{link}__v"].astype(float)
    f = arr[f"{link}__f"]
    me = bpy.data.meshes.new(name)
    me.from_pydata(v.tolist(), [], f.tolist())
    me.update()
    me.shade_smooth()
    try:
        me.set_sharpness_by_angle(angle=math.radians(35))
    except Exception:
        pass
    me.materials.append(mat)
    return me


def mat4(m):
    return Matrix([list(r) for r in m])


robot = bpy.data.collections.new("Robot")
scene.collection.children.link(robot)
tubecol = bpy.data.collections.new("Tube")
scene.collection.children.link(tubecol)

# --- tube: stationary, extended with older sections -------------------------
tube_me = make_mesh("tube_section", meta["tube_link"])
for i, P in enumerate(meta["tube_sections"]):
    ob = bpy.data.objects.new(f"tube_{i:02d}", tube_me)
    ob.matrix_world = mat4(P)
    tubecol.objects.link(ob)


# --- robot links, keyed from the timeline ----------------------------------
def frame_of(i):
    return HOLD + 1 + i * (DURATION - 2 * HOLD - 1) / (n - 1)


objs = {}
for link in meta["links"]:
    if link == meta["tube_link"]:
        continue
    ob = bpy.data.objects.new(link, make_mesh(link, link))
    ob.rotation_mode = "QUATERNION"
    robot.objects.link(ob)
    objs[link] = ob

for link, ob in objs.items():
    prev = None
    keys = [(frame_of(i), fr["poses"][link]) for i, fr in enumerate(frames)]
    keys = [(1, frames[0]["poses"][link])] + keys + [(DURATION, frames[-1]["poses"][link])]
    for t, P in keys:
        loc, q, _ = mat4(P).decompose()
        if prev is not None and prev.dot(q) < 0:
            q.negate()
        prev = q
        ob.location, ob.rotation_quaternion = loc, q
        ob.keyframe_insert("location", frame=t)
        ob.keyframe_insert("rotation_quaternion", frame=t)
    ad = ob.animation_data.action
    fcs = []
    try:
        fcs = list(ad.fcurves)
    except AttributeError:  # Blender 5 layered actions
        for layer in ad.layers:
            for strip in layer.strips:
                for bag in strip.channelbags:
                    fcs.extend(bag.fcurves)
    for fc in fcs:
        for kp in fc.keyframe_points:
            kp.interpolation = "LINEAR"

# --- camera: 3/4 elevated, slow half orbit, follows the robot ---------------
def centre(fr):
    ps = [np.array(P)[:3, 3] for l, P in fr["poses"].items()
          if l.startswith(("left_", "right_", "baseforreal", "B_", "In_", "EE"))]
    return Vector(np.mean(ps, axis=0).tolist())

c0, c1 = centre(frames[0]), centre(frames[-1])
target = bpy.data.objects.new("CamTarget", None)
scene.collection.objects.link(target)
target.location = c0  # owner 2026-11-24: fixed camera at the starting view

pivot = bpy.data.objects.new("CamPivot", None)
scene.collection.objects.link(pivot)
pivot.parent = target
cam_data = bpy.data.cameras.new("Cam")
cam_data.lens = 35
cam_data.clip_start, cam_data.clip_end = 0.01, 200
cam = bpy.data.objects.new("Camera", cam_data)
scene.collection.objects.link(cam)
cam.parent = pivot
dist, elev = 1.55, math.radians(28)
cam.location = (dist * math.cos(elev), 0, dist * math.sin(elev))
trk = cam.constraints.new("TRACK_TO")
trk.target = target
trk.track_axis, trk.up_axis = "TRACK_NEGATIVE_Z", "UP_Y"
scene.camera = cam
az0 = math.radians(-35)  # front 3/4, DexArm side visible; no orbit
pivot.rotation_euler = (0, 0, az0)

# --- lights ---------------------------------------------------------------
def light(name, kind, energy, loc, size=1.0, color=(1, 1, 1)):
    ld = bpy.data.lights.new(name, kind)
    ld.energy = energy
    ld.color = color
    if kind == "AREA":
        ld.size = size
    elif kind == "SUN":
        ld.angle = math.radians(2)
    ob = bpy.data.objects.new(name, ld)
    ob.location = loc
    scene.collection.objects.link(ob)
    tr = ob.constraints.new("TRACK_TO")
    tr.target = target
    tr.track_axis, tr.up_axis = "TRACK_NEGATIVE_Z", "UP_Y"
    return ob

light("Sun", "SUN", 4.0, (2, -2, 3), color=(1.0, 0.97, 0.92))
light("Fill", "AREA", 120, (-1.5, 1.2, 1.0), size=1.5, color=(0.75, 0.82, 1.0))
light("Rim", "AREA", 200, (0.5, 1.8, -0.6), size=1.0, color=(0.85, 0.9, 1.0))

# --- world: space ---------------------------------------------------------
world = bpy.data.worlds.new("Space")
scene.world = world
world.use_nodes = True
nt = world.node_tree
nt.nodes.clear()
N = nt.nodes.new
out = N("ShaderNodeOutputWorld")
bg = N("ShaderNodeBackground")
coord = N("ShaderNodeTexCoord")
vor = N("ShaderNodeTexVoronoi")
vor.inputs["Scale"].default_value = 260
vor.inputs["Randomness"].default_value = 1.0
star = N("ShaderNodeMapRange")
star.inputs["From Min"].default_value = 0.0
star.inputs["From Max"].default_value = 0.06
star.inputs["To Min"].default_value = 1.0
star.inputs["To Max"].default_value = 0.0
powr = N("ShaderNodeMath"); powr.operation = "POWER"; powr.inputs[1].default_value = 6
# per-star brightness: voronoi cell colour
sep = N("ShaderNodeSeparateColor")
bri = N("ShaderNodeMath"); bri.operation = "POWER"; bri.inputs[1].default_value = 3
mul = N("ShaderNodeMath"); mul.operation = "MULTIPLY"
gain = N("ShaderNodeMath"); gain.operation = "MULTIPLY"; gain.inputs[1].default_value = 12
# faint nebula
neb = N("ShaderNodeTexNoise")
neb.inputs["Scale"].default_value = 1.6
neb.inputs["Detail"].default_value = 6
ramp = N("ShaderNodeValToRGB")
ramp.color_ramp.elements[0].position = 0.45
ramp.color_ramp.elements[0].color = (0, 0, 0, 1)
ramp.color_ramp.elements[1].position = 0.85
ramp.color_ramp.elements[1].color = (0.05, 0.025, 0.10, 1)
add = N("ShaderNodeMix"); add.data_type = "RGBA"; add.blend_type = "ADD"
add.inputs[0].default_value = 1.0
L = nt.links.new
L(coord.outputs["Generated"], vor.inputs["Vector"])
L(vor.outputs["Distance"], star.inputs["Value"])
L(star.outputs["Result"], powr.inputs[0])
L(vor.outputs["Color"], sep.inputs[0])
L(sep.outputs[0], bri.inputs[0])
L(powr.outputs[0], mul.inputs[0])
L(bri.outputs[0], mul.inputs[1])
L(mul.outputs[0], gain.inputs[0])
L(coord.outputs["Generated"], neb.inputs["Vector"])
L(neb.outputs["Fac"], ramp.inputs["Fac"])
L(ramp.outputs["Color"], add.inputs[6])
L(gain.outputs[0], add.inputs[7])
L(add.outputs[2], bg.inputs["Color"])
bg.inputs["Strength"].default_value = 1.0
L(bg.outputs[0], out.inputs[0])

# --- render settings ------------------------------------------------------
r = scene.render
for eng in ("BLENDER_EEVEE", "BLENDER_EEVEE_NEXT"):
    try:
        r.engine = eng
        break
    except TypeError:
        continue
r.resolution_x, r.resolution_y, r.resolution_percentage = 1280, 720, 100
try:
    scene.eevee.taa_render_samples = 48
    scene.eevee.use_shadows = True
except AttributeError:
    pass
for attr, val in (("use_raytracing", True),):
    if hasattr(scene.eevee, attr):
        setattr(scene.eevee, attr, val)
scene.view_settings.view_transform = "AgX"
try:
    scene.view_settings.look = "AgX - Medium High Contrast"
except TypeError:
    pass

blend = os.path.join(DATA, "inchworm_crawl.blend")
bpy.ops.wm.save_as_mainfile(filepath=blend)
print("SAVED", blend)

if STILL is not None:
    scene.frame_set(int(STILL))
    r.image_settings.file_format = "PNG"
    r.filepath = os.path.join(DATA, f"still_{int(STILL):03d}.png")
    bpy.ops.render.render(write_still=True)
    print("STILL", r.filepath)
elif not NO_RENDER:
    ims = r.image_settings
    if hasattr(ims, "media_type"):
        ims.media_type = "VIDEO"
    ims.file_format = "FFMPEG"
    r.ffmpeg.format = "MPEG4"
    r.ffmpeg.codec = "H264"
    r.ffmpeg.constant_rate_factor = "HIGH"
    r.filepath = os.path.join(DATA, "inchworm_crawl.mp4")
    bpy.ops.render.render(animation=True)
    print("VIDEO", r.filepath)
