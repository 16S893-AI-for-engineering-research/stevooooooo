"""Blender scene for the inchworm crawl (run INSIDE Blender, not with uv).

Input: ``out/blender/crawl_data.npz`` + ``crawl_frames.json`` from
``blender_crawl_export.py``. Builds the cell (all links in white plastic), a
long stationary tube (older printed sections behind the robot), a procedural
Earth-orbit backdrop (star map, textured Earth, atmosphere), a fixed 3/4 elevated camera, and keys
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
INTRO = 60           # 2 s zoom-in from far away, robot still at the start pose
CRAWL = 270          # 9 s crawl cycle
HOLD_END = 15        # 0.5 s hold on the final pose
DURATION = INTRO + CRAWL + HOLD_END

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
    return INTRO + 1 + i * (CRAWL - 1) / (n - 1)


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

# --- camera: fixed 3/4 side view with an intro zoom-in ---------------
def centre(fr):
    ps = [np.array(P)[:3, 3] for l, P in fr["poses"].items()
          if l.startswith(("left_", "right_", "baseforreal", "B_", "In_", "EE"))]
    return Vector(np.mean(ps, axis=0).tolist())

c0, c1 = centre(frames[0]), centre(frames[-1])
target = bpy.data.objects.new("CamTarget", None)
scene.collection.objects.link(target)
target.location = (c0 + c1) / 2  # owner 2026-11-24: fixed, centred on the robot's travel

pivot = bpy.data.objects.new("CamPivot", None)
scene.collection.objects.link(pivot)
pivot.parent = target
cam_data = bpy.data.cameras.new("Cam")
cam_data.lens = 35
cam_data.clip_start, cam_data.clip_end = 0.01, 200
cam = bpy.data.objects.new("Camera", cam_data)
scene.collection.objects.link(cam)
cam.parent = pivot
elev = math.radians(24)
# owner 2026-11-24: zoom in from far away to a slightly wider final view
for t, dist in ((1, 4.2), (INTRO, 1.95)):
    cam.location = (dist * math.cos(elev), 0, dist * math.sin(elev))
    cam.keyframe_insert("location", frame=t)
trk = cam.constraints.new("TRACK_TO")
trk.target = target
trk.track_axis, trk.up_axis = "TRACK_NEGATIVE_Z", "UP_Y"
scene.camera = cam
az0 = math.radians(-18)  # more side-on (0 = pure side view along -X); no orbit
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
light("Earthshine", "AREA", 160, (-0.6, 0.4, -1.2), size=2.0, color=(0.55, 0.7, 1.0))

# --- world: low Earth orbit ----------------------------------------------
# Owner 2026-11-24: realistic space, in orbit around Earth. Textures (CC BY 4.0,
# Solar System Scope, based on NASA imagery) live in --tex; see README.md.
# The Earth is scaled down with its distance (apparent size unchanged) so the
# depth range stays small: real ratio orbit radius / Earth radius = 9371/6371.
TEX = arg("--tex", os.path.join(DATA, "tex"))
EARTH_R = 500.0
EARTH_D = EARTH_R * 9371.0 / 6371.0     # ~3000 km altitude view
LIMB_DEPRESSION = math.radians(13)      # horizon just above the frame centre


def img(name, non_color=False):
    im = bpy.data.images.load(os.path.join(TEX, name), check_existing=True)
    if non_color:
        im.colorspace_settings.name = "Non-Color"
    return im


def tex_node(nt, name, non_color=False):
    t = nt.nodes.new("ShaderNodeTexImage")
    t.image = img(name, non_color)
    t.interpolation = "Cubic"
    return t


# Stars: real sky map, dim (sunlit subject = short exposure), no nebula tint.
world = bpy.data.worlds.new("Space")
scene.world = world
world.use_nodes = True
nt = world.node_tree
nt.nodes.clear()
env = nt.nodes.new("ShaderNodeTexEnvironment")
env.image = img("8k_stars_milky_way.jpg")
bg = nt.nodes.new("ShaderNodeBackground")
bg.inputs["Strength"].default_value = 0.22
wout = nt.nodes.new("ShaderNodeOutputWorld")
nt.links.new(env.outputs["Color"], bg.inputs["Color"])
nt.links.new(bg.outputs[0], wout.inputs[0])

# Earth placement: beyond the robot along the camera's view direction, below it.
view_h = Vector((-math.cos(az0), -math.sin(az0), 0.0))  # camera looks along -X rotated by az0
alpha = math.asin(EARTH_R / EARTH_D)
dep = LIMB_DEPRESSION + alpha
earth_c = target.location + EARTH_D * (math.cos(dep) * view_h + Vector((0, 0, -math.sin(dep))))
cam_data.clip_end = EARTH_D * 1.6


def sphere(name, radius, mat_):
    bpy.ops.mesh.primitive_uv_sphere_add(segments=256, ring_count=128, radius=radius,
                                         location=earth_c)
    ob = bpy.context.active_object
    ob.name = name
    bpy.ops.object.shade_smooth()
    ob.data.materials.append(mat_)
    return ob


# The globe is far outside any shadow map, so the scene lights cannot light it
# (it renders black). It is self-lit instead: emission = texture * Lambert term
# for the same sun direction as the "Sun" light, so the lighting stays consistent.
SUN_TO = Vector((2, -2, 3)).normalized()  # direction toward the sun (matches light "Sun")


def lambert(n, gain=1.0, wrap=0.08):
    geo = n.nodes.new("ShaderNodeNewGeometry")
    dot = n.nodes.new("ShaderNodeVectorMath"); dot.operation = "DOT_PRODUCT"
    dot.inputs[1].default_value = SUN_TO
    n.links.new(geo.outputs["Normal"], dot.inputs[0])
    mr = n.nodes.new("ShaderNodeMapRange")  # soft terminator
    mr.inputs["From Min"].default_value = -wrap
    mr.inputs["From Max"].default_value = 1.0
    mr.inputs["To Max"].default_value = gain
    n.links.new(dot.outputs["Value"], mr.inputs["Value"])
    return mr.outputs["Result"], dot.outputs["Value"]


def emissive_material(name):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    m.node_tree.nodes.clear()
    out = m.node_tree.nodes.new("ShaderNodeOutputMaterial")
    return m, m.node_tree, out


# Surface: day map lit by the sun; city lights on the night side.
em, n, eo = emissive_material("Earth")
day = tex_node(n, "8k_earth_daymap.jpg")
night = tex_node(n, "8k_earth_nightmap.jpg")
lit, ndl = lambert(n, gain=1.6)
dmul = n.nodes.new("ShaderNodeVectorMath"); dmul.operation = "SCALE"
n.links.new(day.outputs["Color"], dmul.inputs[0]); n.links.new(lit, dmul.inputs["Scale"])
dark = n.nodes.new("ShaderNodeMapRange")  # night lights only where ndl < 0
dark.inputs["From Min"].default_value = 0.0
dark.inputs["From Max"].default_value = -0.15
dark.inputs["To Max"].default_value = 0.6
n.links.new(ndl, dark.inputs["Value"])
nmul = n.nodes.new("ShaderNodeVectorMath"); nmul.operation = "SCALE"
n.links.new(night.outputs["Color"], nmul.inputs[0]); n.links.new(dark.outputs["Result"], nmul.inputs["Scale"])
sadd = n.nodes.new("ShaderNodeVectorMath"); sadd.operation = "ADD"
n.links.new(dmul.outputs[0], sadd.inputs[0]); n.links.new(nmul.outputs[0], sadd.inputs[1])
ee = n.nodes.new("ShaderNodeEmission")
n.links.new(sadd.outputs[0], ee.inputs["Color"])
n.links.new(ee.outputs[0], eo.inputs["Surface"])
earth = sphere("Earth", EARTH_R, em)

# Clouds: sunlit white, alpha from the cloud map.
cm, n, co = emissive_material("Clouds")
cl = tex_node(n, "8k_earth_clouds.jpg", non_color=True)
clit, _ = lambert(n, gain=1.9)
ce = n.nodes.new("ShaderNodeEmission")
ce.inputs["Color"].default_value = (1, 1, 1, 1)
n.links.new(clit, ce.inputs["Strength"])
ct = n.nodes.new("ShaderNodeBsdfTransparent")
mix = n.nodes.new("ShaderNodeMixShader")
n.links.new(cl.outputs["Color"], mix.inputs["Fac"])
n.links.new(ct.outputs[0], mix.inputs[1]); n.links.new(ce.outputs[0], mix.inputs[2])
n.links.new(mix.outputs[0], co.inputs["Surface"])
if hasattr(cm, "surface_render_method"):
    cm.surface_render_method = "BLENDED"
clouds = sphere("Clouds", EARTH_R * 1.004, cm)

# Atmosphere: thin blue limb glow (emission rising toward grazing angles).
am = bpy.data.materials.new("Atmosphere")
am.use_nodes = True
n = am.node_tree
n.nodes.clear()
lw = n.nodes.new("ShaderNodeLayerWeight")
lw.inputs["Blend"].default_value = 0.35
pw = n.nodes.new("ShaderNodeMath"); pw.operation = "POWER"; pw.inputs[1].default_value = 3.0
emi = n.nodes.new("ShaderNodeEmission")
emi.inputs["Color"].default_value = (0.30, 0.55, 1.0, 1)
emi.inputs["Strength"].default_value = 2.5
tr = n.nodes.new("ShaderNodeBsdfTransparent")
add = n.nodes.new("ShaderNodeAddShader")
mo = n.nodes.new("ShaderNodeOutputMaterial")
n.links.new(lw.outputs["Facing"], pw.inputs[0])
mul = n.nodes.new("ShaderNodeMath"); mul.operation = "MULTIPLY"
n.links.new(pw.outputs[0], mul.inputs[0])
alit, _ = lambert(n, gain=1.0, wrap=0.25)
n.links.new(alit, mul.inputs[1])
n.links.new(mul.outputs[0], emi.inputs["Strength"])
n.links.new(emi.outputs[0], add.inputs[0])
n.links.new(tr.outputs[0], add.inputs[1])
n.links.new(add.outputs[0], mo.inputs["Surface"])
if hasattr(am, "surface_render_method"):
    am.surface_render_method = "BLENDED"
atmo = sphere("Atmosphere", EARTH_R * 1.025, am)
for ob in (earth, clouds, atmo):  # the far globe must not shadow the robot
    ob.visible_shadow = False

# Orient the globe (which region is in view) and let it turn slowly; clouds drift.
EARTH_YAW = math.radians(float(arg("--earth-yaw", "200")))
EARTH_TILT = math.radians(float(arg("--earth-tilt", "-25")))
for ob, extra in ((earth, 0.0), (clouds, 0.6), (atmo, 0.0)):
    for t, spin in ((1, 0.0), (DURATION, math.radians(1.2) + math.radians(extra))):
        ob.rotation_euler = (EARTH_TILT, 0.0, EARTH_YAW + spin)
        ob.keyframe_insert("rotation_euler", frame=t)

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
