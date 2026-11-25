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
# Owner 2026-11-24 storyline: the final layer of the current section finishes
# (during the zoom-in), which triggers the regrip crawl; after the crawl the
# DexArm starts printing the next section on the same tube face.
INTRO = 60           # 2 s zoom-in from far away
P1_START, P1_END = 8, 120         # final layer
RETRACT = 15         # DexArm head to/from its stowed pose
CRAWL = 270          # 9 s crawl cycle
CRAWL_START = P1_END + RETRACT
CRAWL_END = CRAWL_START + CRAWL
P2_START = CRAWL_END + RETRACT
P2_END = P2_START + 112           # first part of the next section
HOLD_END = 20
DURATION = P2_END + HOLD_END

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

# Owner 2026-11-24: all STS3215 servos black (semi-gloss plastic housing).
servo_mat = bpy.data.materials.new("BlackServo")
servo_mat.use_nodes = True
sb = servo_mat.node_tree.nodes.get("Principled BSDF")
sb.inputs["Base Color"].default_value = (0.018, 0.018, 0.02, 1)
sb.inputs["Roughness"].default_value = 0.45


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
    mk = f"{link}__m"
    if mk in arr.files and arr[mk].any():
        me.materials.append(servo_mat)
        me.polygons.foreach_set("material_index", arr[mk].astype(np.int32))
        me.update()
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


# --- pretend print beads: spirals on the newest tube end face, growing behind
# the nozzle (visual only; beads stay with the tube, which never moves).
hot = bpy.data.materials.new("HotTip")
hot.use_nodes = True
hb = hot.node_tree.nodes.get("Principled BSDF")
hb.inputs["Emission Color"].default_value = (1.0, 0.35, 0.05, 1)
hb.inputs["Base Color"].default_value = (1.0, 0.4, 0.1, 1)
BEAD_R = meta["print"]["bead_r"]
bpy.ops.mesh.primitive_uv_sphere_add(radius=BEAD_R * 1.3, segments=16, ring_count=8)
tip = bpy.context.active_object
tip.name = "HotTip"
tip.data.materials.append(hot)
for coll in tip.users_collection:
    coll.objects.unlink(tip)
tubecol.objects.link(tip)
tip.scale = (0.001,) * 3
tip.keyframe_insert("scale", frame=1)
hb.inputs["Emission Strength"].default_value = 0.0
hb.inputs["Emission Strength"].keyframe_insert("default_value", frame=1)

for pi, (pdata, t0, t1) in enumerate(zip(meta["print"]["prints"],
                                         (P1_START, P2_START), (P1_END, P2_END))):
    pts = pdata["bead_pts"]
    cu = bpy.data.curves.new(f"PrintBead{pi}", "CURVE")
    cu.dimensions = "3D"
    cu.bevel_depth = BEAD_R
    cu.bevel_resolution = 3
    cu.use_fill_caps = True
    sp = cu.splines.new("POLY")
    sp.points.add(len(pts) - 1)
    for i_, p_ in enumerate(pts):
        sp.points[i_].co = (*p_, 1.0)
    ob = bpy.data.objects.new(f"PrintBead{pi}", cu)
    tubecol.objects.link(ob)
    cu.materials.append(mat)
    for t, f in ((1, 0.0), (t0, 0.0), (t1, 1.0)):
        cu.bevel_factor_end = f
        cu.keyframe_insert("bevel_factor_end", frame=t)
    for k in range(0, len(pts), 4):
        tip.location = pts[k]
        tip.keyframe_insert("location", frame=t0 + k * (t1 - t0) / (len(pts) - 1))
    for t, on in ((t0 - 1, False), (t0, True), (t1, True), (t1 + 6, False)):
        tip.scale = (1, 1, 1) if on else (0.001,) * 3
        tip.keyframe_insert("scale", frame=t)
        hb.inputs["Emission Strength"].default_value = 6.0 if on else 0.0
        hb.inputs["Emission Strength"].keyframe_insert("default_value", frame=t)


# --- robot links, keyed from the timeline ----------------------------------
def frame_of(i):
    return CRAWL_START + i * CRAWL / (n - 1)


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
    pr = meta["print"]["prints"]
    if link in pr[0]["poses"][0]:  # DexArm link: print, stow, crawl, print again
        def path(pp, t0, t1):
            return [(t0 + k * (t1 - t0) / (len(pp) - 1), P[link]) for k, P in enumerate(pp)]
        keys = ([(1, pr[0]["poses"][0][link])] + path(pr[0]["poses"], P1_START, P1_END)
                + keys + path(pr[1]["poses"], P2_START, P2_END))
        keys.append((DURATION, pr[1]["poses"][-1][link]))
    else:
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
FINAL_DIST = 1.95
for t, dist in ((1, 4.2), (INTRO, FINAL_DIST)):
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
# Owner 2026-11-24: Earth on the LEFT of the frame, robot against black space.
# The limb passes through this point (deg left of / below the frame centre).
LIMB_LEFT, LIMB_DOWN = math.radians(13), math.radians(4)


def img(name, non_color=False):
    im = bpy.data.images.load(os.path.join(TEX, name), check_existing=True)
    if non_color:
        im.colorspace_settings.name = "Non-Color"
    return im


def tex_node(nt, name, non_color=False):
    """Equirectangular texture on the globe: lon = atan2(y, x), lat = asin(z) in
    object space (image left edge = 180 W, standard for these maps)."""
    t = nt.nodes.new("ShaderNodeTexImage")
    t.image = img(name, non_color)
    t.interpolation = "Cubic"
    t.extension = "REPEAT"
    tc = nt.nodes.new("ShaderNodeTexCoord")
    nrm = nt.nodes.new("ShaderNodeVectorMath"); nrm.operation = "NORMALIZE"
    nt.links.new(tc.outputs["Object"], nrm.inputs[0])
    sep = nt.nodes.new("ShaderNodeSeparateXYZ")
    nt.links.new(nrm.outputs[0], sep.inputs[0])
    at = nt.nodes.new("ShaderNodeMath"); at.operation = "ARCTAN2"
    nt.links.new(sep.outputs["Y"], at.inputs[0]); nt.links.new(sep.outputs["X"], at.inputs[1])
    u = nt.nodes.new("ShaderNodeMath"); u.operation = "MULTIPLY_ADD"
    nt.links.new(at.outputs[0], u.inputs[0])
    u.inputs[1].default_value = 1 / (2 * math.pi); u.inputs[2].default_value = 0.5
    asn = nt.nodes.new("ShaderNodeMath"); asn.operation = "ARCSINE"
    nt.links.new(sep.outputs["Z"], asn.inputs[0])
    v = nt.nodes.new("ShaderNodeMath"); v.operation = "MULTIPLY_ADD"
    nt.links.new(asn.outputs[0], v.inputs[0])
    v.inputs[1].default_value = 1 / math.pi; v.inputs[2].default_value = 0.5
    cmb = nt.nodes.new("ShaderNodeCombineXYZ")
    nt.links.new(u.outputs[0], cmb.inputs["X"]); nt.links.new(v.outputs[0], cmb.inputs["Y"])
    nt.links.new(cmb.outputs[0], t.inputs["Vector"])
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

# Earth placement from the final camera pose: the limb runs through the chosen
# point left of centre and the globe lies beyond it (left and slightly down).
cam_final = target.location + Matrix.Rotation(az0, 3, "Z") @ Vector(
    (FINAL_DIST * math.cos(elev), 0, FINAL_DIST * math.sin(elev)))
fwd = (target.location - cam_final).normalized()
right = fwd.cross(Vector((0, 0, 1))).normalized()
up = right.cross(fwd)
limb_dir = (fwd - math.tan(LIMB_LEFT) * right - math.tan(LIMB_DOWN) * up).normalized()
outward = -right * math.cos(math.radians(25)) - up * math.sin(math.radians(25))
outward = (outward - outward.dot(limb_dir) * limb_dir).normalized()
alpha = math.asin(EARTH_R / EARTH_D)
c_dir = limb_dir * math.cos(alpha) + outward * math.sin(alpha)
earth_c = cam_final + EARTH_D * c_dir
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

# Atmosphere as seen from orbit: a thin bright band hugging the limb that
# fades fast into black space (no wide glow), a faint haze over the disc near
# the limb, and only on the sunlit side. Shell thickness ~100 km at scale.
ATMO_R = EARTH_R * 1.016
am, n, ao = emissive_material("Atmosphere")
geo = n.nodes.new("ShaderNodeNewGeometry")
d = n.nodes.new("ShaderNodeVectorMath"); d.operation = "DOT_PRODUCT"
n.links.new(geo.outputs["Normal"], d.inputs[0]); n.links.new(geo.outputs["Incoming"], d.inputs[1])
ab = n.nodes.new("ShaderNodeMath"); ab.operation = "ABSOLUTE"
n.links.new(d.outputs["Value"], ab.inputs[0])
# grazing = 1 - |N.I|; the Earth's limb sits at grazing = 1 - sqrt(1 - (R/ATMO_R)^2)
g = n.nodes.new("ShaderNodeMath"); g.operation = "SUBTRACT"
g.inputs[0].default_value = 1.0
n.links.new(ab.outputs[0], g.inputs[1])
limb_g = 1 - math.sqrt(1 - (EARTH_R / ATMO_R) ** 2)
ramp = n.nodes.new("ShaderNodeValToRGB")
els = ramp.color_ramp.elements
els[0].position, els[0].color = 0.0, (0, 0, 0, 1)
els[1].position, els[1].color = 1.0, (0, 0, 0, 1)
for pos, col in ((limb_g - 0.30, (0.00, 0.00, 0.00, 1)),     # disc: clear
                 (limb_g - 0.06, (0.05, 0.10, 0.22, 1)),     # haze toward the limb
                 (limb_g,        (0.55, 0.80, 1.00, 1)),     # bright band on the limb
                 (limb_g + 0.05, (0.10, 0.25, 0.75, 1)),     # deep blue just above
                 (limb_g + 0.14, (0.00, 0.00, 0.00, 1))):    # gone into space
    e = els.new(pos); e.color = col
n.links.new(g.outputs[0], ramp.inputs["Fac"])
alit, _ = lambert(n, gain=1.0, wrap=0.15)
sc_ = n.nodes.new("ShaderNodeVectorMath"); sc_.operation = "SCALE"
n.links.new(ramp.outputs["Color"], sc_.inputs[0]); n.links.new(alit, sc_.inputs["Scale"])
emi = n.nodes.new("ShaderNodeEmission")
emi.inputs["Strength"].default_value = 2.2
n.links.new(sc_.outputs[0], emi.inputs["Color"])
tr = n.nodes.new("ShaderNodeBsdfTransparent")
add = n.nodes.new("ShaderNodeAddShader")
n.links.new(emi.outputs[0], add.inputs[0]); n.links.new(tr.outputs[0], add.inputs[1])
n.links.new(add.outputs[0], ao.inputs["Surface"])
if hasattr(am, "surface_render_method"):
    am.surface_render_method = "BLENDED"
atmo = sphere("Atmosphere", ATMO_R, am)
for ob in (earth, clouds, atmo):  # the far globe must not shadow the robot
    ob.visible_shadow = False

# Orient the globe: the point (--earth-lat, --earth-lon) sits on the visible
# part of the disc next to the limb, north toward screen up. Default: the
# tropical Atlantic / West Africa (owner: not over a pole; dark ocean + land).
LAT = math.radians(float(arg("--earth-lat", "12")))
LON = math.radians(float(arg("--earth-lon", "-25")))
to_cam = (cam_final - earth_c).normalized()
# aim point: 70% of the way from the sub-camera point toward the limb point
limb_pt = (cam_final + limb_dir * (cam_final - earth_c).length * math.cos(alpha) - earth_c).normalized()
aim = to_cam.slerp(limb_pt, 0.7) if hasattr(to_cam, "slerp") else (to_cam + limb_pt).normalized()
n_w = (up - up.dot(aim) * aim).normalized()
e_w = n_w.cross(aim)
p0 = Vector((math.cos(LAT) * math.cos(LON), math.cos(LAT) * math.sin(LON), math.sin(LAT)))
east0 = Vector((-math.sin(LON), math.cos(LON), 0))
north0 = p0.cross(east0)
Rw = Matrix((aim, e_w, n_w)).transposed()
Rl = Matrix((p0, east0, north0)).transposed()
R0 = Rw @ Rl.transposed()
for ob, drift in ((earth, 0.0), (clouds, 0.5), (atmo, 0.0)):
    ob.rotation_mode = "QUATERNION"
    for t, spin in ((1, 0.0), (DURATION, math.radians(0.8 + drift))):
        ob.rotation_quaternion = (R0 @ Matrix.Rotation(spin, 3, "Z")).to_quaternion()
        ob.keyframe_insert("rotation_quaternion", frame=t)

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
