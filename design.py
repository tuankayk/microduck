"""Build the Microduck scene with the original GLB skeleton and collision proxies."""
import json
import math
from pathlib import Path
from random import Random
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
TERRAINS = ("phang", "tron", "doc", "nhap_nho")
GEOMETRY = json.loads((HERE / "microduck" / "khop.json").read_text(encoding="utf-8"))
OFFSETS = GEOMETRY["offsets"]
REACH = math.hypot(OFFSETS["ong_L"][0], OFFSETS["ong_L"][2])
SHANK = math.hypot(OFFSETS["chan_L"][0], OFFSETS["chan_L"][2])
THIGH_BIND = math.atan2(-OFFSETS["ong_L"][0], -OFFSETS["ong_L"][2])
SHANK_BIND = math.atan2(-OFFSETS["chan_L"][0], -OFFSETS["chan_L"][2])


def vec(values):
    return " ".join(f"{value:.8g}" for value in values)


def add_microduck(root, asset_dir):
    visual = ET.parse(HERE / "microduck" / "visual.xml").getroot()
    asset = root.find("asset")
    for item in visual.find("asset"):
        if "file" in item.attrib:
            item.set("file", (asset_dir / item.get("file")).as_posix())
        asset.append(item)
    bodies = {item.get("name"): item for item in root.iter("body")}
    for part in visual.find("attachments"):
        ET.SubElement(
            bodies[part.get("body")], "geom", type="mesh", mesh=part.get("mesh"),
            material=part.get("material"), mass="0", contype="0", conaffinity="0", group="2",
        )


def add_terrain(world, asset, terrain, rng):
    colors = {
        "phang": (".40 .47 .49", ".44 .50 .51", ".08"),
        "tron": (".32 .49 .56", ".38 .56 .61", ".42"),
        "doc": (".43 .42 .36", ".48 .48 .42", ".11"),
        "nhap_nho": (".38 .43 .34", ".45 .48 .39", ".06"),
    }
    color1, color2, shine = colors[terrain]
    ET.SubElement(asset, "texture", name="tile", type="2d", builtin="checker",
                  rgb1=color1, rgb2=color2, width="64", height="64")
    ET.SubElement(asset, "material", name="floor", texture="tile",
                  texrepeat="5 5", texuniform="true", specular=shine)
    grade = rng.uniform(.025, .052) if terrain == "doc" else 0.
    mu = rng.uniform(.24, .36) if terrain == "tron" else rng.uniform(.68, .94)
    ground = dict(name="floor", material="floor", priority="2", contype="1", conaffinity="2",
                  friction=vec((mu, .005, .0002)))
    if terrain == "nhap_nho":
        nx, ny = 241, 65
        amplitude = rng.uniform(.0018, .0035)
        phase = rng.uniform(0, 2 * math.pi)
        xs = [i * .05 - 6 for i in range(nx)]
        ys = [j * .0625 - 2 for j in range(ny)]
        values = [.5 + .26 * math.sin(15*x + phase) * math.cos(5*y) + .20 * math.sin(23*x + 3*y)
                  for y in ys for x in xs]
        ET.SubElement(asset, "hfield", name="track", nrow=str(ny), ncol=str(nx),
                      size=vec((6, 2, 2 * amplitude, .1)), elevation=vec(values))
        ET.SubElement(world, "geom", type="hfield", hfield="track", pos=vec((0, 0, -amplitude)), **ground)
    else:
        ET.SubElement(world, "geom", type="plane", size="0 0 .1", euler=vec((0, -grade, 0)), **ground)


def build(terrain="phang", seed=0, *, with_visual=True, portable=False):
    root = ET.Element("mujoco", model="Microduck")
    ET.SubElement(root, "compiler", angle="radian", autolimits="true")
    ET.SubElement(root, "option", timestep=".002", integrator="implicitfast", iterations="48")
    defaults = ET.SubElement(root, "default")
    ET.SubElement(defaults, "joint", damping=".017", armature=".00007")
    ET.SubElement(defaults, "geom", condim="3", solref=".008 1")
    ET.SubElement(root, "statistic", center=".02 0 .13", extent=".55")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth="1400", offheight="1000")
    ET.SubElement(visual, "headlight", ambient=".48 .48 .48", diffuse=".58 .58 .58")
    asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "texture", name="sky", type="skybox", builtin="gradient",
                  rgb1=".24 .34 .43", rgb2=".68 .75 .78", width="64", height="384")
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", pos=".25 -1 3", dir="0 .25 -1", diffuse=".65 .65 .65")
    add_terrain(world, asset, terrain, Random(seed))
    actuators = ET.SubElement(root, "actuator")

    def body(parent, name, mass, center, half_size):
        part = ET.SubElement(parent, "body", name=name, pos=vec(OFFSETS[name]))
        # The GLB supplies shape and pivots. Inertias approximate the assembled components.
        inertia = [mass / 3 * sum(half_size[j] ** 2 for j in range(3) if j != i) for i in range(3)]
        ET.SubElement(part, "inertial", pos=vec(center), mass=str(mass), diaginertia=vec(inertia))
        return part

    def hinge(part, name, axis, limits, kp, kv, torque):
        ET.SubElement(part, "joint", name=name, axis=vec(axis), range=vec(limits))
        ET.SubElement(actuators, "position", name=name, joint=name, kp=str(kp), kv=str(kv),
                      forcerange=vec((-torque, torque)), ctrlrange=vec(limits))

    def collider(parent, name, center, half_size):
        ET.SubElement(parent, "geom", type="box", name=name, pos=vec(center), size=vec(half_size),
                      mass="0", contype="2", conaffinity="1", group="3", rgba="0 0 0 0")

    x, y, z = (1, 0, 0), (0, 1, 0), (0, 0, 1)
    base = body(world, "than", .30, (-.006, 0, -.007), (.034, .029, .033))
    ET.SubElement(base, "freejoint", name="goc")
    collider(base, "vo_than", (.004, 0, -.014), (.037, .027, .036))
    neck = body(base, "co", .027, (0, -.0145, .025), (.013, .016, .026))
    hinge(neck, "nghieng_co", y, (-.31, .31), 2.3, .09, .60)
    pitch = body(neck, "dau_gat", .016, (0, -.007, .008), (.013, .012, .014))
    hinge(pitch, "gat_dau", y, (-.38, .28), 2.4, .08, .52)
    yaw = body(pitch, "dau_xoay", .019, (-.008, 0, .005), (.016, .017, .015))
    hinge(yaw, "xoay_co", z, (-.63, .63), 2.1, .075, .55)
    head = body(yaw, "dau", .078, (.022, 0, .004), (.060, .043, .025))
    hinge(head, "nghieng_dau", x, (-.23, .23), 1.8, .065, .48)
    collider(head, "kinh_dau", (.027, 0, .011), (.060, .041, .024))
    jaw = body(head, "mo", .018, (.031, 0, -.007), (.029, .040, .009))
    hinge(jaw, "mo_mo", y, (-.04, .57), 1.3, .04, .35)

    for side, sign in (("L", 1), ("R", -1)):
        hip_yaw = body(base, "hong_xoay_" + side, .033, (.007, 0, -.012), (.014, .013, .016))
        hinge(hip_yaw, "xoay_hong_" + side, z, (-.30, .30), 11, .23, 1.4)
        hip_roll = body(hip_yaw, "hong_doc_" + side, .029, (-.008, sign * .010, 0), (.016, .017, .011))
        hinge(hip_roll, "hong_nghieng_" + side, x, (-.38, .38), 17, .28, 1.6)
        thigh = body(hip_roll, "dui_" + side, .046, (-.014, sign * .014, -.012), (.023, .015, .020))
        hinge(thigh, "hong_" + side, y, (-1.15, .57), 15, .25, 1.6)
        shin = body(thigh, "ong_" + side, .039, (0, sign * .010, -.020), (.012, .015, .027))
        hinge(shin, "goi_" + side, y, (-.99, .77), 15, .23, 1.6)
        foot = body(shin, "chan_" + side, .038, (.007, -sign * .015, -.013), (.026, .019, .017))
        hinge(foot, "co_chan_" + side, y, (-.63, .73), 12, .22, 1.4)
        low, high = GEOMETRY["sole_bounds"]["chan_" + side]
        center = [(a + b) / 2 for a, b in zip(low, high)]
        half = [(b - a) / 2 for a, b in zip(low, high)]
        half[0] -= .0025
        half[1] -= .002
        collider(foot, "de_" + side, center, half)

    ET.SubElement(ET.SubElement(root, "keyframe"), "key", name="dung")
    if with_visual:
        add_microduck(root, Path("microduck") if portable else HERE / "microduck")
    return ET.tostring(root, encoding="unicode")


if __name__ == "__main__":
    path = HERE / "Canard_D1.xml"
    scene = ET.fromstring(build(portable=True))
    ET.indent(scene)
    ET.ElementTree(scene).write(path, encoding="utf-8")

