"""Convert the original Microduck GLB into MuJoCo meshes, skins and joint offsets."""
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np

# glTF is Y-up. Microduck faces +X, with its left side along glTF -Z.
AXES = np.array(((1, 0, 0), (0, 0, -1), (0, 1, 0)))
BODIES = {
    "root": ("than", None),
    "left_hip_yaw": ("hong_xoay_L", "root"),
    "left_hip_roll": ("hong_doc_L", "left_hip_yaw"),
    "left_hip_pitch": ("dui_L", "left_hip_roll"),
    "left_knee": ("ong_L", "left_hip_pitch"),
    "left_ankle": ("chan_L", "left_knee"),
    "right_hip_yaw": ("hong_xoay_R", "root"),
    "right_hip_roll": ("hong_doc_R", "right_hip_yaw"),
    "right_hip_pitch": ("dui_R", "right_hip_roll"),
    "right_knee": ("ong_R", "right_hip_pitch"),
    "right_ankle": ("chan_R", "right_knee"),
    "neck_pitch": ("co", "root"),
    "head_pitch": ("dau_gat", "neck_pitch"),
    "head_yaw": ("dau_xoay", "head_pitch"),
    "head_roll": ("dau", "head_yaw"),
    "mouth": ("mo", "head_roll"),
}


def vector(values):
    return " ".join(f"{value:.8g}" for value in values)


def export(source, target):
    raw = source.read_bytes()
    magic, version, _ = struct.unpack_from("<4sII", raw)
    if (magic, version) != (b"glTF", 2):
        raise ValueError("Cần file Microduck GLB phiên bản 2.")
    json_size = struct.unpack_from("<I", raw, 12)[0]
    doc = json.loads(raw[20:20 + json_size])
    buffer = memoryview(raw)[28 + json_size:]
    target.mkdir(parents=True, exist_ok=True)

    def accessor(index):
        item = doc["accessors"][index]
        view = doc["bufferViews"][item["bufferView"]]
        dtype = np.dtype({5121: "u1", 5123: "<u2", 5125: "<u4", 5126: "<f4"}[item["componentType"]])
        width = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT4": 16}[item["type"]]
        return np.ndarray(
            (item["count"], width), dtype=dtype, buffer=buffer,
            offset=view.get("byteOffset", 0) + item.get("byteOffset", 0),
            strides=(view.get("byteStride", width * dtype.itemsize), dtype.itemsize),
        )

    skin = doc["skins"][0]
    bones = {
        j: doc["nodes"][node]["name"].rsplit("_", 1)[0]
        for j, node in enumerate(skin["joints"]) if node != skin["skeleton"]
    }
    if set(bones.values()) != BODIES.keys():
        raise ValueError("Bộ khớp trong GLB khác với Microduck.")
    bind = np.linalg.inv(accessor(skin["inverseBindMatrices"]).reshape(-1, 4, 4).transpose(0, 2, 1))
    positions = {name: AXES @ bind[j, :3, 3] for j, name in bones.items()}
    groups = defaultdict(list)
    mixed = []
    all_vertices = []
    sole_bounds = {}
    for mesh in doc["meshes"]:
        for primitive in mesh["primitives"]:
            attrs = primitive["attributes"]
            vertices = accessor(attrs["POSITION"]) @ AXES.T
            normals = accessor(attrs["NORMAL"]) @ AXES.T
            uv = accessor(attrs["TEXCOORD_0"])
            faces = accessor(primitive["indices"]).reshape(-1, 3)
            joints, weights = accessor(attrs["JOINTS_0"]), accessor(attrs["WEIGHTS_0"])
            influencing = np.unique(joints[weights > 1e-6])
            material = primitive["material"]
            all_vertices.append(vertices)
            if len(influencing) == 1:
                bone = bones[influencing[0]]
                local = vertices - positions[bone]
                groups[BODIES[bone][0], material].append((local, normals, uv, faces))
                if bone.endswith("_ankle") and material == 10:
                    sole_bounds[BODIES[bone][0]] = (local.min(axis=0).tolist(), local.max(axis=0).tolist())
            else:
                mixed.append((vertices, uv, faces, material, joints, weights, influencing))

    vertices = np.concatenate(all_vertices)
    ground = float(vertices[:, 2].min())
    geometry = {
        "source_sha256": hashlib.sha256(raw).hexdigest(),
        "height_m": float(vertices[:, 2].max() - ground),
        "offsets": {
            body: (positions[bone] - (positions[parent] if parent else (0, 0, ground))).tolist()
            for bone, (body, parent) in BODIES.items()
        },
        "sole_bounds": sole_bounds,
    }
    (target / "khop.json").write_text(json.dumps(geometry, indent=2), encoding="utf-8")

    visual = ET.Element("microduck")
    asset = ET.SubElement(visual, "asset")
    attachments = ET.SubElement(visual, "attachments")
    saved_images = set()
    for i, material in enumerate(doc["materials"]):
        pbr = material["pbrMetallicRoughness"]
        diffuse = pbr.get("baseColorTexture")
        settings = {"specular": ".30", "shininess": ".37"}
        if diffuse is not None:
            index = doc["textures"][diffuse["index"]]["source"]
            filename = f"color_{index}.png"
            tex = f"micro_tex_{index}"
            if index not in saved_images:
                view = doc["bufferViews"][doc["images"][index]["bufferView"]]
                start = view.get("byteOffset", 0)
                (target / filename).write_bytes(buffer[start:start + view["byteLength"]])
                ET.SubElement(asset, "texture", name=tex, type="2d", file=filename)
                saved_images.add(index)
            settings["texture"] = tex
        if material["name"] == "Lente_-_nero":
            settings.update(rgba=".35 .35 .35 1", specular=".08", shininess=".18")
        if "baseColorFactor" in pbr:
            settings["rgba"] = vector(pbr["baseColorFactor"])
        ET.SubElement(asset, "material", name=f"micro_mat_{i}", **settings)

    for (body, material), parts in sorted(groups.items()):
        name = f"micro_{body}_{material}"
        offset = 0
        faces = []
        for vertices, _, _, indices in parts:
            faces.append(indices.astype("<i4") + offset)
            offset += len(vertices)
        arrays = [np.concatenate(column).astype("<f4") for column in zip(*parts)][:3]
        indices = np.concatenate(faces).astype("<i4")
        payload = struct.pack("<4i", *(len(a) for a in (*arrays, indices)))
        payload += b"".join(a.tobytes() for a in (*arrays, indices))
        filename = f"{name}.msh"
        (target / filename).write_bytes(payload)
        ET.SubElement(asset, "mesh", name=name, file=filename, maxhullvert="32", inertia="shell")
        ET.SubElement(attachments, "part", body=body, mesh=name, material=f"micro_mat_{material}")

    for i, (vertices, uv, faces, material, joints, weights, influencing) in enumerate(mixed):
        skin_asset = ET.SubElement(
            asset, "skin", name=f"cap_{i:02}", material=f"micro_mat_{material}", group="2",
            vertex=vector((vertices - (0, 0, ground)).ravel()),
            texcoord=vector((uv * (1, -1) + (0, 1)).ravel()),
            face=vector(faces.ravel()),
        )
        for joint in influencing:
            bone = bones[joint]
            weight = np.sum(np.where(joints == joint, weights, 0), axis=1)
            ids = np.flatnonzero(weight > 1e-6)
            ET.SubElement(
                skin_asset, "bone", body=BODIES[bone][0], bindquat="1 0 0 0",
                bindpos=vector(positions[bone] - (0, 0, ground)),
                vertid=vector(ids), vertweight=vector(weight[ids]),
            )
    ET.indent(visual)
    ET.ElementTree(visual).write(target / "visual.xml", encoding="utf-8")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("Cách dùng: python nhap_glb.py đường_dẫn_tới_microduck.glb")
    export(Path(sys.argv[1]), Path(__file__).resolve().parent / "microduck")

