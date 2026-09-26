#!/usr/bin/env python3
"""Extract a BeamNG vehicle's exterior mesh from a mod zip as an STL.

Resolves one vehicle configuration (.pc) through the jbeam part tree to the
exact set of meshes (flexbodies) that configuration uses, reads them from the
vehicle's Collada (.dae) file, adds tyres as cylinders at the hub positions
(tyre meshes live in the base game, not in vehicle mods), converts BeamNG's
frame (forward = -Y, up = +Z) to the wind tunnel's (flow along +X, nose
towards -X, up = +Z) and writes a binary STL in metres.

The result is raw "game" geometry: open panels with gaps. The wind tunnel
handles that directly (its solid mask flood-fills the air from outside);
build/wt_prep can optionally shrink-wrap it into one smooth closed hull.

usage:
  beamng_to_stl.py MOD.zip [--vehicle NAME] [--config NAME] [--list] -o OUT.stl
"""

import argparse
import math
import re
import struct
import sys
import xml.etree.ElementTree as ET
import zipfile
from pathlib import PurePosixPath

import numpy as np

# --------------------------------------------------------------------------
# jbeam: JSON-ish with comments, optional commas and trailing commas.


def parse_jbeam(text):
    pos = 0
    n = len(text)

    def skip():
        nonlocal pos
        while pos < n:
            c = text[pos]
            if c in " \t\r\n,":
                pos += 1
            elif text.startswith("//", pos):
                end = text.find("\n", pos)
                pos = n if end < 0 else end + 1
            elif text.startswith("/*", pos):
                end = text.find("*/", pos + 2)
                pos = n if end < 0 else end + 2
            else:
                break

    number = re.compile(r"-?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?")

    def value():
        nonlocal pos
        skip()
        c = text[pos]
        if c == "{":
            pos += 1
            obj = {}
            while True:
                skip()
                if text[pos] == "}":
                    pos += 1
                    return obj
                key = value()
                skip()
                if text[pos] == ":":
                    pos += 1
                obj[key] = value()
        if c == "[":
            pos += 1
            arr = []
            while True:
                skip()
                if text[pos] == "]":
                    pos += 1
                    return arr
                arr.append(value())
        if c == '"':
            end = pos + 1
            out = []
            while text[end] != '"':
                if text[end] == "\\":
                    end += 1
                out.append(text[end])
                end += 1
            pos = end + 1
            return "".join(out)
        for word, val in (("true", True), ("false", False), ("null", None)):
            if text.startswith(word, pos):
                pos += len(word)
                return val
        m = number.match(text, pos)
        if m:
            pos = m.end()
            s = m.group(0)
            return float(s) if any(ch in s for ch in ".eE") else int(s)
        raise ValueError(f"jbeam parse error at {pos}: {text[pos:pos + 40]!r}")

    return value()


# --------------------------------------------------------------------------
# Collada: node name -> world-space triangles.

NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}


def node_matrix(node):
    m = np.eye(4)
    for child in node:
        tag = child.tag.split("}")[1]
        vals = [float(v) for v in child.text.split()] if child.text else []
        if tag == "matrix":
            m = m @ np.array(vals).reshape(4, 4)
        elif tag == "translate":
            t = np.eye(4)
            t[:3, 3] = vals
            m = m @ t
        elif tag == "scale":
            m = m @ np.diag(vals + [1.0])
        elif tag == "rotate":
            x, y, z, ang = vals
            a = math.radians(ang)
            axis = np.array([x, y, z]) / (np.linalg.norm([x, y, z]) or 1.0)
            k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
            r = np.eye(3) + math.sin(a) * k + (1 - math.cos(a)) * (k @ k)
            t = np.eye(4)
            t[:3, :3] = r
            m = m @ t
    return m


def geometry_triangles(geom):
    mesh = geom.find("c:mesh", NS)
    if mesh is None:
        return np.zeros((0, 3, 3))
    sources = {}
    for src in mesh.findall("c:source", NS):
        arr = src.find("c:float_array", NS)
        acc = src.find("c:technique_common/c:accessor", NS)
        stride = int(acc.get("stride", "3")) if acc is not None else 3
        data = np.array(arr.text.split(), dtype=np.float64) if arr is not None and arr.text else np.zeros(0)
        sources["#" + src.get("id")] = data.reshape(-1, stride)[:, :3]
    verts = mesh.find("c:vertices", NS)
    vert_src = {}
    if verts is not None:
        for inp in verts.findall("c:input", NS):
            if inp.get("semantic") == "POSITION":
                vert_src["#" + verts.get("id")] = sources[inp.get("source")]
    tris = []
    for prim in list(mesh):
        tag = prim.tag.split("}")[1]
        if tag not in ("triangles", "polylist", "polygons"):
            continue
        inputs = prim.findall("c:input", NS)
        stride = max(int(i.get("offset", "0")) for i in inputs) + 1
        vin = next(i for i in inputs if i.get("semantic") == "VERTEX")
        positions = vert_src.get(vin.get("source"))
        if positions is None:
            positions = sources[vin.get("source")]
        off = int(vin.get("offset", "0"))
        if tag == "polygons":
            polys = [np.array(p.text.split(), dtype=np.int64) for p in prim.findall("c:p", NS)]
            counts = [len(p) // stride for p in polys]
            idx = np.concatenate(polys) if polys else np.zeros(0, dtype=np.int64)
        else:
            p = prim.find("c:p", NS)
            if p is None or not p.text:
                continue
            idx = np.array(p.text.split(), dtype=np.int64)
            if tag == "polylist":
                counts = [int(v) for v in prim.find("c:vcount", NS).text.split()]
            else:
                counts = [3] * (len(idx) // (3 * stride))
        vidx = idx[off::stride]
        start = 0
        fan = []
        for cnt in counts:  # fan-triangulate polygons
            for k in range(1, cnt - 1):
                fan.append((vidx[start], vidx[start + k], vidx[start + k + 1]))
            start += cnt
        if fan:
            tris.append(positions[np.array(fan)])
    return np.concatenate(tris) if tris else np.zeros((0, 3, 3))


def load_dae(data):
    root = ET.fromstring(data)
    unit = root.find("c:asset/c:unit", NS)
    scale = float(unit.get("meter", "1")) if unit is not None else 1.0
    geoms = {g.get("id"): g for g in root.findall(".//c:library_geometries/c:geometry", NS)}
    cache = {}
    meshes = {}

    def walk(node, parent):
        world = parent @ node_matrix(node)
        for inst in node.findall("c:instance_geometry", NS):
            gid = inst.get("url")[1:]
            if gid not in cache:
                cache[gid] = geometry_triangles(geoms[gid])
            t = cache[gid]
            if len(t):
                h = np.concatenate([t.reshape(-1, 3), np.ones((t.shape[0] * 3, 1))], axis=1)
                w = (h @ world.T)[:, :3].reshape(-1, 3, 3) * scale
                name = node.get("name") or node.get("id")
                meshes.setdefault(name, []).append(w)
        for child in node.findall("c:node", NS):
            walk(child, world)

    for scene in root.findall(".//c:library_visual_scenes/c:visual_scene", NS):
        for node in scene.findall("c:node", NS):
            walk(node, np.eye(4))
    return {k: np.concatenate(v) for k, v in meshes.items()}


# --------------------------------------------------------------------------
# Part tree resolution.


def slot_rows(part):
    """Yield (slotName, defaultPart) for both slot table formats."""
    if "slots2" in part:
        rows = part["slots2"]
        header = rows[0]
        ni, di = header.index("name"), header.index("default")
        for r in rows[1:]:
            if isinstance(r, list):
                yield r[ni], r[di]
    elif "slots" in part:
        for r in part["slots"][1:]:
            if isinstance(r, list) and len(r) >= 2:
                yield r[0], r[1]


def flexbody_meshes(part):
    rows = part.get("flexbodies", [])
    for r in rows[1:] if rows else []:
        if isinstance(r, list) and r and isinstance(r[0], str):
            yield r[0]


def slot_offsets(part):
    """slotName -> nodeOffset dict (from a slot row's options), if any."""
    out = {}
    if "slots2" in part:
        rows = part["slots2"]
        ni = rows[0].index("name")
        for r in rows[1:]:
            if isinstance(r, list) and isinstance(r[-1], dict) and "nodeOffset" in r[-1]:
                out[r[ni]] = r[-1]["nodeOffset"]
    elif "slots" in part:
        for r in part["slots"][1:]:
            if isinstance(r, list) and isinstance(r[-1], dict) and "nodeOffset" in r[-1]:
                out[r[0]] = r[-1]["nodeOffset"]
    return out


def resolve_parts(parts, config, main):
    """Active part names, plus the nodeOffset each part was mounted with."""
    chosen = config.get("parts", {})
    active, offsets, queue = [], {}, [(main, None)]
    seen = set()
    while queue:
        name, offset = queue.pop()
        if not name or name in seen or name not in parts:
            continue
        seen.add(name)
        active.append(name)
        if offset is not None:
            offsets[name] = offset
        slot_off = slot_offsets(parts[name])
        for slot, default in slot_rows(parts[name]):
            queue.append((chosen.get(slot, default), slot_off.get(slot, offset)))
    return active, offsets


def eval_number(value, variables):
    """Evaluate a jbeam number or a simple '$=' expression (vars, arithmetic, case(x == nil, a, b))."""
    if isinstance(value, (int, float)):
        return float(value)
    if not isinstance(value, str):
        return 0.0
    expr = value[2:] if value.startswith("$=") else value
    m = re.fullmatch(r"\s*case\((.*?)==\s*nil\s*,(.*),(.*)\)\s*", expr)
    if m:
        var = m.group(1).strip()
        return eval_number(m.group(3) if var in variables else m.group(2), variables)
    expr = re.sub(r"\$(\w+)", lambda v: repr(variables.get(v.group(1), 0.0)), expr)
    if not re.fullmatch(r"[0-9eE.+\-*/() ]+", expr):
        return 0.0
    return float(eval(expr, {"__builtins__": {}}))  # arithmetic only, checked above


def part_variables(parts, active, config):
    """Defaults from the active parts' variable tables, overridden by the config."""
    variables = {}
    for p in active:
        rows = parts[p].get("variables", [])
        for r in rows[1:] if rows else []:
            if isinstance(r, list) and len(r) > 4 and isinstance(r[0], str):
                variables[r[0].lstrip("$")] = float(r[4]) if isinstance(r[4], (int, float)) else 0.0
    for k, v in config.get("vars", {}).items():
        variables[k.lstrip("$")] = float(v)
    return variables


def wheel_hubs(parts, active, offsets, variables):
    """Tyre centre per hub group (e.g. wheelhub_FL). BeamNG puts the wheel's
    centre plane at the hub's outer node plus the part's pressureWheels
    "wheelOffset" (negative = inboard); nodes get the part's mount nodeOffset
    (x mirrored for right-side nodes)."""
    hubs = {}
    for p in active:
        part = parts[p]
        rows = part.get("nodes", [])
        off = offsets.get(p, {})
        ox, oy, oz = (eval_number(off.get(k, 0), variables) for k in ("x", "y", "z"))
        wheel_offset = 0.0
        for r in part.get("pressureWheels", []):
            if isinstance(r, dict) and "wheelOffset" in r:
                wheel_offset = eval_number(r["wheelOffset"], variables)
        group = ""
        nodes = {}
        for r in rows[1:] if rows else []:
            if isinstance(r, dict):
                g = r.get("group", group)
                group = g if isinstance(g, str) else (g[0] if g else "")
            elif isinstance(r, list) and len(r) >= 4 and isinstance(r[1], (int, float)):
                if group.startswith("wheelhub_"):
                    x, y, z = float(r[1]), float(r[2]), float(r[3])
                    x += ox if x >= 0 else -ox
                    nodes.setdefault(group, []).append(np.array((x, y + oy, z + oz)))
        for g, pts in nodes.items():
            outer = max(pts, key=lambda q: abs(q[0]))
            centre = np.mean(np.array(pts), axis=0)
            centre[0] = outer[0] + np.sign(outer[0]) * wheel_offset
            hubs[g] = centre
    return hubs


# --------------------------------------------------------------------------
# Tyres.


def cylinder(centre, axis, radius, width, segments=48):
    axis = axis / np.linalg.norm(axis)
    ref = np.array([0.0, 0.0, 1.0]) if abs(axis[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(axis, ref)
    u /= np.linalg.norm(u)
    v = np.cross(axis, u)
    a0, a1 = centre - axis * width / 2, centre + axis * width / 2
    ring = [np.cos(2 * math.pi * i / segments) * u + np.sin(2 * math.pi * i / segments) * v for i in range(segments)]
    tris = []
    for i in range(segments):
        p, q = ring[i] * radius, ring[(i + 1) % segments] * radius
        tris += [(a0 + p, a1 + p, a1 + q), (a0 + p, a1 + q, a0 + q), (a0, a0 + q, a0 + p), (a1, a1 + p, a1 + q)]
    return np.array(tris)


def tyre_size(config, axle):
    """(radius, width) in metres from a tyre part name like tire_F_175_70_13_standard."""
    for slot, part in config.get("parts", {}).items():
        if slot.lower().startswith(f"tire_{axle.lower()}") and part:
            m = re.search(r"(\d{3})_(\d{2})_(\d{2})", part)
            if m:
                w, aspect, rim = (int(g) for g in m.groups())
                return rim * 0.0254 / 2 + w / 1000 * aspect / 100, w / 1000
    return 0.29, 0.18


# --------------------------------------------------------------------------


def write_stl(path, tris):
    tris = tris.astype(np.float32)
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.divide(n, ln, out=np.zeros_like(n), where=ln > 0)
    rec = np.zeros(len(tris), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["n"], rec["v"] = n, tris
    with open(path, "wb") as f:
        f.write(b"beamng_to_stl".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        f.write(rec.tobytes())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("zip")
    ap.add_argument("--vehicle", help="vehicle folder name (default: the one with a .dae)")
    ap.add_argument("--config", help="configuration (.pc) name; default: the first")
    ap.add_argument("--list", action="store_true", help="list vehicles and configurations, then exit")
    ap.add_argument("--no-tyres", action="store_true")
    ap.add_argument("-o", "--output")
    args = ap.parse_args()

    z = zipfile.ZipFile(args.zip)
    names = z.namelist()
    vehicles = sorted({PurePosixPath(n).parts[1] for n in names if n.startswith("vehicles/") and n.endswith(".dae")
                       and PurePosixPath(n).parts[1] != "common"})
    vehicle = args.vehicle or (vehicles[0] if vehicles else None)
    if not vehicle:
        sys.exit("no vehicle with a .dae found in the zip")
    configs = sorted(PurePosixPath(n).stem for n in names if n.startswith(f"vehicles/{vehicle}/") and n.endswith(".pc"))
    if args.list:
        print("vehicles:", ", ".join(vehicles))
        print(f"configurations of {vehicle}:", ", ".join(configs))
        return
    if not args.output:
        sys.exit("-o OUT.stl is required")
    config_name = args.config or configs[0]
    config = parse_jbeam(z.read(f"vehicles/{vehicle}/{config_name}.pc").decode("utf-8", "replace"))

    parts = {}
    for n in names:
        if n.endswith(".jbeam") and (n.startswith(f"vehicles/{vehicle}/") or n.startswith("vehicles/common/")):
            try:
                doc = parse_jbeam(z.read(n).decode("utf-8", "replace"))
            except (ValueError, IndexError) as e:
                print(f"  skipping {n}: {e}", file=sys.stderr)
                continue
            if isinstance(doc, dict):
                parts.update({k: v for k, v in doc.items() if isinstance(v, dict)})
    main_part = config.get("mainPartName", vehicle)
    active, offsets = resolve_parts(parts, config, main_part)
    variables = part_variables(parts, active, config)
    wanted = [m for p in active for m in flexbody_meshes(parts[p])]

    # Wheel, rim and hubcap meshes (vehicles/common/...) are placed at the hubs
    # by the physics at runtime and sit at the origin in the file; the tyre
    # cylinders added below stand in for them.
    meshes = {}
    for n in names:
        if n.endswith(".dae") and n.startswith(f"vehicles/{vehicle}/"):
            meshes.update(load_dae(z.read(n)))

    used = [m for m in dict.fromkeys(wanted) if m in meshes]
    missing = [m for m in dict.fromkeys(wanted) if m not in meshes]
    tris = [meshes[m] for m in used]

    if not args.no_tyres:
        hubs = wheel_hubs(parts, active, offsets, variables)
        for group, centre in sorted(hubs.items()):
            axle = "F" if group.upper().endswith(("FL", "FR")) else "R"
            radius, width = tyre_size(config, axle)
            tris.append(cylinder(centre, np.array([1.0, 0.0, 0.0]), radius, width))
            print(f"  tyre {group[9:]}: centre ({centre[0]:+.3f}, {centre[1]:+.3f}, {centre[2]:.3f}) "
                  f"r={radius:.3f} w={width:.3f}")
        if not hubs:
            print("  no wheel hub nodes found; tyres omitted", file=sys.stderr)

    t = np.concatenate(tris)
    # BeamNG: forward -Y, up +Z  ->  tunnel: nose towards -X, up +Z.
    t = np.stack([t[..., 1], -t[..., 0], t[..., 2]], axis=-1)
    write_stl(args.output, t)
    lo, hi = t.reshape(-1, 3).min(0), t.reshape(-1, 3).max(0)
    print(f"{vehicle} / {config_name}: {len(active)} parts, {len(used)} meshes, {len(t)} triangles")
    print(f"size {hi[0] - lo[0]:.3f} x {hi[1] - lo[1]:.3f} x {hi[2] - lo[2]:.3f} m (L x W x H)")
    if missing:
        print(f"{len(missing)} flexbody meshes not in this mod (base-game assets): {', '.join(missing[:12])}"
              + (" ..." if len(missing) > 12 else ""))
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
