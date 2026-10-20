"""SimSpec -> layout -> .wbt text + controller config.

Built-in assets (E-puck, Mavic2Pro, RectangleArena, WoodenBox, CardboardBox, backgrounds) are referenced through
`AssetResolver`-verified EXTERNPROTO lines. Everything else (custom wheeled robot, arm, shelves, line track,
markers) is built from core Webots nodes only (Robot, Solid, Shape, HingeJoint, BallJoint, RotationalMotor,
PositionSensor, DistanceSensor, Camera, Box, Cylinder, Sphere).
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

from app.pipeline.spec import ARM, SimSpec
from app.webots_assets import Asset

EPUCK_RADIUS = 0.037
EPUCK_AXLE = 0.052


# ------------------------------------------------------------------------------------------------ layout
@dataclass
class Layout:
    starts: list[tuple[float, float, float]] = field(default_factory=list)       # x, y, heading
    targets: list[list[tuple[float, float]]] = field(default_factory=list)       # per robot
    obstacles: list[dict] = field(default_factory=list)                          # {x,y,sx,sy,h,kind}
    markers: list[dict] = field(default_factory=list)                            # {x,y,r,color,name}
    line: list[tuple[float, float]] = field(default_factory=list)                # track polyline (closed)
    track_center: tuple[float, float] = (0.0, 0.0)
    notes: list[str] = field(default_factory=list)


def _fmt(v: float) -> str:
    return f"{v:.4f}".rstrip("0").rstrip(".") if "." in f"{v:.4f}" else f"{v:.4f}"


def robot_radius(spec: SimSpec) -> float:
    r = spec.robot
    if r.kind == "epuck":
        return EPUCK_RADIUS
    return 0.5 * math.hypot(r.body_length, r.body_width)


def contact_radius(spec: SimSpec) -> float:
    """Radius used for the controller's contact accounting (mean half-extent of the body)."""
    r = spec.robot
    return EPUCK_RADIUS if r.kind == "epuck" else (r.body_length + r.body_width) / 4


def look_at(pos: tuple[float, float, float], target: tuple[float, float, float]) -> str:
    """Viewpoint `orientation` axis-angle for ENU worlds (camera looks along +x, up is +z) aiming at `target`."""
    f = [t - p for t, p in zip(target, pos)]
    n = math.sqrt(sum(c * c for c in f))
    x = [c / n for c in f]                                   # camera forward
    z = [0.0, 0.0, 1.0]
    y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]]   # left = up x forward
    ny = math.sqrt(sum(c * c for c in y))
    y = [c / ny for c in y]
    z = [x[1] * y[2] - x[2] * y[1], x[2] * y[0] - x[0] * y[2], x[0] * y[1] - x[1] * y[0]]  # camera up = forward x left
    m = [[x[0], y[0], z[0]], [x[1], y[1], z[1]], [x[2], y[2], z[2]]]       # columns = camera axes
    tr = m[0][0] + m[1][1] + m[2][2]
    ang = math.acos(max(-1.0, min(1.0, (tr - 1) / 2)))
    ax = [m[2][1] - m[1][2], m[0][2] - m[2][0], m[1][0] - m[0][1]]
    na = math.sqrt(sum(c * c for c in ax)) or 1.0
    return f"{ax[0] / na:.5f} {ax[1] / na:.5f} {ax[2] / na:.5f} {ang:.5f}"


def make_track(shape: str, w: float, h: float, n: int = 72) -> list[tuple[float, float]]:
    a, b = w / 2 - 0.6, h / 2 - 0.6
    pts = []
    for i in range(n):
        t = 2 * math.pi * i / n
        c, s = math.cos(t), math.sin(t)
        if shape == "square":                      # superellipse, rounded corners
            pts.append((a * math.copysign(abs(c) ** 0.5, c), b * math.copysign(abs(s) ** 0.5, s)))
        else:
            pts.append((a * c, b * s))
    return pts


def build_layout(spec: SimSpec, seed: str) -> Layout:
    rng = random.Random(seed)
    r, e, b = spec.robot, spec.environment, spec.behavior
    L = Layout()
    w, h = e.width, e.height
    rr = robot_radius(spec)
    n = r.count

    if b.task == "pick_and_place":
        pick = b.pick_position or (0.30, 0.10)
        place = b.place_position or (0.15, -0.35)
        L.markers = [{"x": place[0], "y": place[1], "r": 0.05, "color": (0.1, 0.7, 0.2), "name": "place_zone",
                      "z": ARM["table_h"] + 0.001}]
        L.targets = [[pick, place]]
        return L
    if b.task == "fly_waypoints":
        wps = b.targets or [(2.0, 0.0), (2.0, 2.0), (0.0, 2.0), (0.0, 0.0)]
        L.targets = [list(wps)]
        L.starts = [(0.0, 0.0, 0.0)]
        L.markers = [{"x": p[0], "y": p[1], "r": 0.25, "color": (1.0, 0.5, 0.0), "name": f"waypoint_{i + 1}", "z": 0.001}
                     for i, p in enumerate(wps)]
        return L
    if b.task == "line_follow":
        L.line = make_track(e.track_shape, w, h)
        p0, p1 = L.line[0], L.line[1]
        L.starts = [(p0[0], p0[1], math.atan2(p1[1] - p0[1], p1[0] - p0[0]))]
        L.targets = [[]]
        return L

    # ---------------- wheeled robots in arena / warehouse
    hw, hh = w / 2 - 0.35, h / 2 - 0.35
    starts_given = list(r.start_positions)
    for i in range(n):
        if i < len(starts_given):
            sx, sy = starts_given[i]
        elif len(starts_given) == 1 and n == 1:
            sx, sy = starts_given[0]
        else:
            sx = -hw + 0.1
            sy = (-hh + 0.4 + i * (2 * hh - 0.8) / max(1, n - 1)) if n > 1 else -hh + 0.1
            if e.kind == "warehouse":
                sy = hh - 0.4 - i * 0.7
        L.starts.append((sx, sy, 0.0))

    wp = list(b.targets)
    default_targets: list[tuple[float, float]] = []
    if not wp and b.task == "go_to_targets":
        if e.kind == "warehouse":
            default_targets = [(-hw + 1.0, -hh + 0.5), (hw - 0.5, -hh + 0.5)]       # pick-up, drop-off (bottom aisle)
            L.notes.append("No targets given: pick-up/drop-off zones placed at opposite ends of the bottom aisle.")
        else:
            default_targets = [(hw - 0.1, L.starts[0][1] + 0.0 if n == 1 else 0.0)]
    for i in range(n):
        if wp:
            L.targets.append(wp if (n == 1 or len(wp) < n) else wp[i::n])
        elif default_targets:
            L.targets.append([(tx, ty if e.kind == "warehouse" else (L.starts[i][1] if n > 1 else hh - 0.1))
                              for tx, ty in default_targets])
        else:
            L.targets.append([])
    # headings: face first target
    L.starts = [(sx, sy, math.atan2(L.targets[i][0][1] - sy, L.targets[i][0][0] - sx) if L.targets[i] else 0.0)
                for i, (sx, sy, _) in enumerate(L.starts)]

    keep_out = [(s[0], s[1], 0.5) for s in L.starts] + [(t[0], t[1], 0.45) for ts in L.targets for t in ts]
    # shelves (warehouse): rows with aisles, never on a start/target
    if e.kind == "warehouse":
        want = e.shelves or 4
        placed = 0
        for k in range(want * 3):
            if placed >= want:
                break
            row, col = k % 2, k // 2
            sx = -w / 2 + 2.2 + col * 1.7
            sy = h / 5 if row == 0 else -h / 5
            if abs(sx) > w / 2 - 1.0:
                continue
            if any(abs(sx - o["x"]) < 1.0 and abs(sy - o["y"]) < 0.5 for o in L.obstacles):
                continue
            if any(abs(sx - kx) < 0.8 + 0.2 and abs(sy - ky) < 0.35 + 0.2 + kr * 0.5 for kx, ky, kr in keep_out):
                continue
            L.obstacles.append({"x": sx, "y": sy, "sx": 0.8, "sy": 0.3, "h": 1.0, "kind": "shelf"})
            placed += 1
        if placed < want:
            L.notes.append(f"Only {placed} of {want} shelves fit without blocking starts/targets.")
        for k, (tx, ty) in enumerate(t for ts in L.targets[:1] for t in ts[:2]):
            L.markers.append({"x": tx, "y": ty, "r": 0.18, "color": (0.1, 0.6, 0.9) if k else (0.1, 0.8, 0.3),
                              "name": "dropoff_zone" if k else "pickup_zone", "z": 0.001})
    # box obstacles (explicit positions first, then random ones up to the requested total)
    for p in e.obstacle_positions:
        L.obstacles.append({"x": p[0], "y": p[1], "sx": 0.12, "sy": 0.12, "h": 0.12, "kind": "box"})
    total = max(e.obstacles, len(e.obstacle_positions))
    guard = 0
    while sum(1 for o in L.obstacles if o["kind"] == "box") < total:
        guard += 1
        if guard > 5000:
            raise ValueError("The room is too small for that many obstacles; reduce the obstacle count.")
        x, y = rng.uniform(-w / 2 + 0.4, w / 2 - 0.4), rng.uniform(-h / 2 + 0.4, h / 2 - 0.4)
        s = round(rng.uniform(0.08, 0.16), 3)
        if any(math.dist((x, y), (kx, ky)) < kr + 0.2 for kx, ky, kr in keep_out):
            continue
        if any(abs(x - o["x"]) < (o["sx"] + s) / 2 + 0.25 and abs(y - o["y"]) < (o["sy"] + s) / 2 + 0.25 for o in L.obstacles):
            continue
        L.obstacles.append({"x": round(x, 3), "y": round(y, 3), "sx": s, "sy": s, "h": s, "kind": "box"})
    return L


# ------------------------------------------------------------------------------------------------ node writers
def _pbr(c, rough=1.0, metal=0.0) -> str:
    return f"PBRAppearance {{ baseColor {c[0]} {c[1]} {c[2]} roughness {rough} metalness {metal} }}"


def _box_shape(size, color) -> str:
    return f"Shape {{ appearance {_pbr(color)} geometry Box {{ size {_fmt(size[0])} {_fmt(size[1])} {_fmt(size[2])} }} }}"


def custom_robot(name: str, ctrl: str, x: float, y: float, heading: float, spec: SimSpec, line_camera: bool) -> str:
    r = spec.robot
    L_, W, H, R = r.body_length, r.body_width, r.body_height, r.wheel_radius
    ww = 0.025
    z0 = H / 2 - R + 0.02                      # body centre height in the robot frame (axle at z = 0)
    mass = max(0.6, 350 * L_ * W * H)
    four = r.drive == "four_wheel"
    wheels = ([(L_ * 0.32, 1), (L_ * 0.32, -1), (-L_ * 0.32, 1), (-L_ * 0.32, -1)] if four else [(0.0, 1), (0.0, -1)])
    names = {(1, True): "front_left", (-1, True): "front_right"}
    out = []
    for wi, (wx, side) in enumerate(wheels):
        wy = side * (W / 2 + ww / 2)
        pos = "front" if (four and wx > 0) else ("rear" if four else "")
        side_n = "left" if side > 0 else "right"
        base = f"{pos}_{side_n}".strip("_")
        out.append(f"""    HingeJoint {{
      jointParameters HingeJointParameters {{ axis 0 1 0 anchor {_fmt(wx)} {_fmt(wy)} 0 }}
      device [
        RotationalMotor {{ name "{base}_motor" maxVelocity {_fmt(r.max_speed * 1.3)} maxTorque 8 }}
        PositionSensor {{ name "{base}_sensor" }}
      ]
      endPoint Solid {{
        translation {_fmt(wx)} {_fmt(wy)} 0
        children [ Shape {{ appearance {_pbr((0.08, 0.08, 0.08))} geometry Cylinder {{ height {ww} radius {_fmt(R)} }} }} ]
        name "wheel_{base}"
        boundingObject Cylinder {{ height {ww} radius {_fmt(R)} }}
        physics Physics {{ density -1 mass 0.15 }}
      }}
    }}""")
    if not four:
        for cx in (L_ / 2 - 0.04, -L_ / 2 + 0.04):
            out.append(f"""    BallJoint {{
      jointParameters BallJointParameters {{ anchor {_fmt(cx)} 0 {_fmt(-R + 0.02)} }}
      endPoint Solid {{
        translation {_fmt(cx)} 0 {_fmt(-R + 0.02)}
        children [ Shape {{ appearance {_pbr((0.7, 0.7, 0.7))} geometry Sphere {{ radius 0.02 }} }} ]
        name "caster_{'front' if cx > 0 else 'rear'}"
        boundingObject Sphere {{ radius 0.02 }}
        physics Physics {{ density -1 mass 0.05 }}
      }}
    }}""")
    for sname, ang in (("ds_left_side", 1.2), ("ds_left", 0.6), ("ds_front", 0.0), ("ds_right", -0.6), ("ds_right_side", -1.2)):
        sx_ = L_ / 2 if abs(ang) < 1.0 else L_ / 2 * 0.7
        out.append(f"""    DistanceSensor {{
      translation {_fmt(sx_)} {_fmt(math.sin(ang) * W * 0.45)} {_fmt(z0)}
      rotation 0 0 1 {ang}
      name "{sname}"
      lookupTable [ 0 1000 0, 0.5 0 0 ]
      type "generic"
    }}""")
    if line_camera:
        out.append(f"""    Camera {{
      translation {_fmt(L_ / 2 + 0.05)} 0 {_fmt(-R + 0.12)}
      rotation 0 1 0 1.5708
      name "line_camera"
      fieldOfView 1.2
      width 32
      height 8
    }}""")
    children = "\n".join(out)
    color = (0.15, 0.4, 0.85) if name.endswith("1") else (0.85, 0.4, 0.15)
    return f"""Robot {{
  translation {_fmt(x)} {_fmt(y)} {_fmt(R)}
  rotation 0 0 1 {heading:.5f}
  name "{name}"
  controller "{ctrl}"
  supervisor TRUE
  children [
    Transform {{
      translation 0 0 {_fmt(z0)}
      children [ {_box_shape((L_, W, H), color)} ]
    }}
{children}
  ]
  boundingObject Transform {{
    translation 0 0 {_fmt(z0)}
    children [ Box {{ size {_fmt(L_)} {_fmt(W)} {_fmt(H)} }} ]
  }}
  physics Physics {{ density -1 mass {_fmt(mass)} }}
}}"""


def arm_robot(name: str, ctrl: str) -> str:
    l1, l2, h0, th = ARM["l1"], ARM["l2"], ARM["h0"], ARM["table_h"]

    def joint(axis, anchor, motor, sensor, lo, hi, solid) -> str:
        return f"""HingeJoint {{
        jointParameters HingeJointParameters {{ axis {axis} anchor {anchor} }}
        device [
          RotationalMotor {{ name "{motor}" minPosition {lo} maxPosition {hi} maxVelocity 2.0 maxTorque 60 }}
          PositionSensor {{ name "{sensor}" }}
        ]
        endPoint {solid}
      }}"""

    link2 = f"""Solid {{
          translation {l1} 0 0
          children [
            Transform {{ translation {l2 / 2} 0 0 children [ {_box_shape((l2, 0.045, 0.045), (0.2, 0.7, 0.3))} ] }}
            DEF ARM_TIP Solid {{
              translation {l2} 0 0
              children [ Shape {{ appearance {_pbr((0.9, 0.1, 0.1))} geometry Sphere {{ radius 0.02 }} }} ]
              name "arm_tip"
            }}
          ]
          name "link2"
          boundingObject Transform {{ translation {l2 / 2} 0 0 children [ Box {{ size {l2} 0.045 0.045 }} ] }}
          physics Physics {{ density -1 mass 0.6 }}
        }}"""
    elbow = joint("0 1 0", f"{l1} 0 0", "elbow motor", "elbow sensor", 0, 2.8, link2)
    link1 = f"""Solid {{
          translation 0 0 {h0}
          children [
            Transform {{ translation {l1 / 2} 0 0 children [ {_box_shape((l1, 0.05, 0.05), (0.2, 0.4, 0.9))} ] }}
            {elbow}
          ]
          name "link1"
          boundingObject Transform {{ translation {l1 / 2} 0 0 children [ Box {{ size {l1} 0.05 0.05 }} ] }}
          physics Physics {{ density -1 mass 0.8 }}
        }}"""
    shoulder = joint("0 1 0", f"0 0 {h0}", "shoulder motor", "shoulder sensor", -1.6, 1.0, link1)
    turret = f"""Solid {{
          children [
            Transform {{ translation 0 0 {h0 / 2} children [ {_box_shape((0.08, 0.08, h0), (0.9, 0.6, 0.1))} ] }}
            {shoulder}
          ]
          name "turret"
          boundingObject Transform {{ translation 0 0 {h0 / 2} children [ Box {{ size 0.08 0.08 {h0} }} ] }}
          physics Physics {{ density -1 mass 1.0 }}
        }}"""
    yaw = joint("0 0 1", "0 0 0", "yaw motor", "yaw sensor", -3.14, 3.14, turret)
    return f"""Robot {{
  translation 0 0 {th}
  name "{name}"
  controller "{ctrl}"
  supervisor TRUE
  children [
    Shape {{ appearance {_pbr((0.3, 0.3, 0.3))} geometry Cylinder {{ height 0.02 radius 0.08 }} }}
    {yaw}
  ]
  boundingObject Cylinder {{ height 0.02 radius 0.08 }}
}}"""


def _marker(m: dict) -> str:
    c = m["color"]
    return f"""Solid {{
  translation {_fmt(m['x'])} {_fmt(m['y'])} {m.get('z', 0.001)}
  children [ Shape {{ appearance {_pbr(c)} geometry Cylinder {{ height 0.002 radius {m['r']} }} }} ]
  name "{m['name']}"
}}"""


def _cylinder_upright(shape: str) -> str:
    return shape


def _flat_cyl(m: dict) -> str:
    # Webots Cylinder is y-axis aligned: rotate to lie flat on the (x, y) floor
    c = m["color"]
    return f"""Solid {{
  translation {_fmt(m['x'])} {_fmt(m['y'])} {m.get('z', 0.001)}
  rotation 1 0 0 1.5708
  children [ Shape {{ appearance {_pbr(c)} geometry Cylinder {{ height 0.002 radius {m['r']} }} }} ]
  name "{m['name']}"
}}"""


def _line_segments(pts: list[tuple[float, float]]) -> str:
    out = []
    n = len(pts)
    for i in range(n):
        (x0, y0), (x1, y1) = pts[i], pts[(i + 1) % n]
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        ln = math.hypot(x1 - x0, y1 - y0) * 1.08
        ang = math.atan2(y1 - y0, x1 - x0)
        out.append(f"""Solid {{
  translation {_fmt(cx)} {_fmt(cy)} 0.0006
  rotation 0 0 1 {ang:.5f}
  children [ {_box_shape((ln, 0.045, 0.001), (0.02, 0.02, 0.02))} ]
  name "line_{i}"
}}""")
    return "\n".join(out)


# ------------------------------------------------------------------------------------------------ world
def needed_assets(spec: SimSpec) -> list[str]:
    keys = ["TexturedBackground", "TexturedBackgroundLight", "RectangleArena"]
    r, e = spec.robot, spec.environment
    if r.kind == "epuck":
        keys.append("E-puck")
    if r.kind == "drone":
        keys.append("Mavic2Pro")
    if r.kind == "arm" or (r.kind in ("epuck", "custom_wheeled") and (e.obstacles or e.obstacle_positions)):
        keys.append("WoodenBox")
    if e.kind == "warehouse":
        keys.append("CardboardBox")
    return keys


def controller_name(spec: SimSpec) -> str:
    return {"epuck": "wheeled_controller", "custom_wheeled": "wheeled_controller",
            "arm": "arm_controller", "drone": "drone_controller"}[spec.robot.kind]


def build_world(spec: SimSpec, layout: Layout, assets: dict[str, Asset], version: str) -> tuple[str, dict]:
    """Return (world text, controller config dict)."""
    r, e, b = spec.robot, spec.environment, spec.behavior
    ctrl = controller_name(spec)
    w, h = e.width, e.height
    cam = max(w, h) * 1.1 + 0.8
    ext = "\n".join(a.externproto_line for a in assets.values())
    has_caster = r.kind == "custom_wheeled" and r.drive == "differential"      # noqa: F841 (documented behaviour)
    parts: list[str] = []
    vp_target = (0.0, 0.0, 0.0)
    floor = "floorAppearance " + _pbr((0.88, 0.88, 0.88)) if b.task == "line_follow" else ""
    wall_h = 0.1 if r.kind != "drone" else 0.05
    if r.kind == "arm":
        parts.append(f"RectangleArena {{ floorSize 4 4 wallHeight 0.02 }}")
        parts.append(f"""Solid {{
  translation 0 0 {ARM['table_h'] / 2}
  children [ {_box_shape((1.2, 1.2, ARM['table_h']), (0.55, 0.4, 0.25))} ]
  name "table"
  boundingObject Box {{ size 1.2 1.2 {ARM['table_h']} }}
}}""")
        cz = ARM["table_h"] + ARM["cube"] / 2
        pick = layout.targets[0][0]
        parts.append(f"DEF OBJECT WoodenBox {{ translation {_fmt(pick[0])} {_fmt(pick[1])} {_fmt(cz + 0.002)} name \"cube\" "
                     f"size {ARM['cube']} {ARM['cube']} {ARM['cube']} mass 0.1 }}")
        for m in layout.markers:
            parts.append(_flat_cyl(m))
        parts.append(arm_robot("arm", ctrl))
        names = ["arm"]
        vp = (0.0, -1.5, 1.4)
        vp_target = (0.0, 0.0, ARM["table_h"])
    elif r.kind == "drone":
        parts.append(f"RectangleArena {{ floorSize {_fmt(w)} {_fmt(h)} wallHeight {wall_h} }}")
        for m in layout.markers:
            parts.append(_flat_cyl(m))
        parts.append(f'Mavic2Pro {{ translation 0 0 0.1 name "drone" controller "{ctrl}" supervisor TRUE }}')
        names = ["drone"]
        vp = (0.0, -cam * 0.7, cam * 0.6)
    else:
        parts.append(f"RectangleArena {{ floorSize {_fmt(w)} {_fmt(h)} wallHeight {wall_h} {floor} }}")
        if layout.line:
            parts.append(_line_segments(layout.line))
        for i, o in enumerate(layout.obstacles):
            if o["kind"] == "shelf":
                parts.append(f"""Solid {{
  translation {_fmt(o['x'])} {_fmt(o['y'])} {_fmt(o['h'] / 2)}
  children [ {_box_shape((o['sx'], o['sy'], o['h']), (0.45, 0.45, 0.5))} ]
  name "shelf_{i}"
  boundingObject Box {{ size {o['sx']} {o['sy']} {o['h']} }}
}}""")
            else:
                parts.append(f"WoodenBox {{ translation {_fmt(o['x'])} {_fmt(o['y'])} {_fmt(o['sx'] / 2)} name \"obstacle_{i}\" "
                             f"size {o['sx']} {o['sy']} {o['h']} mass 3 }}")
        for m in layout.markers:
            parts.append(_flat_cyl(m))
        if e.kind == "warehouse" and layout.markers:
            m0 = layout.markers[0]
            for j, (dx, dy) in enumerate(((0.0, 0.45), (0.3, 0.45))):
                parts.append(f'CardboardBox {{ translation {_fmt(m0["x"] + dx)} {_fmt(m0["y"] + dy)} 0.1 name "carton_{j}" size 0.2 0.2 0.2 }}')
        if b.task == "go_to_targets" and e.kind != "warehouse":
            for k, t in enumerate(layout.targets[0] if layout.targets else []):
                parts.append(_flat_cyl({"x": t[0], "y": t[1], "r": 0.06, "color": (1.0, 0.1, 0.1), "name": f"target_{k + 1}"}))
        names = []
        for i, (sx, sy, hd) in enumerate(layout.starts):
            nm = f"robot_{i + 1}"
            names.append(nm)
            if r.kind == "epuck":
                parts.append(f'E-puck {{ translation {_fmt(sx)} {_fmt(sy)} 0 rotation 0 0 1 {hd:.5f} name "{nm}" '
                             f'controller "{ctrl}" supervisor TRUE }}')
            else:
                parts.append(custom_robot(nm, ctrl, sx, sy, hd, spec, b.task == "line_follow"))
        vp = (0.0, -cam * 0.64, cam * 0.77)
    world = f"""#VRML_SIM {version} utf8

{ext}

WorldInfo {{
  title "{spec.title.replace('"', "'")}"
  basicTimeStep {spec.sim.time_step_ms}
  coordinateSystem "ENU"
}}
Viewpoint {{
  orientation {look_at(vp, vp_target)}
  position {vp[0]:.3f} {vp[1]:.3f} {vp[2]:.3f}
}}
TexturedBackground {{
}}
TexturedBackgroundLight {{
}}
""" + "\n".join(parts) + "\n"

    # ---------------- controller config
    cfg: dict = {"task": b.task, "robot_names": names, "timeout_s": spec.sim.timeout_s, "keep_running": False,
                 "room": [w, h], "obstacles": layout.obstacles, "goal_tol": 0.08, "loop_targets": b.loop_targets,
                 "duration_s": b.duration_s, "laps": b.laps, "max_speed": min(r.max_speed, 6.28) if r.kind == "epuck" else r.max_speed,
                 "robot_radius": robot_radius(spec), "contact_radius": contact_radius(spec), "track_center": list(layout.track_center)}
    if r.kind in ("epuck", "custom_wheeled"):
        cfg["robots"] = {nm: {"start": list(layout.starts[i]), "targets": [list(t) for t in layout.targets[i]]}
                         for i, nm in enumerate(names)}
        if r.kind == "epuck":
            cfg["motors"] = {"left": ["left wheel motor"], "right": ["right wheel motor"]}
            cfg["distance"] = {"layout": "epuck", "names": [f"ps{i}" for i in range(8)], "scale": 1000.0,
                               "avoid_on": 0.12, "avoid_off": 0.05, "side_on": 0.03}
            cfg["bypass_s"] = 1.6
            cfg["goal_tol"] = 0.07
        else:
            if r.drive == "four_wheel":
                cfg["motors"] = {"left": ["front_left_motor", "rear_left_motor"], "right": ["front_right_motor", "rear_right_motor"]}
            else:
                cfg["motors"] = {"left": ["left_motor"], "right": ["right_motor"]}
            on_ = round(max(0.2, min(0.4, 0.45 - 0.5 * r.body_length)), 3)   # longer body -> react earlier
            cfg["distance"] = {"layout": "custom", "names": ["ds_left_side", "ds_left", "ds_front", "ds_right", "ds_right_side"], "scale": 1000.0,
                               "avoid_on": on_, "avoid_off": round(on_ * 0.55, 3), "side_on": round(on_ * 0.5, 3)}
            cfg["bypass_s"] = 1.2 + 2.0 * max(0.0, r.body_length - 0.3)
            cfg["goal_tol"] = max(0.12, 0.35 * r.body_length)
            cfg["line_camera"] = {"name": "line_camera"}
    elif r.kind == "arm":
        cfg["arm"] = {"l1": ARM["l1"], "l2": ARM["l2"], "h0": ARM["h0"], "base": [0.0, 0.0, ARM["table_h"]],
                      "pick": list(layout.targets[0][0]), "place": list(layout.targets[0][1]),
                      "cube_z": ARM["table_h"] + ARM["cube"] / 2,
                      "limits": {"shoulder": [-1.6, 1.0], "elbow": [0.0, 2.8]}}
        cfg["robots"] = {"arm": {"targets": []}}
    else:
        cfg["altitude"] = b.altitude
        cfg["waypoints"] = [list(p) for p in layout.targets[0]]
        cfg["robots"] = {"drone": {"targets": []}}
    return world, cfg
