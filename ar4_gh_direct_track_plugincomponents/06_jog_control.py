#! python3
# r: compas
# r: compas_fab
# r: compas_robots
# r: pyserial

# =============================================================================
# GH_PINS — Rhino does not create pins from pasted code. Copy these names into
# the Python Script component UI (Type Hint / Access in COMPONENT_PINS.txt).
# INPUTS:
#   J1
#   J2
#   J3
#   J4
#   J5
#   J6
#   TrackM
#   Send
#   Capture
#   CaptureLabel
#   ClearCaptures
#   Speed
#   SpeedType
#   Accel
#   Decel
#   Ramp
#   Wrist
#   SendTimeout
#   EncodeFirmwareOnSend
#   DrawRobot
#   DrawTrack
#   DrawSkeleton
#   Mode
#   X
#   Y
#   Z
#   Rx
#   Ry
#   Rz
#   HideGripper
#   BasePlane
#   TrackLengthMM
#   TrackRailLengthMM
#   TrackFirmwareInverted
#   HomeJointsDeg
#   JointSigns
# OUTPUTS:
#   robot_geometry
#   tool_geometry
#   track_geometry
#   skeleton_lines
#   flange_plane
#   tcp_plane
#   joints_urdf_deg
#   xyz_mm
#   rpy_deg
#   rj_command
#   captures
#   capture_count
#   capture_labels
#   jog_state
#   status
# =============================================================================

"""
AR4 + External Track — Component 6: Manual Jog / Pose Capture
Grasshopper Python 3 / Rhino 8

Sliders drive the joints. FK redraws the robot every solve, and a separate
Send toggle pushes the current pose to the Teensy. Capture logs the pose for
Comp7 to turn into a TCP calibration.

Joint convention matches Comp3 and Comp4: J1..J6 sliders are URDF / geometric
degrees, the same numbers that appear in Comp3's rj_commands. The firmware
offset is applied only on the way out to serial:

    firmware = (urdf - HomeJointsDeg) * JointSigns

so a slider at 0 is the URDF zero pose, not the robot's home switch.

SAFETY
------
Send moves the real arm. It is edge triggered — the pose goes out once when
Send flips False -> True, never continuously while a slider is dragged, so a
slider drag cannot chase the robot across the cell. Jog speed defaults to a
percentage move (Sp) at 15 percent rather than Comp3's Sm.


INPUTS
------
Mode              : "joint" (default) or "cartesian" / "cart" / "absolute"
J1..J6            : joint mode — URDF degrees
TrackM            : 0..1 usable track travel (both modes)
X,Y,Z             : cartesian — absolute TCP in robot-base mm once you move
                    those pins (same frame as Comp3). Untouched pins keep the
                    TCP frozen at mode entry.
Rx,Ry,Rz          : rotation jog in degrees about the TCP tool axes
                    (Rx about TCP X, then Ry about TCP Y, then Rz about TCP Z).
                    Values are deltas from the pin reading at mode entry, so
                    leave them at 0 and nudge. Rotations are always centered
                    on the TCP, not the flange.

Tool / TCP come from Comp1 sticky (and Comp7 calibration). No ToolMeshes /
TCPPlane pins on this component.

Serial (Comp1 bus; Send ignored if Connect is False):
Send, Speed, SpeedType, Accel, Decel, Ramp, Wrist, EncodeFirmwareOnSend,
SendTimeout

Capture: Capture, CaptureLabel, ClearCaptures
Display: DrawRobot, DrawTrack, DrawSkeleton, HideGripper
Track: TrackLengthMM, TrackRailLengthMM, TrackFirmwareInverted
HomeJointsDeg / JointSigns : defaults from sticky


OUTPUTS
-------
robot_geometry / tool_geometry / track_geometry / skeleton_lines
flange_plane / tcp_plane
joints_urdf_deg   : the joint angles actually drawn / sent (both modes)
xyz_mm / rpy_deg  : current TCP pose readout in robot-base mm / deg
rj_command, captures, capture_count, capture_labels, jog_state, status
"""

import math
import os
import time
import xml.etree.ElementTree as ET

import Rhino.Geometry as rg

try:
    import scriptcontext as sc
except Exception:
    sc = None


CAPTURE_STICKY_KEY = "AR4_JOG_CAPTURES"

GRIPPER_LINK_NAMES = (
    "gripper_base_link",
    "gripper_jaw1_link",
    "gripper_jaw2_link"
)

DEFAULT_HOME_URDF_DEG = [-16.0, 0.0, 0.0, 0.0, 22.0, -10.0]
DEFAULT_JOINT_SIGNS = [-1.0, 1.0, 1.0, 1.0, 1.0, 1.0]

MOTION_JOINT_NAMES = [
    "track_joint",
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6"
]


# ------------------------------------------------------------
# Helpers: Grasshopper values
# ------------------------------------------------------------

def unwrap_gh(value):
    if value is None:
        return None

    if hasattr(value, "Value"):
        return value.Value

    return value


def gh_input(name, default=None):
    """Read a GH input by name without raising when the pin is absent."""
    return unwrap_gh(globals().get(name, default))


def to_bool(value, default=False):
    value = unwrap_gh(value)

    if value is None:
        return default

    if isinstance(value, bool):
        return value

    text = str(value).strip().lower()

    if text in ["true", "1", "yes", "y", "on"]:
        return True

    if text in ["false", "0", "no", "n", "off"]:
        return False

    return default


def to_float(value, default=0.0):
    value = unwrap_gh(value)

    if value is None:
        return default

    try:
        return float(value)
    except Exception:
        try:
            return float(str(value).strip())
        except Exception:
            return default


def to_text(value, default=""):
    value = unwrap_gh(value)

    if value is None:
        return default

    return str(value).strip()


def clamp(value, low, high):
    if value < low:
        return low

    if value > high:
        return high

    return value


def parse_float_list_input(value, count=6, default=None):
    if value is None:
        return None if default is None else list(default)

    value = unwrap_gh(value)

    try:
        if isinstance(value, (list, tuple)):
            vals = [to_float(v, 0.0) for v in value]
        else:
            vals = [float(v) for v in str(value).replace(",", " ").split()]
    except Exception:
        return None if default is None else list(default)

    if len(vals) < count:
        if default is None:
            return None
        vals = vals + list(default)[len(vals):count]

    return vals[:count]


def edge_triggered(key, value):
    """
    True only on the solve where value goes False -> True.

    Grasshopper re-runs the whole script on every change, so a plain bool read
    would fire again on the next unrelated slider move.
    """
    if sc is None:
        return False

    previous = bool(sc.sticky.get(key, False))
    sc.sticky[key] = bool(value)

    return bool(value) and not previous


# ------------------------------------------------------------
# Helpers: cached startup model
# ------------------------------------------------------------

def cache_timestamp_from_id(cache_id):
    try:
        return int(str(cache_id).split("_")[-1])
    except Exception:
        return 0


def find_latest_startup_cache():
    if sc is None:
        return None, None, {}

    candidates = []

    try:
        keys = list(sc.sticky.keys())
    except Exception:
        keys = []

    for key in keys:
        k = str(key)

        if k.startswith("AR4_STARTUP_") and k.endswith("::model"):
            cid = k.replace("::model", "")
            model = sc.sticky.get(cid + "::model", None)

            if model is not None:
                candidates.append((cache_timestamp_from_id(cid), cid, model))

    if not candidates:
        return None, None, {}

    candidates.sort()
    _, cid, model = candidates[-1]

    return cid, model, sc.sticky.get(cid + "::meshes", {})


# ------------------------------------------------------------
# Base plane
# ------------------------------------------------------------

_BASE_XFORM = None


def to_plane(value, default=None):
    value = unwrap_gh(value)

    if value is None:
        return default

    if isinstance(value, rg.Plane):
        return rg.Plane(value) if value.IsValid else default

    try:
        return rg.Plane(
            rg.Point3d(float(value.X), float(value.Y), float(value.Z)),
            rg.Vector3d.XAxis,
            rg.Vector3d.YAxis
        )
    except Exception:
        return default


def is_world_plane(plane):
    if plane is None:
        return True

    try:
        return (
            plane.Origin.EpsilonEquals(rg.Point3d(0.0, 0.0, 0.0), 1e-9)
            and plane.XAxis.EpsilonEquals(rg.Vector3d.XAxis, 1e-9)
            and plane.YAxis.EpsilonEquals(rg.Vector3d.YAxis, 1e-9)
        )
    except Exception:
        return False


def set_base_xform(base_plane):
    global _BASE_XFORM

    if is_world_plane(base_plane):
        _BASE_XFORM = None
    else:
        _BASE_XFORM = rg.Transform.PlaneToPlane(rg.Plane.WorldXY, base_plane)


def resolve_base_plane(cache_id):
    plane = to_plane(gh_input("BasePlane", None), None)

    if plane is not None:
        return plane, "BasePlane input"

    if sc is not None and cache_id:
        plane = to_plane(sc.sticky.get(str(cache_id) + "::base_plane", None), None)

        if plane is not None:
            return plane, "Comp1 startup base_plane"

    return rg.Plane.WorldXY, "world XY"


def to_world_plane(plane):
    if _BASE_XFORM is None or plane is None:
        return plane

    p = rg.Plane(plane)
    p.Transform(_BASE_XFORM)

    return p


def to_world_point(point):
    if _BASE_XFORM is None or point is None:
        return point

    p = rg.Point3d(point)
    p.Transform(_BASE_XFORM)

    return p


# ------------------------------------------------------------
# TCP frame — same dict contract as Comp3, Comp4 and Comp7
# ------------------------------------------------------------

def tcp_dict_to_plane(tcp):
    if not isinstance(tcp, dict):
        return rg.Plane.WorldXY

    try:
        o = [float(v) for v in tcp.get("origin_m", [0.0, 0.0, 0.0])]
        x = [float(v) for v in tcp.get("xaxis", [1.0, 0.0, 0.0])]
        y = [float(v) for v in tcp.get("yaxis", [0.0, 1.0, 0.0])]

        plane = rg.Plane(
            rg.Point3d(o[0], o[1], o[2]),
            rg.Vector3d(x[0], x[1], x[2]),
            rg.Vector3d(y[0], y[1], y[2])
        )

        return plane if plane.IsValid else rg.Plane.WorldXY

    except Exception:
        return rg.Plane.WorldXY


def offset_plane_along_z(plane, distance):
    p = rg.Plane(plane)
    z = rg.Vector3d(p.ZAxis)

    if z.Length > 1e-12:
        z.Unitize()

    p.Origin = p.Origin + z * distance

    return p


def plane_origin_to_meters(plane, mm_per_unit=0.001):
    p = rg.Plane(plane)
    p.Origin = rg.Point3d(
        p.Origin.X * mm_per_unit,
        p.Origin.Y * mm_per_unit,
        p.Origin.Z * mm_per_unit
    )

    return p


def resolve_tcp_frame():
    """Flange-local TCP in URDF meters from Comp7 / Comp1 sticky only."""
    if sc is not None:
        d = sc.sticky.get("AR4_TCP_FRAME", None)
        if isinstance(d, dict):
            return tcp_dict_to_plane(d), "Comp7 " + str(d.get("source", ""))

        d = sc.sticky.get("AR4_TOOL_TCP_FRAME", None)
        if isinstance(d, dict):
            return tcp_dict_to_plane(d), str(d.get("source", "Comp1 tool"))

    return rg.Plane.WorldXY, "flange (no tool)"


def apply_tcp_frame(flange_plane, tcp_local, display_scale):
    if tcp_local is None or flange_plane is None:
        return flange_plane

    s = float(display_scale)

    p = rg.Plane(tcp_local)
    p.Origin = rg.Point3d(
        p.Origin.X * s,
        p.Origin.Y * s,
        p.Origin.Z * s
    )
    p.Transform(rg.Transform.PlaneToPlane(rg.Plane.WorldXY, flange_plane))

    return p


# ------------------------------------------------------------
# Track mapping — must stay identical to Comp3 and Comp4
# ------------------------------------------------------------

def track_pose_to_urdf_meters(track_m, travel_mm=870.0, rail_mm=1000.0):
    travel_mm = abs(to_float(travel_mm, 870.0))
    if travel_mm < 1e-9:
        travel_mm = 870.0

    rail_mm = abs(to_float(rail_mm, 1000.0))
    if rail_mm < 1e-9:
        rail_mm = 1000.0

    return clamp(to_float(track_m, 0.0), 0.0, 1.0) * (travel_mm / rail_mm)


def track_m_to_j7_mm(track_m, travel_mm, track_firmware_inverted):
    travel_mm = abs(to_float(travel_mm, 870.0))
    if travel_mm < 1e-9:
        travel_mm = 870.0

    t = clamp(to_float(track_m, 0.0), 0.0, 1.0)

    if track_firmware_inverted:
        return (1.0 - t) * travel_mm

    return t * travel_mm


# ------------------------------------------------------------
# FK and display
# ------------------------------------------------------------

def make_config(model, track_m, joints_rad, travel_mm=870.0, rail_mm=1000.0):
    config = model.zero_configuration()

    try:
        configurable = model.get_configurable_joint_names()
    except Exception:
        configurable = []

    if "track_joint" in configurable:
        config["track_joint"] = track_pose_to_urdf_meters(
            track_m=track_m,
            travel_mm=travel_mm,
            rail_mm=rail_mm
        )

    for i in range(6):
        name = "joint_{}".format(i + 1)

        if name in configurable:
            config[name] = joints_rad[i]

    if "gripper_jaw1_joint" in configurable:
        config["gripper_jaw1_joint"] = 0.0

    return config


def safe_fk(model, config, link_name=None):
    try:
        if link_name:
            return model.forward_kinematics(config, link_name=link_name)

        return model.forward_kinematics(config)

    except Exception:
        return None


def choose_tool_link(model):
    """Same preference order as Comp3 so TCP sticky matches the flange link."""
    try:
        link_names = [link.name for link in model.links]
    except Exception:
        link_names = []

    for name in ["ee_link", "tool0", "tool_link", "pneumatic_gripper_tool_link", "link_6"]:
        if name in link_names:
            return name

    return link_names[-1] if link_names else None


def get_urdf_path_from_cache(cache_id):
    if sc is None or not cache_id:
        return ""

    state = sc.sticky.get(str(cache_id) + "::startup_state", None)
    if isinstance(state, dict):
        return to_text(state.get("urdf", ""), "")

    return ""


def parse_float_list_text(text, default_values):
    if text is None:
        return list(default_values)

    try:
        values = [float(v) for v in str(text).replace(",", " ").split()]
    except Exception:
        return list(default_values)

    result = list(default_values)
    for i in range(min(len(values), len(result))):
        result[i] = values[i]
    return result


def parse_urdf_motion_joints(urdf_path):
    data = {}
    if not urdf_path or not os.path.exists(urdf_path):
        return data

    try:
        root = ET.parse(urdf_path).getroot()
    except Exception:
        return data

    for joint in root.findall("joint"):
        name = joint.attrib.get("name", "")
        if name not in MOTION_JOINT_NAMES:
            continue

        jtype = joint.attrib.get("type", "")
        parent = joint.find("parent")
        child = joint.find("child")
        origin = joint.find("origin")
        axis = joint.find("axis")
        limit = joint.find("limit")

        xyz = [0.0, 0.0, 0.0]
        rpy = [0.0, 0.0, 0.0]
        if origin is not None:
            xyz = parse_float_list_text(origin.attrib.get("xyz", None), [0.0, 0.0, 0.0])
            rpy = parse_float_list_text(origin.attrib.get("rpy", None), [0.0, 0.0, 0.0])

        axis_xyz = [0.0, 0.0, 1.0]
        if axis is not None:
            axis_xyz = parse_float_list_text(axis.attrib.get("xyz", None), [0.0, 0.0, 1.0])

        lower = 0.0 if name == "track_joint" else -math.pi
        upper = 1.0 if name == "track_joint" else math.pi
        if limit is not None:
            try:
                lower = float(limit.attrib.get("lower", lower))
            except Exception:
                pass
            try:
                upper = float(limit.attrib.get("upper", upper))
            except Exception:
                pass
            # Track q is logical 0..1 even if a URDF lists meter limits.
            if name == "track_joint" and upper > 1.0 + 1e-9:
                lower = 0.0
                upper = 1.0

        data[name] = {
            "name": name,
            "type": jtype,
            "parent": parent.attrib.get("link", "") if parent is not None else "",
            "child": child.attrib.get("link", "") if child is not None else "",
            "xyz": xyz,
            "rpy": rpy,
            "axis": axis_xyz,
            "lower": lower,
            "upper": upper
        }

    return data


def parse_urdf_fk_chain(urdf_path, tool_link):
    chain = []
    if not urdf_path or not os.path.exists(urdf_path) or not tool_link:
        return chain

    try:
        root = ET.parse(urdf_path).getroot()
    except Exception:
        return chain

    joint_by_child = {}
    for joint in root.findall("joint"):
        child = joint.find("child")
        if child is None:
            continue
        joint_by_child[child.attrib.get("link", "")] = joint

    link = tool_link
    guard = 0
    while link in joint_by_child and guard < 256:
        guard += 1
        joint = joint_by_child[link]
        origin = joint.find("origin")
        axis = joint.find("axis")
        parent = joint.find("parent")

        xyz = [0.0, 0.0, 0.0]
        rpy = [0.0, 0.0, 0.0]
        if origin is not None:
            xyz = parse_float_list_text(origin.attrib.get("xyz", None), [0.0, 0.0, 0.0])
            rpy = parse_float_list_text(origin.attrib.get("rpy", None), [0.0, 0.0, 0.0])

        axis_xyz = [0.0, 0.0, 1.0]
        if axis is not None:
            axis_xyz = parse_float_list_text(axis.attrib.get("xyz", None), [0.0, 0.0, 1.0])

        chain.append({
            "name": joint.attrib.get("name", ""),
            "type": joint.attrib.get("type", "fixed"),
            "xyz": xyz,
            "rpy": rpy,
            "axis": axis_xyz
        })
        link = parent.attrib.get("link", "") if parent is not None else ""

    chain.reverse()
    return chain


def get_joint_info(cache_id, urdf_path):
    if sc is None:
        return parse_urdf_motion_joints(urdf_path)

    key = "{}::motion_joint_info::{}".format(cache_id, urdf_path)
    cached = sc.sticky.get(key, None)
    if isinstance(cached, dict) and cached:
        return cached

    joint_info = parse_urdf_motion_joints(urdf_path)
    sc.sticky[key] = joint_info
    return joint_info


def get_fk_chain(cache_id, urdf_path, tool_link):
    if sc is None:
        return parse_urdf_fk_chain(urdf_path, tool_link)

    key = "{}::fk_chain::{}::{}".format(cache_id, urdf_path, tool_link)
    cached = sc.sticky.get(key, None)
    if isinstance(cached, list) and cached:
        return cached

    chain = parse_urdf_fk_chain(urdf_path, tool_link)
    sc.sticky[key] = chain
    return chain


def frame_to_local_plane(frame):
    """COMPAS frame -> rg.Plane in robot-local URDF meters, no base transform."""
    p = frame.point
    x = frame.xaxis
    y = frame.yaxis

    return rg.Plane(
        rg.Point3d(float(p.x), float(p.y), float(p.z)),
        rg.Vector3d(float(x.x), float(x.y), float(x.z)),
        rg.Vector3d(float(y.x), float(y.y), float(y.z))
    )


def frame_to_plane(frame, display_scale=1.0):
    p = frame.point
    x = frame.xaxis
    y = frame.yaxis
    s = float(display_scale)

    return to_world_plane(rg.Plane(
        rg.Point3d(float(p.x) * s, float(p.y) * s, float(p.z) * s),
        rg.Vector3d(float(x.x), float(x.y), float(x.z)),
        rg.Vector3d(float(y.x), float(y.y), float(y.z))
    ))


def frame_point(frame, display_scale=1.0):
    p = frame.point
    s = float(display_scale)

    return to_world_point(
        rg.Point3d(float(p.x) * s, float(p.y) * s, float(p.z) * s)
    )


def link_ref_name(value):
    if value is None:
        return None

    if isinstance(value, str):
        return value

    if hasattr(value, "link"):
        return value.link

    if hasattr(value, "name"):
        return value.name

    return str(value)


def get_frame_by_link(model, config):
    frame_by_link = {}

    try:
        link_names = [link.name for link in model.links]
    except Exception:
        link_names = []

    for name in link_names:
        f = safe_fk(model, config, name)

        if f is not None:
            frame_by_link[name] = f

    return frame_by_link


def draw_robot_meshes(mesh_cache, frame_by_link, display_scale=1.0, exclude_links=None):
    geometry = []
    s = float(display_scale)
    scale_local = rg.Transform.Scale(rg.Point3d(0.0, 0.0, 0.0), s)
    exclude = set(exclude_links or ())

    for link_name, local_meshes in mesh_cache.items():

        if link_name not in frame_by_link or link_name in exclude:
            continue

        link_plane = frame_to_plane(frame_by_link[link_name], display_scale)
        xform = rg.Transform.PlaneToPlane(rg.Plane.WorldXY, link_plane)

        for base_mesh in local_meshes:
            moved = base_mesh.DuplicateMesh()

            if abs(s - 1.0) > 1e-12:
                moved.Transform(scale_local)

            moved.Transform(xform)
            geometry.append(moved)

    return geometry


def get_tool_meshes():
    """Tool meshes are owned by Comp1 (sticky AR4_TOOL_MESHES)."""
    if sc is None:
        return []
    cached = sc.sticky.get("AR4_TOOL_MESHES", None)
    return list(cached) if isinstance(cached, list) else []


def plane_from_xyz_rpy_mm(x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg):
    """
    Robot-base TCP plane in display mm.

    Fixed-axis RPY matching URDF convention: R = Rz(rz) Ry(ry) Rx(rx).
    Rotations use the parent (pre-rotation) axes — same as Comp3 / URDF.
    """
    origin = rg.Point3d(x_mm, y_mm, z_mm)
    plane = rg.Plane(origin, rg.Vector3d.XAxis, rg.Vector3d.YAxis)
    px = rg.Vector3d(plane.XAxis)
    py = rg.Vector3d(plane.YAxis)
    pz = rg.Vector3d(plane.ZAxis)
    rx = math.radians(rx_deg)
    ry = math.radians(ry_deg)
    rz = math.radians(rz_deg)
    if abs(rx) > 1e-15:
        plane.Transform(rg.Transform.Rotation(rx, px, origin))
    if abs(ry) > 1e-15:
        plane.Transform(rg.Transform.Rotation(ry, py, origin))
    if abs(rz) > 1e-15:
        plane.Transform(rg.Transform.Rotation(rz, pz, origin))
    return plane


def rpy_deg_from_plane(plane):
    """Extract fixed-axis RPY degrees from a plane (Rz*Ry*Rx)."""
    r00 = plane.XAxis.X
    r10 = plane.XAxis.Y
    r20 = plane.XAxis.Z
    r11 = plane.YAxis.Y
    r12 = plane.ZAxis.Y
    r21 = plane.YAxis.Z
    r22 = plane.ZAxis.Z
    sy = math.sqrt(r00 * r00 + r10 * r10)
    if sy > 1e-9:
        rx = math.atan2(r21, r22)
        ry = math.atan2(-r20, sy)
        rz = math.atan2(r10, r00)
    else:
        rx = math.atan2(-r12, r11)
        ry = math.atan2(-r20, sy)
        rz = 0.0
    return [
        math.degrees(rx),
        math.degrees(ry),
        math.degrees(rz)
    ]


def scale_plane_origin(plane, scale):
    p = rg.Plane(plane)
    p.Origin = rg.Point3d(
        p.Origin.X * scale,
        p.Origin.Y * scale,
        p.Origin.Z * scale
    )
    return p


def plane_origin_transform(parent_plane, xyz, rpy):
    """
    Same as Comp3: URDF joint origin translated in the parent frame, then
    fixed-axis RPY about the PARENT axes (R = Rz Ry Rx).
    """
    px = rg.Vector3d(parent_plane.XAxis)
    py = rg.Vector3d(parent_plane.YAxis)
    pz = rg.Vector3d(parent_plane.ZAxis)

    origin = (
        parent_plane.Origin
        + px * xyz[0]
        + py * xyz[1]
        + pz * xyz[2]
    )

    joint_plane = rg.Plane(parent_plane)
    joint_plane.Origin = origin

    roll, pitch, yaw = rpy
    if abs(roll) > 1e-15:
        joint_plane.Transform(rg.Transform.Rotation(roll, px, origin))
    if abs(pitch) > 1e-15:
        joint_plane.Transform(rg.Transform.Rotation(pitch, py, origin))
    if abs(yaw) > 1e-15:
        joint_plane.Transform(rg.Transform.Rotation(yaw, pz, origin))

    return joint_plane


def pins_changed(pins, pin0, tol=1e-6):
    if not pin0 or len(pin0) < len(pins):
        return True
    for a, b in zip(pins, pin0):
        if abs(float(a) - float(b)) > tol:
            return True
    return False


def build_cartesian_target_m(lock_plane, pin_vals, pin0, display_scale):
    """
    Build the IK target TCP plane in URDF meters.

    Position: absolute robot-base mm from X/Y/Z once those pins move; otherwise
    keep the locked TCP origin.

    Orientation: always the locked TCP frame plus Rx/Ry/Rz deltas about that
    TCP's own axes (centered on the TCP tip, not the flange). Absolute RPY
    from near-zero panels was flipping the tool because entry orientation is
    typically near 180 deg, not 0.
    """
    lock = rg.Plane(lock_plane)
    s = float(display_scale) if abs(float(display_scale)) > 1e-12 else 1000.0

    xyz_pins = list(pin_vals[:3])
    xyz0 = list(pin0[:3])
    rpy_pins = list(pin_vals[3:6])
    rpy0 = list(pin0[3:6])

    if pins_changed(xyz_pins, xyz0):
        origin = rg.Point3d(
            float(xyz_pins[0]) / s,
            float(xyz_pins[1]) / s,
            float(xyz_pins[2]) / s
        )
    else:
        origin = rg.Point3d(lock.Origin)

    target = rg.Plane(lock)
    target.Origin = origin

    # Freeze tool axes from the lock frame; apply deltas about the TCP origin.
    tx = rg.Vector3d(lock.XAxis)
    ty = rg.Vector3d(lock.YAxis)
    tz = rg.Vector3d(lock.ZAxis)
    if tx.Length > 1e-12:
        tx.Unitize()
    if ty.Length > 1e-12:
        ty.Unitize()
    if tz.Length > 1e-12:
        tz.Unitize()

    drx = math.radians(float(rpy_pins[0]) - float(rpy0[0]))
    dry = math.radians(float(rpy_pins[1]) - float(rpy0[1]))
    drz = math.radians(float(rpy_pins[2]) - float(rpy0[2]))

    if abs(drx) > 1e-15:
        target.Transform(rg.Transform.Rotation(drx, tx, origin))
    if abs(dry) > 1e-15:
        target.Transform(rg.Transform.Rotation(dry, ty, origin))
    if abs(drz) > 1e-15:
        target.Transform(rg.Transform.Rotation(drz, tz, origin))

    return target


def axis_world_from_joint_plane(joint_plane, axis_xyz):
    v = (
        rg.Vector3d(joint_plane.XAxis) * axis_xyz[0]
        + rg.Vector3d(joint_plane.YAxis) * axis_xyz[1]
        + rg.Vector3d(joint_plane.ZAxis) * axis_xyz[2]
    )
    if v.Length > 1e-12:
        v.Unitize()
    return v


def tcp_plane_from_flange(flange_plane, tcp_local):
    if tcp_local is None:
        return flange_plane
    p = rg.Plane(tcp_local)
    p.Transform(rg.Transform.PlaneToPlane(rg.Plane.WorldXY, flange_plane))
    return p


def chain_joint_values(track_m, joints_rad, travel_mm, rail_mm):
    values = {
        "track_joint": track_pose_to_urdf_meters(track_m, travel_mm, rail_mm)
    }
    for i in range(6):
        values["joint_{}".format(i + 1)] = joints_rad[i]
    return values


def chain_fk(chain, values):
    plane = rg.Plane.WorldXY
    axes = {}

    for entry in chain:
        plane = plane_origin_transform(plane, entry["xyz"], entry["rpy"])
        jtype = entry["type"]
        if jtype == "fixed":
            continue

        axis_world = axis_world_from_joint_plane(plane, entry["axis"])
        axes[entry["name"]] = (axis_world, rg.Point3d(plane.Origin))

        value = values.get(entry["name"], 0.0)
        if jtype == "prismatic":
            plane.Origin = plane.Origin + axis_world * value
        elif abs(value) > 1e-15:
            plane.Transform(
                rg.Transform.Rotation(value, axis_world, plane.Origin)
            )

    return plane, axes


def rotation_matrix_from_plane(plane):
    x = rg.Vector3d(plane.XAxis)
    y = rg.Vector3d(plane.YAxis)
    z = rg.Vector3d(plane.ZAxis)
    x.Unitize()
    y.Unitize()
    z.Unitize()
    return [
        [x.X, y.X, z.X],
        [x.Y, y.Y, z.Y],
        [x.Z, y.Z, z.Z]
    ]


def rotation_error_vector(current_plane, target_plane):
    rc = rotation_matrix_from_plane(current_plane)
    rt = rotation_matrix_from_plane(target_plane)
    r = [[0.0] * 3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            r[i][j] = sum(rt[i][k] * rc[j][k] for k in range(3))

    trace = r[0][0] + r[1][1] + r[2][2]
    angle = math.acos(clamp((trace - 1.0) * 0.5, -1.0, 1.0))
    if angle < 1e-9:
        return rg.Vector3d(0.0, 0.0, 0.0), 0.0

    if angle < math.pi - 1e-6:
        s = angle / (2.0 * math.sin(angle))
        return rg.Vector3d(
            (r[2][1] - r[1][2]) * s,
            (r[0][2] - r[2][0]) * s,
            (r[1][0] - r[0][1]) * s
        ), angle

    axis = [0.0, 0.0, 0.0]
    for i in range(3):
        axis[i] = math.sqrt(max(0.0, (r[i][i] + 1.0) * 0.5))
    if axis[0] >= axis[1] and axis[0] >= axis[2]:
        axis[1] = r[0][1] / (2.0 * axis[0] + 1e-12)
        axis[2] = r[0][2] / (2.0 * axis[0] + 1e-12)
    elif axis[1] >= axis[2]:
        axis[0] = r[0][1] / (2.0 * axis[1] + 1e-12)
        axis[2] = r[1][2] / (2.0 * axis[1] + 1e-12)
    else:
        axis[0] = r[0][2] / (2.0 * axis[2] + 1e-12)
        axis[1] = r[1][2] / (2.0 * axis[2] + 1e-12)
    return rg.Vector3d(axis[0] * angle, axis[1] * angle, axis[2] * angle), angle


def plane_error(current_plane, target_plane, rotation_weight):
    dp = target_plane.Origin - current_plane.Origin
    er, _ = rotation_error_vector(current_plane, target_plane)
    return [
        float(dp.X),
        float(dp.Y),
        float(dp.Z),
        float(er.X) * rotation_weight,
        float(er.Y) * rotation_weight,
        float(er.Z) * rotation_weight
    ]


def error_norm(err):
    return math.sqrt(sum(v * v for v in err))


def jacobian_from_axes(axes, joint_info, names, tcp_point):
    J = [[0.0 for _ in range(len(names))] for _ in range(6)]
    for col, name in enumerate(names):
        entry = axes.get(name, None)
        if entry is None:
            continue
        axis_world, joint_origin = entry
        info = joint_info.get(name, None)
        jtype = info.get("type", "revolute") if info else "revolute"
        if jtype == "prismatic":
            linear = rg.Vector3d(axis_world)
            angular = rg.Vector3d(0, 0, 0)
        else:
            linear = rg.Vector3d.CrossProduct(axis_world, tcp_point - joint_origin)
            angular = rg.Vector3d(axis_world)
        J[0][col] = float(linear.X)
        J[1][col] = float(linear.Y)
        J[2][col] = float(linear.Z)
        J[3][col] = float(angular.X)
        J[4][col] = float(angular.Y)
        J[5][col] = float(angular.Z)
    return J


def mat_transpose(A):
    return [[A[r][c] for r in range(len(A))] for c in range(len(A[0]))]


def mat_mul(A, B):
    rows = len(A)
    cols = len(B[0])
    inner = len(B)
    out = []
    for r in range(rows):
        row = []
        for c in range(cols):
            s = 0.0
            for k in range(inner):
                s += A[r][k] * B[k][c]
            row.append(s)
        out.append(row)
    return out


def mat_vec_mul(A, x):
    return [sum(a * b for a, b in zip(row, x)) for row in A]


def solve_linear_system(A, b):
    n = len(A)
    M = [list(row) for row in A]
    y = list(b)

    for k in range(n):
        pivot = k
        pivot_abs = abs(M[k][k])
        for r in range(k + 1, n):
            if abs(M[r][k]) > pivot_abs:
                pivot_abs = abs(M[r][k])
                pivot = r
        if pivot_abs < 1e-12:
            return [0.0 for _ in range(n)]
        if pivot != k:
            M[k], M[pivot] = M[pivot], M[k]
            y[k], y[pivot] = y[pivot], y[k]
        for r in range(k + 1, n):
            factor = M[r][k] / M[k][k]
            for c in range(k, n):
                M[r][c] -= factor * M[k][c]
            y[r] -= factor * y[k]

    x = [0.0 for _ in range(n)]
    for i in reversed(range(n)):
        s = y[i]
        for j in range(i + 1, n):
            s -= M[i][j] * x[j]
        if abs(M[i][i]) < 1e-12:
            x[i] = 0.0
        else:
            x[i] = s / M[i][i]
    return x


def damped_least_squares(J, err, damping):
    JT = mat_transpose(J)
    JTJ = mat_mul(JT, J)
    JTe = mat_vec_mul(JT, err)
    n = len(JTJ)
    for i in range(n):
        JTJ[i][i] += damping * damping
    return solve_linear_system(JTJ, JTe)


def active_joint_names():
    return list(MOTION_JOINT_NAMES)


def get_bounds(joint_info, names):
    lower = []
    upper = []
    for name in names:
        info = joint_info.get(name, None)
        if info is None:
            if name == "track_joint":
                lower.append(0.0)
                upper.append(1.0)
            else:
                lower.append(-math.pi)
                upper.append(math.pi)
        else:
            lower.append(info["lower"])
            upper.append(info["upper"])
    return lower, upper


def bound_q(q, lower, upper):
    return [clamp(v, lo, hi) for v, lo, hi in zip(q, lower, upper)]


def damp_track_column_if_at_limit(J, q, names):
    if not names or names[0] != "track_joint":
        return J
    track = q[0]
    if track < 0.04 or track > 0.96:
        scale = 0.10
    elif track < 0.10 or track > 0.90:
        scale = 0.35
    else:
        scale = 1.0
    if scale < 1.0:
        for r in range(len(J)):
            J[r][0] *= scale
    return J


def limit_aware_step(dq, q, names):
    if not names or names[0] != "track_joint":
        return dq
    track = q[0]
    if track <= 0.005 and dq[0] < 0.0:
        dq[0] = 0.0
    if track >= 0.995 and dq[0] > 0.0:
        dq[0] = 0.0
    if track < 0.06 and dq[0] < 0.0:
        dq[0] *= 0.15
    if track > 0.94 and dq[0] > 0.0:
        dq[0] *= 0.15
    return dq


def solve_cartesian_jog(chain, joint_info, target_tcp_m, tcp_local,
                        seed_track_m, seed_joints_rad, travel_mm, rail_mm,
                        rot_weight=0.16, max_iters=45,
                        pos_tol=0.0005, rot_tol=None):
    """
    Analytic geometric-Jacobian DLS IK (same core as Comp3 single-seed).
    Much more stable than finite-difference COMPAS FK for live jogging.
    """
    if rot_tol is None:
        rot_tol = math.radians(0.5)

    if not chain:
        return False, seed_track_m, list(seed_joints_rad[:6]), None, 999999.0

    damping = 0.05
    track_scale = track_pose_to_urdf_meters(1.0, travel_mm, rail_mm)
    names = active_joint_names()
    lower, upper = get_bounds(joint_info, names)

    q = [clamp(seed_track_m, 0.0, 1.0)] + [float(v) for v in seed_joints_rad[:6]]
    while len(q) < 7:
        q.append(0.0)
    q = bound_q(q, lower, upper)

    best_error = 999999.0
    best_track = q[0]
    best_joints = list(q[1:7])
    best_plane = None
    previous_error = None
    stagnant_count = 0

    for _ in range(max_iters):
        track_m = q[0]
        joints_rad = list(q[1:7])
        flange_plane, joint_axes = chain_fk(
            chain,
            chain_joint_values(track_m, joints_rad, travel_mm, rail_mm)
        )
        tcp_plane = tcp_plane_from_flange(flange_plane, tcp_local)
        if tcp_plane is None:
            break

        err = plane_error(tcp_plane, target_tcp_m, rot_weight)
        en = error_norm(err)
        if en < best_error:
            best_error = en
            best_track = track_m
            best_joints = list(joints_rad)
            best_plane = tcp_plane

        dp = target_tcp_m.Origin - tcp_plane.Origin
        _, rot_ang = rotation_error_vector(tcp_plane, target_tcp_m)
        if dp.Length < pos_tol and rot_ang < rot_tol:
            return True, track_m, joints_rad, tcp_plane, en

        if previous_error is not None:
            if abs(previous_error - en) < 0.00002:
                stagnant_count += 1
            else:
                stagnant_count = 0
            if stagnant_count >= 8:
                break
        previous_error = en

        J = jacobian_from_axes(joint_axes, joint_info, names, tcp_plane.Origin)
        for r in (3, 4, 5):
            for c in range(len(J[r])):
                J[r][c] *= rot_weight
        if names and names[0] == "track_joint":
            for r in range(len(J)):
                J[r][0] *= track_scale

        J = damp_track_column_if_at_limit(J, q, names)
        dq = damped_least_squares(J, err, damping)
        dq = limit_aware_step(dq, q, names)

        for i in range(len(dq)):
            if names[i] == "track_joint":
                dq[i] = clamp(dq[i], -0.030, 0.030)
            else:
                dq[i] = clamp(dq[i], -0.080, 0.080)

        for i in range(len(q)):
            q[i] += dq[i]
        q = bound_q(q, lower, upper)

    if best_plane is not None:
        dp = target_tcp_m.Origin - best_plane.Origin
        _, rot_ang = rotation_error_vector(best_plane, target_tcp_m)
        ok = dp.Length < pos_tol and rot_ang < rot_tol
    else:
        ok = False

    return ok, best_track, best_joints, best_plane, best_error



def draw_tool_meshes(tool_meshes, flange_plane):
    if not tool_meshes or flange_plane is None:
        return []

    geometry = []
    xform = rg.Transform.PlaneToPlane(rg.Plane.WorldXY, flange_plane)

    for base_mesh in tool_meshes:
        moved = base_mesh.DuplicateMesh()
        moved.Transform(xform)
        geometry.append(moved)

    return geometry


def make_box_at_plane(base_plane, center_xyz, size_xyz):
    cx, cy, cz = center_xyz
    sx, sy, sz = size_xyz

    center = (
        base_plane.Origin
        + base_plane.XAxis * cx
        + base_plane.YAxis * cy
        + base_plane.ZAxis * cz
    )

    p = rg.Plane(base_plane)
    p.Origin = center

    box = rg.Box(
        p,
        rg.Interval(-sx / 2.0, sx / 2.0),
        rg.Interval(-sy / 2.0, sy / 2.0),
        rg.Interval(-sz / 2.0, sz / 2.0)
    )

    return box.ToBrep()


def draw_track_proxy(frame_by_link, display_scale=1.0, rail_length_mm=1000.0):
    geometry = []
    s = float(display_scale)

    rail_mm = abs(to_float(rail_length_mm, 1000.0))
    if rail_mm < 1e-9:
        rail_mm = 1000.0

    rail_len_m = rail_mm / 1000.0
    rail_pad_m = 0.05

    if "track_base_link" in frame_by_link:
        base_plane = frame_to_plane(frame_by_link["track_base_link"], display_scale)
    else:
        base_plane = rg.Plane.WorldXY

    if "track_carriage_link" in frame_by_link:
        carriage_plane = frame_to_plane(frame_by_link["track_carriage_link"], display_scale)
    else:
        carriage_plane = rg.Plane.WorldXY

    geometry.append(make_box_at_plane(
        base_plane,
        ((rail_len_m * 0.5) * s, 0.0, -0.04 * s),
        ((rail_len_m + rail_pad_m) * s, 0.08 * s, 0.05 * s)
    ))

    geometry.append(make_box_at_plane(
        carriage_plane,
        (0.0, 0.0, -0.005 * s),
        (0.14 * s, 0.14 * s, 0.07 * s)
    ))

    return geometry


def draw_skeleton_lines(model, frame_by_link, display_scale=1.0):
    lines = []

    try:
        joints = model.joints
    except Exception:
        return lines

    for joint in joints:
        parent_name = link_ref_name(getattr(joint, "parent", None))
        child_name = link_ref_name(getattr(joint, "child", None))

        if parent_name in frame_by_link and child_name in frame_by_link:
            p0 = frame_point(frame_by_link[parent_name], display_scale)
            p1 = frame_point(frame_by_link[child_name], display_scale)

            if p0.DistanceTo(p1) > 1e-6:
                lines.append(rg.Line(p0, p1))

    return lines


# ------------------------------------------------------------
# RJ command
# ------------------------------------------------------------

def get_encode_settings():
    encode = gh_input("EncodeFirmwareOnSend", None)

    if encode is None and sc is not None:
        encode = sc.sticky.get("AR4_ENCODE_FIRMWARE_ON_SEND", True)

    encode = to_bool(encode, True)

    home = parse_float_list_input(gh_input("HomeJointsDeg", None), 6, None)
    if home is None and sc is not None:
        home = parse_float_list_input(sc.sticky.get("AR4_HOME_URDF_DEG", None), 6, None)
    if home is None:
        home = list(DEFAULT_HOME_URDF_DEG)

    signs = parse_float_list_input(gh_input("JointSigns", None), 6, None)
    if signs is None and sc is not None:
        signs = parse_float_list_input(sc.sticky.get("AR4_JOINT_SIGNS", None), 6, None)
    if signs is None:
        signs = list(DEFAULT_JOINT_SIGNS)

    out_signs = []

    for s in signs[:6]:
        s = to_float(s, 1.0)

        if abs(s) < 1e-12:
            s = 1.0

        out_signs.append(1.0 if s >= 0.0 else -1.0)

    return encode, home[:6], out_signs


def build_rj_command(
    joints_urdf_deg,
    j7_mm,
    speed,
    speed_type,
    accel,
    decel,
    ramp,
    wrist,
    encode,
    home_urdf_deg,
    joint_signs
):
    """
    URDF degrees in, firmware RJ string out.

    Comp5 is not in this path, so the home/sign encode happens here. Leaving
    encode False emits URDF degrees, which is only useful for comparing
    against Comp3 output.
    """
    vals = list(joints_urdf_deg[:6])

    while len(vals) < 6:
        vals.append(0.0)

    if encode:
        vals = [
            (vals[i] - home_urdf_deg[i]) * joint_signs[i]
            for i in range(6)
        ]

    if not wrist:
        wrist = "F"

    speed_type = str(speed_type).strip().lower()

    if speed_type not in ["m", "p", "s"]:
        speed_type = "p"

    return (
        "RJ"
        "A{:.3f}B{:.3f}C{:.3f}D{:.3f}E{:.3f}F{:.3f}"
        "J7{:.3f}J8{:.3f}J9{:.3f}"
        "S{}{:.3f}Ac{:.3f}Dc{:.3f}Rm{:.3f}W{}Lm{}"
    ).format(
        vals[0], vals[1], vals[2], vals[3], vals[4], vals[5],
        j7_mm, 0.0, 0.0,
        speed_type, speed, accel, decel, ramp, wrist, "000000"
    )


SERIAL_BUS_KEY = "AR4_SERIAL_BUS"


def get_connected_bus():
    if sc is None:
        return None
    bus = sc.sticky.get(SERIAL_BUS_KEY)
    if bus is None:
        return None
    try:
        if bus.is_connected():
            return bus
    except Exception:
        return None
    return None


def send_single_command(command, timeout):
    """Send one RJ through Comp1's open SerialBus. Does not open/close the port."""
    bus = get_connected_bus()
    if bus is None:
        return "", "NO_BUS"

    status_code, line = bus.send_and_wait(command, timeout, accept_any_line=False)
    if status_code == "RP":
        return line, "RP"
    if status_code.startswith("ERROR"):
        return line, "FIRMWARE_" + status_code.replace("ERROR_", "")
    return line or "", status_code


# ------------------------------------------------------------
# Capture log
# ------------------------------------------------------------

def load_captures():
    if sc is None:
        return []

    data = sc.sticky.get(CAPTURE_STICKY_KEY, None)

    return list(data) if isinstance(data, list) else []


def store_captures(captures):
    if sc is not None:
        sc.sticky[CAPTURE_STICKY_KEY] = list(captures)


def make_capture(label, joints_deg, track_m, flange_local):
    """
    One calibration sample.

    The flange pose is stored in the robot's own URDF frame, in meters, not in
    world. The TCP offset Comp7 solves for is flange-local, so keeping samples
    out of world space makes the result independent of where the robot was
    placed.
    """
    return {
        "label": label,
        "joints_deg": [float(v) for v in joints_deg[:6]],
        "track_m": float(track_m),
        "origin_m": [
            float(flange_local.Origin.X),
            float(flange_local.Origin.Y),
            float(flange_local.Origin.Z)
        ],
        "xaxis": [
            float(flange_local.XAxis.X),
            float(flange_local.XAxis.Y),
            float(flange_local.XAxis.Z)
        ],
        "yaxis": [
            float(flange_local.YAxis.X),
            float(flange_local.YAxis.Y),
            float(flange_local.YAxis.Z)
        ],
        "zaxis": [
            float(flange_local.ZAxis.X),
            float(flange_local.ZAxis.Y),
            float(flange_local.ZAxis.Z)
        ]
    }


# ------------------------------------------------------------
# Outputs / defaults
# ------------------------------------------------------------

robot_geometry = []
tool_geometry = []
track_geometry = []
skeleton_lines = []

flange_plane = rg.Plane.Unset
tcp_plane = rg.Plane.Unset
joints_urdf_deg = [0.0] * 6
xyz_mm = [0.0, 0.0, 0.0]
rpy_deg = [0.0, 0.0, 0.0]

rj_command = ""
captures = []
capture_count = 0
capture_labels = []

jog_state = {}
status = ""


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------

try:
    if sc is None:
        raise Exception("scriptcontext is not available.")

    cache_id, model, mesh_cache = find_latest_startup_cache()

    if model is None:
        raise Exception("No cached robot model found. Run the Startup component first.")

    tool_link = choose_tool_link(model)

    if not tool_link:
        raise Exception("Could not identify a tool/end-effector link.")

    base_plane, base_plane_source = resolve_base_plane(cache_id)
    set_base_xform(base_plane)

    display_scale = 1000.0

    travel_mm = to_float(gh_input("TrackLengthMM", None), None)
    if travel_mm is None or travel_mm <= 0.0:
        travel_mm = to_float(sc.sticky.get("AR4_TRACK_LENGTH_MM", 870.0), 870.0)

    rail_mm = to_float(gh_input("TrackRailLengthMM", None), None)
    if rail_mm is None or rail_mm <= 0.0:
        rail_mm = to_float(sc.sticky.get("AR4_TRACK_RAIL_LENGTH_MM", 1000.0), 1000.0)

    track_inverted = gh_input("TrackFirmwareInverted", None)
    if track_inverted is None:
        track_inverted = sc.sticky.get("AR4_TRACK_FIRMWARE_INVERTED", False)
    track_inverted = to_bool(track_inverted, False)

    mode_raw = to_text(gh_input("Mode", "joint"), "joint").strip().lower()
    if mode_raw in ["cart", "xyz", "tcp", "cartesian", "abs", "absolute", "jog"]:
        mode = "cartesian"
    else:
        mode = "joint"

    track_m = clamp(to_float(gh_input("TrackM", 0.0), 0.0), 0.0, 1.0)
    tcp_local, tcp_source = resolve_tcp_frame()

    urdf_path = get_urdf_path_from_cache(cache_id)
    fk_chain = get_fk_chain(cache_id, urdf_path, tool_link) if urdf_path else []
    joint_info = get_joint_info(cache_id, urdf_path) if urdf_path else {}

    # Seed joints from sticky so Cartesian <-> joint stays continuous.
    seed_joints = None
    if sc is not None:
        prev = sc.sticky.get("AR4_JOG_LAST_JOINTS_DEG", None)
        if isinstance(prev, (list, tuple)) and len(prev) >= 6:
            seed_joints = [to_float(v, 0.0) for v in prev[:6]]

    joint_slider_deg = [
        to_float(gh_input("J1", 0.0), 0.0),
        to_float(gh_input("J2", 0.0), 0.0),
        to_float(gh_input("J3", 0.0), 0.0),
        to_float(gh_input("J4", 0.0), 0.0),
        to_float(gh_input("J5", 0.0), 0.0),
        to_float(gh_input("J6", 0.0), 0.0)
    ]

    prev_mode = to_text(sc.sticky.get("AR4_JOG_LAST_MODE", "joint"), "joint") if sc else "joint"
    entered_cart = (mode == "cartesian" and prev_mode != "cartesian")
    if sc is not None:
        sc.sticky["AR4_JOG_LAST_MODE"] = mode

    if mode == "cartesian":
        if seed_joints is None:
            seed_joints = list(joint_slider_deg)

        seed_track = track_m
        if sc is not None:
            st = sc.sticky.get("AR4_JOG_LAST_TRACK_M", None)
            if st is not None:
                seed_track = clamp(to_float(st, track_m), 0.0, 1.0)

        if not fk_chain:
            raise Exception(
                "Cartesian jog needs the URDF path from Comp1 "
                "(sticky '{}::startup_state'). Re-run Comp1 Load/Connect.".format(cache_id)
            )

        # Freeze the exact TCP plane at mode entry (no RPY round-trip).
        if entered_cart or not isinstance(sc.sticky.get("AR4_JOG_CART_LOCK_PLANE"), rg.Plane):
            lock_joints = [math.radians(v) for v in seed_joints]
            flange_lock, _ = chain_fk(
                fk_chain,
                chain_joint_values(seed_track, lock_joints, travel_mm, rail_mm)
            )
            tcp_lock = tcp_plane_from_flange(flange_lock, tcp_local)
            sc.sticky["AR4_JOG_CART_LOCK_PLANE"] = rg.Plane(tcp_lock)
            lock_mm = scale_plane_origin(tcp_lock, display_scale)
            sc.sticky["AR4_JOG_CART_LOCK_XYZ"] = [
                float(lock_mm.Origin.X),
                float(lock_mm.Origin.Y),
                float(lock_mm.Origin.Z)
            ]
            sc.sticky["AR4_JOG_CART_LOCK_RPY"] = rpy_deg_from_plane(lock_mm)
            sc.sticky["AR4_JOG_CART_PIN0"] = [
                to_float(gh_input("X", 0.0), 0.0),
                to_float(gh_input("Y", 0.0), 0.0),
                to_float(gh_input("Z", 0.0), 0.0),
                to_float(gh_input("Rx", 0.0), 0.0),
                to_float(gh_input("Ry", 0.0), 0.0),
                to_float(gh_input("Rz", 0.0), 0.0)
            ]
            sc.sticky["AR4_JOG_CART_USE_PINS"] = False

        pin_vals = [
            to_float(gh_input("X", 0.0), 0.0),
            to_float(gh_input("Y", 0.0), 0.0),
            to_float(gh_input("Z", 0.0), 0.0),
            to_float(gh_input("Rx", 0.0), 0.0),
            to_float(gh_input("Ry", 0.0), 0.0),
            to_float(gh_input("Rz", 0.0), 0.0)
        ]
        pin0 = list(sc.sticky.get("AR4_JOG_CART_PIN0") or pin_vals)
        lock_plane = rg.Plane(sc.sticky["AR4_JOG_CART_LOCK_PLANE"])
        lock_xyz = list(sc.sticky.get("AR4_JOG_CART_LOCK_XYZ") or [0.0, 0.0, 0.0])
        lock_rpy = list(sc.sticky.get("AR4_JOG_CART_LOCK_RPY") or [0.0, 0.0, 0.0])

        if pins_changed(pin_vals, pin0):
            sc.sticky["AR4_JOG_CART_USE_PINS"] = True

        if sc.sticky.get("AR4_JOG_CART_USE_PINS", False):
            target_m = build_cartesian_target_m(
                lock_plane, pin_vals, pin0, display_scale
            )
            if pins_changed(pin_vals[:3], pin0[:3]):
                x_mm, y_mm, z_mm = pin_vals[0], pin_vals[1], pin_vals[2]
            else:
                x_mm, y_mm, z_mm = lock_xyz
            # Status shows commanded orientation = lock RPY + pin deltas.
            rx_d = lock_rpy[0] + (pin_vals[3] - pin0[3])
            ry_d = lock_rpy[1] + (pin_vals[4] - pin0[4])
            rz_d = lock_rpy[2] + (pin_vals[5] - pin0[5])
            cart_label = "TCP"
        else:
            target_m = rg.Plane(lock_plane)
            x_mm, y_mm, z_mm = lock_xyz
            rx_d, ry_d, rz_d = lock_rpy
            cart_label = "HOLD"

        ok_ik, track_m, joints_rad, _, ik_err = solve_cartesian_jog(
            fk_chain,
            joint_info,
            target_m,
            tcp_local,
            seed_track,
            [math.radians(v) for v in seed_joints],
            travel_mm,
            rail_mm
        )
        joints_deg = [math.degrees(v) for v in joints_rad]
        if not ok_ik:
            status_ik = "{} IK weak (err={:.4f}) tgt=({:.1f},{:.1f},{:.1f}/{:.1f},{:.1f},{:.1f})".format(
                cart_label, ik_err, x_mm, y_mm, z_mm, rx_d, ry_d, rz_d
            )
        else:
            status_ik = "{} IK ok (err={:.4f}) tgt=({:.1f},{:.1f},{:.1f}/{:.1f},{:.1f},{:.1f})".format(
                cart_label, ik_err, x_mm, y_mm, z_mm, rx_d, ry_d, rz_d
            )
    else:
        status_ik = "JOINT"
        if sc is not None:
            sc.sticky.pop("AR4_JOG_CART_LOCK_PLANE", None)
            sc.sticky.pop("AR4_JOG_CART_LOCK_XYZ", None)
            sc.sticky.pop("AR4_JOG_CART_LOCK_RPY", None)
            sc.sticky.pop("AR4_JOG_CART_PIN0", None)
            sc.sticky.pop("AR4_JOG_CART_USE_PINS", None)
        joints_deg = list(joint_slider_deg)
        joints_rad = [math.radians(v) for v in joints_deg]

    joints_urdf_deg = list(joints_deg)
    j7_mm = track_m_to_j7_mm(track_m, travel_mm, track_inverted)

    if sc is not None:
        sc.sticky["AR4_JOG_LAST_JOINTS_DEG"] = list(joints_deg)
        sc.sticky["AR4_JOG_LAST_TRACK_M"] = float(track_m)

    config = make_config(model, track_m, joints_rad, travel_mm, rail_mm)
    frame_by_link = get_frame_by_link(model, config)

    tool_frame = frame_by_link.get(tool_link, None)

    if tool_frame is None:
        raise Exception("FK failed for tool link '{}'.".format(tool_link))

    flange_local = frame_to_local_plane(tool_frame)
    flange_plane = frame_to_plane(tool_frame, display_scale)
    tcp_plane = apply_tcp_frame(flange_plane, tcp_local, display_scale)

    # Robot-base TCP readout (before world base transform): use local flange+tcp
    tcp_local_disp = apply_tcp_frame(flange_local, tcp_local, display_scale)
    xyz_mm = [
        float(tcp_local_disp.Origin.X),
        float(tcp_local_disp.Origin.Y),
        float(tcp_local_disp.Origin.Z)
    ]
    rpy_deg = rpy_deg_from_plane(tcp_local_disp)

    # Display.
    tool_meshes = get_tool_meshes()
    hide_gripper = to_bool(
        gh_input("HideGripper", bool(tool_meshes)),
        bool(tool_meshes)
    )

    if to_bool(gh_input("DrawRobot", True), True):
        robot_geometry = draw_robot_meshes(
            mesh_cache,
            frame_by_link,
            display_scale,
            exclude_links=GRIPPER_LINK_NAMES if hide_gripper else ()
        )

    if tool_meshes:
        tool_geometry = draw_tool_meshes(tool_meshes, flange_plane)

    if to_bool(gh_input("DrawTrack", True), True):
        track_geometry = draw_track_proxy(frame_by_link, display_scale, rail_mm)

    if to_bool(gh_input("DrawSkeleton", False), False):
        skeleton_lines = draw_skeleton_lines(model, frame_by_link, display_scale)

    # Command for the current pose.
    encode, home_urdf_deg, joint_signs = get_encode_settings()

    rj_command = build_rj_command(
        joints_urdf_deg=joints_deg,
        j7_mm=j7_mm,
        speed=to_float(gh_input("Speed", 15.0), 15.0),
        speed_type=to_text(gh_input("SpeedType", "p"), "p"),
        accel=to_float(gh_input("Accel", 10.0), 10.0),
        decel=to_float(gh_input("Decel", 10.0), 10.0),
        ramp=to_float(gh_input("Ramp", 10.0), 10.0),
        wrist=to_text(gh_input("Wrist", "F"), "F"),
        encode=encode,
        home_urdf_deg=home_urdf_deg,
        joint_signs=joint_signs
    )

    # Capture log.
    captures = load_captures()

    if edge_triggered("AR4_JOG_CLEAR_EDGE", to_bool(gh_input("ClearCaptures", False), False)):
        captures = []
        store_captures(captures)
        capture_note = "capture log cleared"

    elif edge_triggered("AR4_JOG_CAPTURE_EDGE", to_bool(gh_input("Capture", False), False)):
        label = to_text(gh_input("CaptureLabel", ""), "")

        if not label:
            label = "pose_{}".format(len(captures) + 1)

        captures.append(make_capture(label, joints_deg, track_m, flange_local))
        store_captures(captures)
        capture_note = "captured '{}'".format(label)

    else:
        capture_note = "no capture this solve"

    capture_count = len(captures)
    capture_labels = [c.get("label", "") for c in captures]

    # Send through Comp1's open bus.
    send_requested = to_bool(gh_input("Send", False), False)

    if edge_triggered("AR4_JOG_SEND_EDGE", send_requested):
        reply, reply_status = send_single_command(
            rj_command,
            to_float(gh_input("SendTimeout", 30.0), 30.0)
        )
        if reply_status == "NO_BUS":
            send_note = "Send ignored: Comp1 Connect is False"
        else:
            send_note = "sent, reply {}".format(reply_status)
            sc.sticky["AR4_JOG_LAST_REPLY"] = reply
    elif send_requested:
        send_note = "Send held True; toggle False then True to move again"
    else:
        send_note = "Send off"

    jog_state = {
        "cache_id": cache_id,
        "tool_link": tool_link,
        "mode": mode,
        "joints_urdf_deg": list(joints_deg),
        "xyz_mm": list(xyz_mm),
        "rpy_deg": list(rpy_deg),
        "track_m": track_m,
        "j7_mm": j7_mm,
        "flange_origin_m": [
            flange_local.Origin.X,
            flange_local.Origin.Y,
            flange_local.Origin.Z
        ],
        "tcp_source": tcp_source,
        "base_plane": base_plane,
        "base_plane_source": base_plane_source,
        "encode_firmware_on_send": encode,
        "home_urdf_deg": list(home_urdf_deg),
        "joint_signs": list(joint_signs),
        "capture_count": capture_count,
        "capture_labels": list(capture_labels),
        "tool_mesh_count": len(tool_meshes),
        "gripper_hidden": bool(hide_gripper)
    }

    status = (
        "Comp6 {} | {} | J {} | XYZ {} | RPY {} | TrackM {:.4f} | "
        "TCP {} | encode {} | captures {} | {} | {}"
    ).format(
        mode,
        status_ik,
        [round(v, 2) for v in joints_deg],
        [round(v, 1) for v in xyz_mm],
        [round(v, 1) for v in rpy_deg],
        track_m,
        tcp_source,
        "ON" if encode else "OFF",
        capture_count,
        capture_note,
        send_note
    )

except Exception as e:
    robot_geometry = []
    tool_geometry = []
    track_geometry = []
    skeleton_lines = []

    flange_plane = rg.Plane.Unset
    tcp_plane = rg.Plane.Unset

    rj_command = ""
    captures = []
    capture_count = 0
    capture_labels = []

    jog_state = {}
    status = "Comp6 jog ERROR: {}".format(e)
