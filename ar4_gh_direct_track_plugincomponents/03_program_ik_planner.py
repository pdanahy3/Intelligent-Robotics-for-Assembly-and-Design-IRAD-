#! python3
# r: compas
# r: compas_fab
# r: compas_robots

"""
AR4 + External Track — Component 3: Program / IK Planner
Calibration-frame aware replacement.

PRIMARY PURPOSE
---------------
IK solves and emits RJ commands in the URDF / COMPAS geometric joint frame.
Program output and Comp4 simulation share the same RJ numbers (no firmware
home offset baked into the command string).

Firmware joint zero on this robot is NOT URDF zero. Measured true-home gold:
    Program / sim RJ : A0   B0 C0 D0 E45 F0
    Physical robot   : A-16 B0 C0 D0 E23 F10
    => HomeJointsDeg = [-16, 0, 0, 0, 22, -10], JointSigns = [-1, 1, 1, 1, 1, 1]

Those offsets are applied only when Comp5 sends to the robot:
    firmware_deg = (urdf_deg - home_urdf_deg) * joint_sign

UseCalibrationFrame in this component defaults False so rj_commands stay in
URDF/sim space. HomeJointsDeg is still stored to sticky for the Comp5 encoder.

Rhino / Grasshopper world units for this project are millimeters.
URDF / COMPAS are meters. Targets and the TCP frame are converted mm -> m.

Track convention (shared across Comp2/3/4/5) - simulation / URDF is the source of truth:
    Logical TrackM = 0.0  -> motor / home-switch side
    Logical TrackM = 1.0  -> far end of usable travel
    Firmware: J7 = TrackM * TrackLengthMM   (usable length, default 870)
    URDF prismatic meters on full rail:
        track_joint = TrackM * (TrackLengthMM / TrackRailLengthMM)
        e.g. TrackM=1 -> 0.87 m on a 1000 mm rail

TrackFirmwareInverted is obsolete when MotDir/CalDir match the sim (J7=0 at motor).
Do NOT leave sticky AR4_TRACK_FIRMWARE_INVERTED=True — that inverts J7 in RJ while
sim still uses logical TrackM, which mirrors coordinated TCP paths on the robot.


INPUTS — original required inputs
---------------------------------
cache_id            : from Startup component
Targets             : TCP target planes, points, or plane-like values (mm origins)
Speed               : arm move speed value, normally firmware Sm value
Accel               : firmware Ac percent
Decel               : firmware Dc percent
Ramp                : firmware Rm value
Wrist               : firmware W token, usually "F"
UseTrack            : bool, solve with track_joint enabled
Solve               : bool edge trigger; toggle False -> True to recompute

TCP INPUTS — replaces the old scalar ToolZ
------------------------------------------
The TCP frame is the rigid transform from the tool/flange link to the tool
tip, authored in flange-local Rhino units. It carries orientation, not just
offset, so the IK aligns the real tool frame with the target plane.

Highest wins:
UseCalibratedTCP    : bool, default True. Use the frame Comp7 logged to
                      sticky AR4_TCP_FRAME. Set False to ignore it.
TCPPlane            : plane. Full tool frame relative to the flange.
TCPPoint            : point. Tool tip position relative to the flange. Fixes
                      where the tip is but not how it is turned, so target
                      plane orientation still resolves against flange axes.
ToolZ               : legacy scalar mm along flange Z. Kept so old definitions
                      keep working; prefer TCPPlane or a Comp7 calibration.
Lowest of all is the nominal TCP Comp1 loaded alongside the tool mesh, so a
definition that declares its tool once in Comp1 needs no TCP pins here.

OPTIONAL INPUTS — add these to the GH Python component when needed
-----------------------------------------------------------------
PosTolMM            : default 0.5. Position convergence limit, Rhino units.
RotTolDeg           : default 0.5. Orientation convergence limit, degrees.
                      A target now only counts as solved when BOTH are met.
MaxSeeds            : default 48. Cap on reposture retries per target when the
                      sequential seed fails. The generator can produce ~126;
                      lower this if a hard path is holding up the solve.
RotationWeight      : default 0.16. Meters-per-radian trade inside the least
                      squares step. Raise it if orientation lags position,
                      lower it if the solver fights itself near singularities.
UseCalibrationFrame : bool, default False. Keep False so rj_commands match sim
                      (URDF joint degrees in the RJ string). Comp5 encodes to
                      firmware using HomeJointsDeg when sending.
HomeJointsDeg       : list/string of 6 URDF degrees at firmware A0..F0.
                      Default [16,0,0,0,22,-10]. Stored for Comp5 encode-on-send.
                      Only applied to rj_commands here if UseCalibrationFrame=True.
JointSigns          : list/string of 6 signs, default 1,1,1,1,1,1.
                      Use -1 where MotDir / URDF axis sense opposes firmware +.
SplitTrackMotion    : bool, default False. Keep False for synchronized arm+J7
                      moves so TCP interpolates through targets smoothly.
                      Set True only if you need a J7-only preposition before the
                      arm move (legacy; breaks continuous TCP motion).
TrackSpeedPercent   : default 100.0. Used only when SplitTrackMotion=True.
MaxSolveSeconds     : default 300.0. Hard abort for difficult paths.
TrackMoveEpsilonMM  : default 0.5. Minimum J7 change before split preposition.
TrackFirmwareInverted : bool. Direction mapping from URDF/sim rail to firmware J7.
                        False: J7 = TrackM * TrackLengthMM
                        True:  J7 = (1 - TrackM) * TrackLengthMM
                        Set True when the rail is mounted so firmware J7=0 sits at
                        the HIGH-X end of the Rhino/URDF rail. Symptom of a wrong
                        setting: sim is correct and single targets are correct, but
                        coordinated arm+track paths run mirrored across the track
                        axis on the real robot (error grows with track travel).
TrackLengthMM       : default 870.0. Usable robot travel; must match firmware J7 span.
TrackRailLengthMM   : default 1000.0. Full rail visual length in Rhino/URDF meters*1000.
                      IK maps TrackM 0..1 onto 0..(Travel/Rail) meters on the rail.
TargetUnitsMM       : bool, default True. Scale Rhino target origins and the
                      TCP frame from mm to meters.

OUTPUTS — original outputs
--------------------------
solved
program_joint_values_rad       : URDF / true joint radians per target
program_joint_values_deg       : URDF / true joint degrees per target
program_track_m                : logical TrackM per target (0..1 of usable travel)
program_j7_mm                  : firmware J7 mm per target
rj_commands                    : expanded firmware command list
ik_errors                      : blended least-squares norm per target
tcp_pos_error_mm               : position miss per target, Rhino units
tcp_rot_error_deg              : orientation miss per target, degrees
target_planes_robot
tcp_planes_solved
program_state
status
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


# ------------------------------------------------------------
# Helpers: Grasshopper values
# ------------------------------------------------------------

def unwrap_gh(value):
    if value is None:
        return None
    if hasattr(value, "Value"):
        return value.Value
    return value


def get_optional_input(name, default=None):
    return globals().get(name, default)


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


def parse_float_list_input(value, count=None, default=None):
    """
    Accepts GH lists, tuples, panels like "1,2,3", or semicolon/space text.
    Returns list[float] or default.
    """
    value = unwrap_gh(value)
    if value is None:
        return default

    raw = []

    if isinstance(value, str):
        text = value.replace(";", ",").replace(" ", ",")
        parts = [p for p in text.split(",") if p.strip()]
        raw = parts
    else:
        try:
            raw = list(value)
        except TypeError:
            raw = [value]

    vals = []
    for item in raw:
        try:
            vals.append(float(unwrap_gh(item)))
        except Exception:
            try:
                vals.append(float(str(item).strip()))
            except Exception:
                pass

    if count is not None:
        if len(vals) < count:
            return default
        vals = vals[:count]

    if not vals:
        return default

    return vals


def flatten_targets(values):
    """
    Accepts:
      - a single Rhino Plane
      - a GH list of Rhino Planes
      - Rhino Points, converted to WorldXY-aligned planes
      - simple wrappers with a .Plane property

    Returns:
      list[rg.Plane]
    """
    if values is None:
        return []

    values = unwrap_gh(values)

    if isinstance(values, rg.Plane):
        return [values]

    try:
        raw = list(values)
    except TypeError:
        raw = [values]

    result = []

    for item in raw:
        item = unwrap_gh(item)

        if isinstance(item, rg.Plane):
            result.append(item)

        elif isinstance(item, rg.Point3d):
            result.append(
                rg.Plane(
                    item,
                    rg.Vector3d.XAxis,
                    rg.Vector3d.YAxis
                )
            )

        elif hasattr(item, "Plane") and isinstance(item.Plane, rg.Plane):
            result.append(item.Plane)

    return result


# ------------------------------------------------------------
# Constants
# ------------------------------------------------------------

MOTION_JOINT_NAMES = [
    "track_joint",
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6"
]

# Measured on this AR4: URDF pose when firmware reports A0..F0.
# Stored for Comp5, which applies: firmware = (URDF - HomeURDF) * JointSigns.
# Not baked into Comp3 RJ by default.
#   program/sim RJ A0 B0 C0 D0 E45 F0  ==  physical A-16 B0 C0 D0 E23 F10
#   program/sim joint_1 -36            ==  physical A+20 (both reach toward +X)
# J1's sign is negative: URDF joint_1 turns opposite to firmware A.
DEFAULT_HOME_URDF_DEG = [-16.0, 0.0, 0.0, 0.0, 22.0, -10.0]
DEFAULT_JOINT_SIGNS = [-1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
# False: rj_commands stay in URDF/sim space; Comp5 encodes for the robot.
DEFAULT_USE_CALIBRATION_FRAME = False


# ------------------------------------------------------------
# Deadline / timeout
# ------------------------------------------------------------

def check_deadline(deadline_time):
    if deadline_time is None:
        return
    if time.time() > deadline_time:
        raise Exception("Program solve exceeded MaxSolveSeconds and was aborted.")


# ------------------------------------------------------------
# URDF parsing and caching
# ------------------------------------------------------------

def parse_float_list(text, default_values):
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


def get_urdf_path_from_cache(cache_id):
    """
    Startup component stores:
      <cache_id>::startup_state

    This retrieves the URDF path from that startup state.
    """
    if sc is None:
        return ""

    state = sc.sticky.get(cache_id + "::startup_state", None)

    if isinstance(state, dict):
        return state.get("urdf", "")

    return ""


def parse_urdf_motion_joints_uncached(urdf_path):
    """
    Reads the actual motion joint data from the URDF:
      - joint type
      - parent link
      - child link
      - origin xyz/rpy
      - axis
      - limits

    This is used to build the geometric Jacobian from the real skeleton.
    """
    data = {}

    if not urdf_path or not os.path.exists(urdf_path):
        return data

    root = ET.parse(urdf_path).getroot()

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

        parent_link = parent.attrib.get("link", "") if parent is not None else ""
        child_link = child.attrib.get("link", "") if child is not None else ""

        xyz = [0.0, 0.0, 0.0]
        rpy = [0.0, 0.0, 0.0]

        if origin is not None:
            xyz = parse_float_list(
                origin.attrib.get("xyz", None),
                [0.0, 0.0, 0.0]
            )

            rpy = parse_float_list(
                origin.attrib.get("rpy", None),
                [0.0, 0.0, 0.0]
            )

        axis_xyz = [0.0, 0.0, 0.0]

        if axis is not None:
            axis_xyz = parse_float_list(
                axis.attrib.get("xyz", None),
                [0.0, 0.0, 0.0]
            )

        lower = -math.pi
        upper = math.pi

        if name == "track_joint":
            lower = 0.0
            upper = 1.0

        if limit is not None:
            try:
                lower = float(limit.attrib.get("lower", lower))
            except Exception:
                pass

            try:
                upper = float(limit.attrib.get("upper", upper))
            except Exception:
                pass

        data[name] = {
            "name": name,
            "type": jtype,
            "parent": parent_link,
            "child": child_link,
            "xyz": xyz,
            "rpy": rpy,
            "axis": axis_xyz,
            "lower": lower,
            "upper": upper
        }

    return data


def get_joint_info(cache_id, urdf_path):
    """
    Cached URDF joint info. Cache key includes the URDF path so swapping URDFs
    does not accidentally keep stale track_joint data.
    """
    if sc is None:
        return {}

    key = cache_id + "::motion_joint_info::" + str(urdf_path)

    cached = sc.sticky.get(key, None)

    if isinstance(cached, dict) and cached:
        return cached

    joint_info = parse_urdf_motion_joints_uncached(urdf_path)
    sc.sticky[key] = joint_info

    return joint_info


def parse_urdf_chain_uncached(urdf_path, tool_link):
    """
    Ordered root -> tool_link joint chain, fixed joints included.

    The IK walks this itself instead of asking COMPAS for one link frame at a
    time. Fixed joints have to be in it because they carry real offsets.
    """
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

    # Walk up from the tool to the root, then reverse.
    while link in joint_by_child and guard < 256:
        guard += 1

        joint = joint_by_child[link]
        origin = joint.find("origin")
        axis = joint.find("axis")
        parent = joint.find("parent")

        xyz = [0.0, 0.0, 0.0]
        rpy = [0.0, 0.0, 0.0]

        if origin is not None:
            xyz = parse_float_list(origin.attrib.get("xyz", None), [0.0, 0.0, 0.0])
            rpy = parse_float_list(origin.attrib.get("rpy", None), [0.0, 0.0, 0.0])

        axis_xyz = [0.0, 0.0, 1.0]

        if axis is not None:
            axis_xyz = parse_float_list(axis.attrib.get("xyz", None), [0.0, 0.0, 1.0])

        chain.append({
            "name": joint.attrib.get("name", ""),
            "type": joint.attrib.get("type", "fixed"),
            "xyz": xyz,
            "rpy": rpy,
            "axis": axis_xyz,
            "zero_rpy": abs(rpy[0]) < 1e-15 and abs(rpy[1]) < 1e-15 and abs(rpy[2]) < 1e-15
        })

        link = parent.attrib.get("link", "") if parent is not None else ""

    chain.reverse()

    return chain


def get_fk_chain(cache_id, urdf_path, tool_link):
    if sc is None:
        return parse_urdf_chain_uncached(urdf_path, tool_link)

    key = "{}::fk_chain::{}::{}".format(cache_id, urdf_path, tool_link)
    cached = sc.sticky.get(key, None)

    if isinstance(cached, list) and cached:
        return cached

    chain = parse_urdf_chain_uncached(urdf_path, tool_link)
    sc.sticky[key] = chain

    return chain


# ------------------------------------------------------------
# Rhino FK helpers
#
# FK is done by chain_fk walking the parsed URDF chain, not by COMPAS.
# COMPAS forward_kinematics re-walks the tree from the root per call, so
# building a 7 column Jacobian cost eight full walks per IK iteration.
# ------------------------------------------------------------

def choose_tool_link(model):
    """
    Choose the end-effector link for the TCP frame.
    Adjust this list if your URDF has a specific final tool link.
    """
    try:
        link_names = [link.name for link in model.links]
    except Exception:
        link_names = []

    preferred = [
        "ee_link",
        "tool0",
        "tool_link",
        "pneumatic_gripper_tool_link",
        "link_6"
    ]

    for name in preferred:
        if name in link_names:
            return name

    if link_names:
        return link_names[-1]

    return None


def logical_track_m_to_urdf_m(track_m, travel_mm=870.0, rail_mm=1000.0):
    """
    Logical TrackM 0..1 (fraction of usable travel) -> URDF prismatic meters.
    """
    travel_mm = abs(to_float(travel_mm, 870.0))
    rail_mm = abs(to_float(rail_mm, 1000.0))
    if travel_mm < 1e-9:
        travel_mm = 870.0
    if rail_mm < 1e-9:
        rail_mm = 1000.0
    return clamp(to_float(track_m, 0.0), 0.0, 1.0) * (travel_mm / rail_mm)


def offset_plane_along_z(plane, distance):
    """Legacy ToolZ helper: slide a plane along its own Z, keeping its axes."""
    p = rg.Plane(plane)
    z = rg.Vector3d(p.ZAxis)

    if z.Length > 1e-12:
        z.Unitize()

    p.Origin = p.Origin + z * distance

    return p


# ------------------------------------------------------------
# TCP frame
#
# A TCP frame is the rigid transform from the tool/flange link to the tool
# tip, expressed in flange-local coordinates with the origin in URDF meters.
# It is carried between components as a plain dict so it survives sticky.
# ------------------------------------------------------------

TCP_IDENTITY = {
    "origin_m": [0.0, 0.0, 0.0],
    "xaxis": [1.0, 0.0, 0.0],
    "yaxis": [0.0, 1.0, 0.0],
    "source": "flange (no tool)"
}


def tcp_dict_to_plane(tcp):
    """Flange-local TCP dict -> rg.Plane in URDF meters."""
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


def plane_to_tcp_dict(plane, source):
    """rg.Plane in URDF meters -> flange-local TCP dict."""
    return {
        "origin_m": [
            float(plane.Origin.X),
            float(plane.Origin.Y),
            float(plane.Origin.Z)
        ],
        "xaxis": [
            float(plane.XAxis.X),
            float(plane.XAxis.Y),
            float(plane.XAxis.Z)
        ],
        "yaxis": [
            float(plane.YAxis.X),
            float(plane.YAxis.Y),
            float(plane.YAxis.Z)
        ],
        "source": source
    }


def resolve_tcp_frame(target_scale):
    """
    Returns (flange-local TCP plane in URDF meters, TCP dict).

    Precedence:
      calibrated frame logged by Comp7, unless UseCalibratedTCP is False
      TCPPlane : full tool frame authored in Rhino units, flange-local
      TCPPoint : tool tip position only, axes stay parallel to the flange
      ToolZ    : legacy scalar offset along flange Z
      the nominal TCP Comp1 loaded with the tool

    A TCPPoint fixes where the tool tip is but not how it is turned, so the
    solver still aligns flange axes to the target. Only TCPPlane or a Comp7
    calibration give the target plane's orientation any meaning.
    """
    use_calibrated = to_bool(get_optional_input("UseCalibratedTCP", True), True)

    if use_calibrated and sc is not None:
        cal = sc.sticky.get("AR4_TCP_FRAME", None)

        if isinstance(cal, dict):
            return tcp_dict_to_plane(cal), dict(cal)

    plane = to_plane(get_optional_input("TCPPlane", None), None)

    if plane is not None:
        local = scale_plane_origin(plane, target_scale)
        return local, plane_to_tcp_dict(local, "TCPPlane input")

    point = to_plane(get_optional_input("TCPPoint", None), None)

    if point is not None:
        local = scale_plane_origin(
            rg.Plane(point.Origin, rg.Vector3d.XAxis, rg.Vector3d.YAxis),
            target_scale
        )
        return local, plane_to_tcp_dict(local, "TCPPoint input (no orientation)")

    tool_z_mm = to_float(get_optional_input("ToolZ", None), None)

    if tool_z_mm is not None and abs(tool_z_mm) > 1e-12:
        local = offset_plane_along_z(rg.Plane.WorldXY, tool_z_mm * target_scale)
        return local, plane_to_tcp_dict(
            local,
            "ToolZ legacy ({:.1f} mm)".format(tool_z_mm)
        )

    if sc is not None:
        nominal = sc.sticky.get("AR4_TOOL_TCP_FRAME", None)

        if isinstance(nominal, dict):
            return tcp_dict_to_plane(nominal), dict(nominal)

    return rg.Plane.WorldXY, dict(TCP_IDENTITY)


def plane_origin_transform(parent_plane, xyz, rpy):
    """
    Converts a URDF joint origin into a world/model-space Rhino Plane.

    URDF joint origin:
      xyz is translated in the parent link frame
      rpy rotates the joint frame relative to the parent link frame

    rpy is fixed-axis roll/pitch/yaw, meaning R = Rz(yaw) Ry(pitch) Rx(roll),
    so all three rotations are taken about the PARENT's axes. Rotating about
    axes updated by the previous step instead gives Rx Ry Rz, which differs
    as soon as two components are nonzero -- true here for joint_2, joint_4
    and joint_5, each of which carries both a roll and a yaw.
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


def axis_world_from_joint_plane(joint_plane, axis_xyz):
    """
    Converts the URDF joint axis from the local joint frame into world/model space.
    """
    v = (
        rg.Vector3d(joint_plane.XAxis) * axis_xyz[0]
        + rg.Vector3d(joint_plane.YAxis) * axis_xyz[1]
        + rg.Vector3d(joint_plane.ZAxis) * axis_xyz[2]
    )

    if v.Length > 1e-12:
        v.Unitize()

    return v


# ------------------------------------------------------------
# Target scaling
# ------------------------------------------------------------

def guess_target_scale(targets, force_mm=True):
    """
    The URDF is in meters. Rhino targets in this project are millimeters.

    force_mm=True (default): always convert origins mm -> m.
    force_mm=False: heuristic fallback if |xyz| > 5 treat as mm.
    """
    if force_mm:
        return 0.001

    max_abs = 0.0

    for pl in targets:
        p = pl.Origin
        max_abs = max(max_abs, abs(p.X), abs(p.Y), abs(p.Z))

    if max_abs > 5.0:
        return 0.001

    return 1.0


def scale_plane_origin(plane, scale):
    p = rg.Plane(plane)

    if abs(scale - 1.0) < 1e-12:
        return p

    p.Origin = rg.Point3d(
        p.Origin.X * scale,
        p.Origin.Y * scale,
        p.Origin.Z * scale
    )

    return p


# ------------------------------------------------------------
# Robot base plane
# ------------------------------------------------------------

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


def resolve_base_plane(cache_id):
    """
    Robot base plane in Rhino units: where the URDF 'world' link sits.

    BasePlane pin wins, then the per-cache plane stored by Comp1, then WorldXY.
    Wire the pin when several robots share one URDF, since they also share the
    cache_id and therefore the sticky fallback.
    """
    plane = to_plane(get_optional_input("BasePlane", None), None)

    if plane is not None:
        return plane, "BasePlane input"

    if sc is not None and cache_id:
        plane = to_plane(sc.sticky.get(cache_id + "::base_plane", None), None)

        if plane is not None:
            return plane, "Comp1 startup base_plane"

        state = sc.sticky.get(cache_id + "::startup_state", None)

        if isinstance(state, dict):
            plane = to_plane(state.get("base_plane", None), None)

            if plane is not None:
                return plane, "Comp1 startup_state"

    return rg.Plane.WorldXY, "world XY"


def transform_plane(plane, xform):
    if plane is None or xform is None:
        return plane

    if not isinstance(plane, rg.Plane):
        return plane

    p = rg.Plane(plane)
    p.Transform(xform)
    return p


def base_plane_transforms(base_plane):
    """
    Returns (world_to_local, local_to_world), or (None, None) for WorldXY.

    IK and FK stay in the robot's own URDF frame. Targets come in world and are
    mapped to that frame; solved planes are mapped back out for display. Joint
    values and RJ commands are frame independent, so nothing downstream changes.
    """
    if is_world_plane(base_plane):
        return None, None

    return (
        rg.Transform.PlaneToPlane(base_plane, rg.Plane.WorldXY),
        rg.Transform.PlaneToPlane(rg.Plane.WorldXY, base_plane)
    )


def planes_to_world_display(planes, display_scale, local_to_world):
    """Robot-local URDF-meter planes -> Rhino display units in world space."""
    out = []

    for pl in planes:
        if isinstance(pl, rg.Plane):
            out.append(
                transform_plane(
                    scale_plane_origin(pl, display_scale),
                    local_to_world
                )
            )
        else:
            out.append(pl)

    return out


# ------------------------------------------------------------
# Error and Jacobian
# ------------------------------------------------------------

def rotation_matrix_from_plane(plane):
    """Columns are the unitized plane axes."""
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
    """
    Returns (axis * angle, angle) taking the current orientation onto the
    target, in model coordinates.

    The older half-sum-of-cross-products form decays past 90 degrees and
    reaches exactly zero at 180, so a badly oriented seed reported no
    orientation error and the solver never rotated the tool. Axis-angle stays
    a valid descent direction over the full range.
    """
    rc = rotation_matrix_from_plane(current_plane)
    rt = rotation_matrix_from_plane(target_plane)

    # R = target * current^T
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

    # At 180 degrees the skew part vanishes; take the axis from the longest
    # column of R + I instead.
    axis = rg.Vector3d(0.0, 0.0, 1.0)
    best_len = -1.0

    for j in range(3):
        v = rg.Vector3d(
            r[0][j] + (1.0 if j == 0 else 0.0),
            r[1][j] + (1.0 if j == 1 else 0.0),
            r[2][j] + (1.0 if j == 2 else 0.0)
        )

        if v.Length > best_len:
            best_len = v.Length
            axis = v

    if axis.Length > 1e-12:
        axis.Unitize()

    return axis * angle, angle


def pose_error_parts(current_plane, target_plane):
    """(position error in model meters, orientation error in radians)."""
    dp = target_plane.Origin - current_plane.Origin
    _, angle = rotation_error_vector(current_plane, target_plane)

    return dp.Length, angle


def plane_error(current_plane, target_plane, rotation_weight):
    """
    6D error: position delta in meters, then axis-angle orientation error
    scaled by rotation_weight.

    rotation_weight converts radians into the same least-squares space as
    meters, so the Jacobian's angular rows must be scaled by it as well.
    """
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


def chain_joint_values(track_m, joints_rad, travel_mm, rail_mm):
    """Joint name -> URDF value, for chain_fk."""
    values = {
        "track_joint": logical_track_m_to_urdf_m(track_m, travel_mm, rail_mm)
    }

    for i in range(6):
        values["joint_{}".format(i + 1)] = joints_rad[i]

    return values


def chain_fk(chain, values):
    """
    One root-to-tool pass giving the flange plane and every movable joint's
    world axis and origin.

    This replaces a COMPAS forward_kinematics call per link. Those each
    re-walked the tree from the root, so building a 7 column Jacobian cost
    eight full tree walks per iteration; this costs one.
    """
    plane = rg.Plane.WorldXY
    axes = {}

    for entry in chain:
        plane = plane_origin_transform(plane, entry["xyz"], entry["rpy"])
        jtype = entry["type"]

        if jtype == "fixed":
            continue

        axis_world = axis_world_from_joint_plane(plane, entry["axis"])

        # A joint's own motion moves neither its axis nor, for revolute, its
        # origin, so these are the values the Jacobian needs.
        axes[entry["name"]] = (axis_world, rg.Point3d(plane.Origin))

        value = values.get(entry["name"], 0.0)

        if jtype == "prismatic":
            plane.Origin = plane.Origin + axis_world * value

        elif abs(value) > 1e-15:
            plane.Transform(
                rg.Transform.Rotation(value, axis_world, plane.Origin)
            )

    return plane, axes


def tcp_plane_from_flange(flange_plane, tcp_local):
    if tcp_local is None:
        return flange_plane

    p = rg.Plane(tcp_local)
    p.Transform(rg.Transform.PlaneToPlane(rg.Plane.WorldXY, flange_plane))

    return p


def jacobian_from_axes(axes, joint_info, active_joint_names, tcp_point):
    """6 x n geometric Jacobian from one chain_fk pass."""
    J = [[0.0 for _ in range(len(active_joint_names))] for _ in range(6)]

    for col, name in enumerate(active_joint_names):
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


# ------------------------------------------------------------
# Linear algebra
# ------------------------------------------------------------

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
    """
    Gaussian elimination with partial pivoting.
    """
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
    """
    dq = inverse(JT J + lambda^2 I) JT err
    """
    JT = mat_transpose(J)
    JTJ = mat_mul(JT, J)
    JTe = mat_vec_mul(JT, err)

    n = len(JTJ)

    for i in range(n):
        JTJ[i][i] += damping * damping

    return solve_linear_system(JTJ, JTe)


# ------------------------------------------------------------
# Bounds and active variables
# ------------------------------------------------------------

def active_joint_names(use_track):
    if use_track:
        return list(MOTION_JOINT_NAMES)

    return MOTION_JOINT_NAMES[1:]


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


def unpack_q(q, fixed_track_m, use_track):
    if use_track:
        track_m = q[0]
        joints = list(q[1:7])
    else:
        track_m = fixed_track_m
        joints = list(q[:6])

    track_m = clamp(track_m, 0.0, 1.0)

    while len(joints) < 6:
        joints.append(0.0)

    return track_m, joints[:6]


def angle_wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi

    while a < -math.pi:
        a += 2.0 * math.pi

    return a


# ------------------------------------------------------------
# Track-aware IK helpers
# ------------------------------------------------------------

def make_reposture_seeds(seed_track_m, seed_joints_rad, use_track):
    """
    Generate alternate seeds when the track reaches a limit.

    The target plane stays the same, but we retry with:
      - track pulled slightly away from its limit
      - alternate J1/J2/J3 postures
    """
    seeds = []

    base_joints = list(seed_joints_rad[:6])

    while len(base_joints) < 6:
        base_joints.append(0.0)

    seed_track_m = clamp(seed_track_m, 0.0, 1.0)

    if use_track:
        if seed_track_m < 0.12:
            track_candidates = [
                seed_track_m,
                0.05,
                0.12,
                0.20,
                0.35,
                0.50
            ]

        elif seed_track_m > 0.88:
            track_candidates = [
                seed_track_m,
                0.95,
                0.88,
                0.80,
                0.65,
                0.50
            ]

        else:
            track_candidates = [
                seed_track_m,
                clamp(seed_track_m - 0.15, 0.0, 1.0),
                clamp(seed_track_m + 0.15, 0.0, 1.0),
                0.50
            ]

    else:
        track_candidates = [seed_track_m]

    deg_offsets = [
        (0,    0,    0),

        (10,   0,    0),
        (-10,  0,    0),
        (20,   0,    0),
        (-20,  0,    0),
        (35,   0,    0),
        (-35,  0,    0),

        (0,    10,   0),
        (0,   -10,   0),
        (0,    20,   0),
        (0,   -20,   0),

        (0,    0,    10),
        (0,    0,   -10),
        (0,    0,    20),
        (0,    0,   -20),

        (20,   10,  -10),
        (-20,  10,  -10),
        (20,  -10,   10),
        (-20, -10,   10),

        (45,   0,    0),
        (-45,  0,    0)
    ]

    for track in track_candidates:
        for d1, d2, d3 in deg_offsets:
            joints = list(base_joints)

            joints[0] = angle_wrap_pi(joints[0] + math.radians(d1))
            joints[1] = angle_wrap_pi(joints[1] + math.radians(d2))
            joints[2] = angle_wrap_pi(joints[2] + math.radians(d3))

            seeds.append((track, joints))

    unique = []
    seen = set()

    for track, joints in seeds:
        key = (
            round(track, 3),
            round(joints[0], 3),
            round(joints[1], 3),
            round(joints[2], 3)
        )

        if key not in seen:
            seen.add(key)
            unique.append((track, joints))

    return unique


def damp_track_column_if_at_limit(J, q, names):
    """
    If the track is near either limit, reduce the track column in the Jacobian.
    This forces the solver to rely more on J1-J6.
    """
    if not names:
        return J

    if names[0] != "track_joint":
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
    """
    Prevent the track update from pushing farther into a hard limit.
    The arm joints remain free.
    """
    if not names:
        return dq

    if names[0] != "track_joint":
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


# ------------------------------------------------------------
# IK solver
# ------------------------------------------------------------

def solve_target_single_seed(
    chain,
    joint_info,
    target_plane,
    seed_track_m,
    seed_joints_rad,
    tcp_local,
    use_track,
    max_iter,
    pos_tol,
    rot_tol,
    rot_weight,
    deadline_time,
    travel_mm=870.0,
    rail_mm=1000.0
):
    """
    Solve one target from one seed.

    Converges on physical tolerances: position within pos_tol meters AND
    orientation within rot_tol radians. rot_weight only sets the relative
    priority of the two inside the least-squares step.

    seed_track_m / returned track_m are logical usable-travel fractions 0..1.
    URDF track meters = TrackM * (travel_mm / rail_mm).
    """
    damping = 0.05
    track_scale = logical_track_m_to_urdf_m(1.0, travel_mm, rail_mm)  # meters per TrackM

    names = active_joint_names(use_track)
    lower, upper = get_bounds(joint_info, names)

    if use_track:
        q = [seed_track_m] + list(seed_joints_rad[:6])
    else:
        q = list(seed_joints_rad[:6])

    q = bound_q(q, lower, upper)

    best_error = 999999.0
    best_track = seed_track_m
    best_joints = list(seed_joints_rad[:6])
    best_plane = rg.Plane.Unset
    best_iteration = 0

    previous_error = None
    stagnant_count = 0

    for iteration in range(max_iter):
        check_deadline(deadline_time)

        track_m, joints_rad = unpack_q(q, seed_track_m, use_track)

        flange_plane, joint_axes = chain_fk(
            chain,
            chain_joint_values(track_m, joints_rad, travel_mm, rail_mm)
        )

        tcp_plane = tcp_plane_from_flange(flange_plane, tcp_local)

        if tcp_plane is None:
            return (
                False,
                seed_track_m,
                seed_joints_rad,
                999999.0,
                rg.Plane.Unset,
                iteration
            )

        err = plane_error(tcp_plane, target_plane, rot_weight)
        en = error_norm(err)

        if en < best_error:
            best_error = en
            best_track = track_m
            best_joints = list(joints_rad)
            best_plane = tcp_plane
            best_iteration = iteration

        pos_err, rot_err = pose_error_parts(tcp_plane, target_plane)

        if pos_err < pos_tol and rot_err < rot_tol:
            return (
                True,
                track_m,
                joints_rad,
                en,
                tcp_plane,
                iteration
            )

        if previous_error is not None:
            if abs(previous_error - en) < 0.00002:
                stagnant_count += 1
            else:
                stagnant_count = 0

            if stagnant_count >= 8:
                break

        previous_error = en

        J = jacobian_from_axes(
            joint_axes,
            joint_info,
            names,
            tcp_plane.Origin
        )

        # plane_error scales its rotation rows by rot_weight, so the angular
        # Jacobian rows must carry the same weight. Without this, DLS fits a
        # weighted error against an unweighted Jacobian and only applies
        # rot_weight of the orientation correction per iteration.
        for r in (3, 4, 5):
            for c in range(len(J[r])):
                J[r][c] *= rot_weight

        # Geometric Jacobian is d(pose)/d(URDF meters). q[0] is logical TrackM,
        # so scale the track column by meters-per-TrackM.
        if use_track and names and names[0] == "track_joint":
            for r in range(len(J)):
                J[r][0] *= track_scale

        J = damp_track_column_if_at_limit(J, q, names)

        dq = damped_least_squares(J, err, damping)

        dq = limit_aware_step(dq, q, names)

        for i in range(len(dq)):
            joint_name = names[i]

            if joint_name == "track_joint":
                dq[i] = clamp(dq[i], -0.030, 0.030)
            else:
                dq[i] = clamp(dq[i], -0.080, 0.080)

        for i in range(len(q)):
            q[i] += dq[i]

        q = bound_q(q, lower, upper)

    if isinstance(best_plane, rg.Plane) and best_plane.IsValid:
        pos_err, rot_err = pose_error_parts(best_plane, target_plane)
        converged = pos_err < pos_tol and rot_err < rot_tol
    else:
        converged = False

    return (
        converged,
        best_track,
        best_joints,
        best_error,
        best_plane,
        best_iteration
    )


def solve_target(
    chain,
    joint_info,
    target_plane,
    seed_track_m,
    seed_joints_rad,
    tcp_local,
    use_track,
    deadline_time,
    pos_tol=0.0005,
    rot_tol=0.0087,
    rot_weight=0.16,
    max_seeds=48,
    travel_mm=870.0,
    rail_mm=1000.0
):
    """
    Track-aware multi-seed IK.

    First tries the sequential seed.
    If that fails, retries the same TCP target with alternate J1/J2/J3
    postures and inward track positions, up to max_seeds of them.
    """
    ok, track_m, joints_rad, err, tcp_plane, iterations = solve_target_single_seed(
        chain=chain,
        joint_info=joint_info,
        target_plane=target_plane,
        seed_track_m=seed_track_m,
        seed_joints_rad=seed_joints_rad,
        tcp_local=tcp_local,
        use_track=use_track,
        max_iter=45,
        pos_tol=pos_tol,
        rot_tol=rot_tol,
        rot_weight=rot_weight,
        deadline_time=deadline_time,
        travel_mm=travel_mm,
        rail_mm=rail_mm
    )

    if ok:
        return ok, track_m, joints_rad, err, tcp_plane, iterations

    seeds = make_reposture_seeds(
        seed_track_m,
        seed_joints_rad,
        use_track
    )

    best_ok = ok
    best_track = track_m
    best_joints = joints_rad
    best_error = err
    best_plane = tcp_plane
    best_iterations = iterations

    for seed_index, (alt_track, alt_joints) in enumerate(seeds):
        if seed_index >= max_seeds:
            break

        check_deadline(deadline_time)

        ok2, track2, joints2, err2, plane2, it2 = solve_target_single_seed(
            chain=chain,
            joint_info=joint_info,
            target_plane=target_plane,
            seed_track_m=alt_track,
            seed_joints_rad=alt_joints,
            tcp_local=tcp_local,
            use_track=use_track,
            max_iter=35,
            pos_tol=pos_tol,
            rot_tol=rot_tol,
            rot_weight=rot_weight,
            deadline_time=deadline_time,
            travel_mm=travel_mm,
            rail_mm=rail_mm
        )

        if err2 < best_error:
            best_ok = ok2
            best_track = track2
            best_joints = joints2
            best_error = err2
            best_plane = plane2
            best_iterations = it2

        if ok2:
            return ok2, track2, joints2, err2, plane2, it2

    return (
        best_ok,
        best_track,
        best_joints,
        best_error,
        best_plane,
        best_iterations
    )


# ------------------------------------------------------------
# Calibration frame conversion
# ------------------------------------------------------------

def normalize_signs(vals):
    out = []
    for i in range(6):
        v = 1.0
        if vals and i < len(vals):
            v = vals[i]
        if v < 0:
            out.append(-1.0)
        else:
            out.append(1.0)
    return out


def get_joint_frame_settings():
    """
    Returns:
      use_frame: bool
      home_urdf_deg: list[6]
      joint_signs: list[6]
      warnings: list[str]
      source: str

    home_urdf_deg is the true URDF pose of firmware A0 B0 C0 D0 E0 F0.
    This is NOT necessarily the raw limit switch pose.
    """
    warnings = []

    # Default OFF: rj_commands stay URDF/sim space. Comp5 encodes for firmware.
    # HomeJointsDeg is still resolved and sticky-stored for the send encoder.
    use_frame = to_bool(
        get_optional_input("UseCalibrationFrame", DEFAULT_USE_CALIBRATION_FRAME),
        DEFAULT_USE_CALIBRATION_FRAME
    )

    home_input = parse_float_list_input(
        get_optional_input("HomeJointsDeg", None),
        count=6,
        default=None
    )

    source = "HomeJointsDeg input"

    if home_input is None and sc is not None:
        sticky_home = sc.sticky.get("AR4_HOME_URDF_DEG", None)
        home_input = parse_float_list_input(sticky_home, count=6, default=None)
        if home_input is not None:
            source = "sc.sticky['AR4_HOME_URDF_DEG']"

    if home_input is None:
        home_input = list(DEFAULT_HOME_URDF_DEG)
        source = "default measured home [-16,0,0,0,22,-10]"

    signs_input = parse_float_list_input(
        get_optional_input("JointSigns", None),
        count=6,
        default=None
    )

    if signs_input is None and sc is not None:
        signs_input = parse_float_list_input(
            sc.sticky.get("AR4_JOINT_SIGNS", None),
            count=6,
            default=None
        )

    if signs_input is None:
        signs_input = list(DEFAULT_JOINT_SIGNS)

    joint_signs = normalize_signs(signs_input)

    if sc is not None:
        sc.sticky["AR4_USE_CALIBRATION_FRAME"] = use_frame
        sc.sticky["AR4_HOME_URDF_DEG"] = list(home_input[:6])
        sc.sticky["AR4_JOINT_SIGNS"] = list(joint_signs)

    return use_frame, home_input[:6], joint_signs, warnings, source


def firmware_deg_to_urdf_deg(firmware_deg, home_urdf_deg, joint_signs, use_frame):
    """
    Converts firmware RP/readback angles into true URDF/COMPAS angles.

    If firmware zero is calibrated HOME:
        URDF = HOME_URDF + firmware / sign
    """
    vals = list(firmware_deg[:6])
    while len(vals) < 6:
        vals.append(0.0)

    if not use_frame:
        return vals[:6]

    out = []
    for i in range(6):
        s = joint_signs[i]
        if abs(s) < 1e-12:
            s = 1.0
        out.append(home_urdf_deg[i] + vals[i] / s)
    return out


def urdf_deg_to_firmware_deg(urdf_deg, home_urdf_deg, joint_signs, use_frame):
    """
    Converts true URDF/COMPAS angles into firmware RJ command angles.

    If firmware zero is calibrated HOME:
        firmware = (URDF - HOME_URDF) * sign
    """
    vals = list(urdf_deg[:6])
    while len(vals) < 6:
        vals.append(0.0)

    if not use_frame:
        return vals[:6]

    out = []
    for i in range(6):
        out.append((vals[i] - home_urdf_deg[i]) * joint_signs[i])
    return out


# ------------------------------------------------------------
# Track conversion and command formatting
# ------------------------------------------------------------

def track_m_to_j7_mm(track_m, track_length_mm, track_firmware_inverted):
    """
    URDF TrackM (0 motor .. 1 far) -> firmware J7 command units.

    Normal Annin (CalDir=0, home at limit/motor):
        TrackFirmwareInverted=False
        J7 = TrackM * TrackLengthMM
        TrackM=0 -> J7=0 (motor), TrackM=1 -> J7=max (far)

    Legacy / wrong-cal polarity (high J7 at motor, 0 at far):
        TrackFirmwareInverted=True
        J7 = (1 - TrackM) * TrackLengthMM
    """
    t = clamp(track_m, 0.0, 1.0)
    if track_firmware_inverted:
        return (1.0 - t) * track_length_mm
    return t * track_length_mm


def j7_mm_to_track_m(j7_mm, track_length_mm, track_firmware_inverted):
    if abs(track_length_mm) < 1e-9:
        track_length_mm = 870.0

    v = clamp(float(j7_mm) / float(track_length_mm), 0.0, 1.0)

    if track_firmware_inverted:
        return 1.0 - v
    return v


def get_track_settings():
    track_length_mm = to_float(get_optional_input("TrackLengthMM", 870.0), 870.0)
    if track_length_mm <= 0.0:
        track_length_mm = 870.0

    # Prefer sticky override from Startup / last calibration if GH input absent.
    if sc is not None and get_optional_input("TrackLengthMM", None) is None:
        sticky_len = sc.sticky.get("AR4_TRACK_LENGTH_MM", None)
        if sticky_len is not None:
            track_length_mm = to_float(sticky_len, track_length_mm)

    track_rail_length_mm = to_float(get_optional_input("TrackRailLengthMM", 1000.0), 1000.0)
    if track_rail_length_mm <= 0.0:
        track_rail_length_mm = 1000.0

    if sc is not None and get_optional_input("TrackRailLengthMM", None) is None:
        sticky_rail = sc.sticky.get("AR4_TRACK_RAIL_LENGTH_MM", None)
        if sticky_rail is not None:
            track_rail_length_mm = to_float(sticky_rail, track_rail_length_mm)

    # GH pin wins; otherwise sticky; otherwise False.
    inv_pin = get_optional_input("TrackFirmwareInverted", None)

    if inv_pin is not None:
        track_firmware_inverted = to_bool(inv_pin, False)
    else:
        track_firmware_inverted = False
        if sc is not None:
            sticky_inv = sc.sticky.get("AR4_TRACK_FIRMWARE_INVERTED", None)
            if sticky_inv is not None:
                track_firmware_inverted = to_bool(sticky_inv, False)

    if sc is not None:
        sc.sticky["AR4_TRACK_LENGTH_MM"] = track_length_mm
        sc.sticky["AR4_TRACK_RAIL_LENGTH_MM"] = track_rail_length_mm
        sc.sticky["AR4_TRACK_FIRMWARE_INVERTED"] = track_firmware_inverted

    return track_length_mm, track_rail_length_mm, track_firmware_inverted


def build_rj_command(
    track_m,
    joints_rad,
    speed,
    accel,
    decel,
    ramp,
    wrist,
    home_urdf_deg,
    joint_signs,
    use_calibration_frame,
    track_length_mm,
    track_firmware_inverted,
    speed_type="m"
):
    """
    Builds an RJ command string from URDF TrackM + J1-J6.

    By default (UseCalibrationFrame=False) A..F are URDF degrees so Comp4 sim
    matches the program. Comp5 applies HomeJointsDeg when sending to firmware.
    """
    track_m = clamp(track_m, 0.0, 1.0)

    j7_mm = track_m_to_j7_mm(
        track_m,
        track_length_mm,
        track_firmware_inverted
    )

    joints_urdf_deg = [math.degrees(v) for v in joints_rad[:6]]

    while len(joints_urdf_deg) < 6:
        joints_urdf_deg.append(0.0)

    joints_firmware_deg = urdf_deg_to_firmware_deg(
        joints_urdf_deg,
        home_urdf_deg,
        joint_signs,
        use_calibration_frame
    )

    j1, j2, j3, j4, j5, j6 = joints_firmware_deg[:6]

    if not wrist:
        wrist = "F"

    speed_type = str(speed_type).strip().lower()
    if speed_type not in ["m", "p", "s"]:
        speed_type = "m"

    loop_mode = "000000"

    cmd = (
        "RJ"
        "A{j1:.3f}"
        "B{j2:.3f}"
        "C{j3:.3f}"
        "D{j4:.3f}"
        "E{j5:.3f}"
        "F{j6:.3f}"
        "J7{j7:.3f}"
        "J8{j8:.3f}"
        "J9{j9:.3f}"
        "S{speed_type}{speed:.3f}"
        "Ac{accel:.3f}"
        "Dc{decel:.3f}"
        "Rm{ramp:.3f}"
        "W{wrist}"
        "Lm{loop_mode}"
    ).format(
        j1=j1,
        j2=j2,
        j3=j3,
        j4=j4,
        j5=j5,
        j6=j6,
        j7=j7_mm,
        j8=0.0,
        j9=0.0,
        speed_type=speed_type,
        speed=speed,
        accel=accel,
        decel=decel,
        ramp=ramp,
        wrist=wrist,
        loop_mode=loop_mode
    )

    return cmd, j7_mm, joints_urdf_deg, joints_firmware_deg


# ------------------------------------------------------------
# Seed and calibration readback conversion
# ------------------------------------------------------------

def get_seed_from_calibration(
    home_urdf_deg,
    joint_signs,
    use_calibration_frame,
    track_length_mm,
    track_firmware_inverted
):
    """
    Uses the most recent Calibration component RP/LL state as the starting pose.

    This function treats calibration readback as firmware-space unless the
    calibration component explicitly stored urdf_joint_values_rad or
    urdf_joint_values_deg.

    If no calibration state exists:
      - track starts at motor/home side: TrackM=0.0 (non-inverted default)
      - joints start at URDF zero (same as firmware zero with UseCalibrationFrame=False)
    """
    seed_track_m = 0.0 if not track_firmware_inverted else 1.0

    if use_calibration_frame:
        seed_joints_rad = [math.radians(v) for v in home_urdf_deg]
    else:
        seed_joints_rad = [0.0] * 6

    seed_source = "default"

    if sc is None:
        return seed_track_m, seed_joints_rad, seed_source

    state = sc.sticky.get("AR4_LAST_CALIBRATION_STATE", None)

    if isinstance(state, dict):
        # Track: prefer explicit track_m; otherwise convert j7_mm.
        if "track_m" in state:
            seed_track_m = to_float(state.get("track_m"), seed_track_m)
        elif "j7_mm" in state:
            seed_track_m = j7_mm_to_track_m(
                to_float(state.get("j7_mm"), 0.0),
                track_length_mm,
                track_firmware_inverted
            )

        # Joints: prefer explicit URDF-space values if calibration component has them.
        urdf_rad = None
        try:
            vals = list(state.get("urdf_joint_values_rad", []))
            if len(vals) >= 6:
                urdf_rad = [to_float(v, 0.0) for v in vals[:6]]
                seed_source = "calibration urdf_joint_values_rad"
        except Exception:
            pass

        if urdf_rad is None:
            try:
                vals = list(state.get("urdf_joint_values_deg", []))
                if len(vals) >= 6:
                    urdf_deg = [to_float(v, 0.0) for v in vals[:6]]
                    urdf_rad = [math.radians(v) for v in urdf_deg]
                    seed_source = "calibration urdf_joint_values_deg"
            except Exception:
                pass

        # Backward-compatible path: existing calibration script likely stored
        # joint_values_deg/rad directly from firmware RP. Treat degrees as firmware.
        if urdf_rad is None:
            firmware_deg = None

            try:
                vals = list(state.get("firmware_joint_values_deg", []))
                if len(vals) >= 6:
                    firmware_deg = [to_float(v, 0.0) for v in vals[:6]]
            except Exception:
                pass

            if firmware_deg is None:
                try:
                    vals = list(state.get("joint_values_deg", []))
                    if len(vals) >= 6:
                        firmware_deg = [to_float(v, 0.0) for v in vals[:6]]
                except Exception:
                    pass

            if firmware_deg is None:
                try:
                    vals = list(state.get("joint_values_rad", []))
                    if len(vals) >= 6:
                        firmware_deg = [math.degrees(to_float(v, 0.0)) for v in vals[:6]]
                except Exception:
                    pass

            if firmware_deg is not None:
                urdf_deg = firmware_deg_to_urdf_deg(
                    firmware_deg,
                    home_urdf_deg,
                    joint_signs,
                    use_calibration_frame
                )
                urdf_rad = [math.radians(v) for v in urdf_deg]
                seed_source = "calibration firmware readback converted to URDF"

        if urdf_rad is not None:
            seed_joints_rad = urdf_rad[:6]

    seed_track_m = clamp(seed_track_m, 0.0, 1.0)

    while len(seed_joints_rad) < 6:
        seed_joints_rad.append(0.0)

    return seed_track_m, seed_joints_rad[:6], seed_source


# ------------------------------------------------------------
# Sticky program storage
# ------------------------------------------------------------

def load_last_outputs():
    if sc is None:
        return None

    state = sc.sticky.get("AR4_LAST_PROGRAM_OUTPUTS", None)

    if isinstance(state, dict):
        return state

    return None


def save_outputs(data):
    if sc is None:
        return

    sc.sticky["AR4_LAST_PROGRAM_OUTPUTS"] = data

    sc.sticky["AR4_LAST_PROGRAM_STATE"] = data.get("program_state", {})
    sc.sticky["AR4_LAST_PROGRAM_COMMANDS"] = data.get("rj_commands", [])
    sc.sticky["AR4_LAST_PROGRAM_TRACK_M"] = data.get("program_track_m", [])
    sc.sticky["AR4_LAST_PROGRAM_J7_MM"] = data.get("program_j7_mm", [])
    sc.sticky["AR4_LAST_PROGRAM_JOINTS_RAD"] = data.get("program_joint_values_rad", [])
    sc.sticky["AR4_LAST_PROGRAM_JOINTS_DEG"] = data.get("program_joint_values_deg", [])
    sc.sticky["AR4_LAST_PROGRAM_FIRMWARE_JOINTS_DEG"] = data.get("program_firmware_joint_values_deg", [])
    sc.sticky["AR4_LAST_PROGRAM_TCP_PLANES"] = data.get("tcp_planes_solved", [])
    sc.sticky["AR4_LAST_PROGRAM_TARGET_PLANES"] = data.get("target_planes_robot", [])
    sc.sticky["AR4_LAST_PROGRAM_COMMAND_METADATA"] = data.get("command_metadata", [])


# ------------------------------------------------------------
# Outputs / defaults
# ------------------------------------------------------------

solved = []
program_joint_values_rad = []
program_joint_values_deg = []
program_track_m = []
program_j7_mm = []
rj_commands = []
ik_errors = []
tcp_pos_error_mm = []
tcp_rot_error_deg = []
target_planes_robot = []
tcp_planes_solved = []
program_state = {}
status = ""


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------

try:
    if sc is None:
        raise Exception("scriptcontext is not available.")

    cid = to_text(cache_id, "").strip()

    if not cid or cid == "None":
        raise Exception("No cache_id provided. Connect cache_id from Startup.")

    model = sc.sticky.get(cid + "::model", None)

    if model is None:
        raise Exception("No cached robot model found. Run Startup first.")

    raw_targets = flatten_targets(Targets)

    base_plane, base_plane_source = resolve_base_plane(cid)
    world_to_local, local_to_world = base_plane_transforms(base_plane)

    solve_now = to_bool(Solve, False)
    use_track = to_bool(UseTrack, True)

    # Optional settings. These can be added as GH inputs without breaking older files.
    max_solve_seconds = to_float(get_optional_input("MaxSolveSeconds", 300.0), 300.0)
    if max_solve_seconds <= 0.0:
        max_solve_seconds = 300.0

    # Default False: one RJ per target moves arm + J7 together for smooth TCP.
    # True restores legacy track-first preposition (not synchronized).
    split_track_motion = to_bool(get_optional_input("SplitTrackMotion", False), False)
    track_speed_percent = to_float(get_optional_input("TrackSpeedPercent", 100.0), 100.0)
    track_speed_percent = clamp(track_speed_percent, 1.0, 100.0)
    track_move_epsilon_mm = to_float(get_optional_input("TrackMoveEpsilonMM", 0.5), 0.5)
    if track_move_epsilon_mm < 0.0:
        track_move_epsilon_mm = 0.5

    track_length_mm, track_rail_length_mm, track_firmware_inverted = get_track_settings()

    (
        use_calibration_frame,
        home_urdf_deg,
        joint_signs,
        frame_warnings,
        home_source
    ) = get_joint_frame_settings()

    # Edge-trigger Solve.
    edge_key = "AR4_PROGRAM_SOLVE_LAST_STATE"
    previous_solve = sc.sticky.get(edge_key, False)
    sc.sticky[edge_key] = solve_now

    last = load_last_outputs()

    if not raw_targets:
        status = "No target TCP planes provided."

    elif not solve_now:
        if last:
            solved = last.get("solved", [])
            program_joint_values_rad = last.get("program_joint_values_rad", [])
            program_joint_values_deg = last.get("program_joint_values_deg", [])
            program_track_m = last.get("program_track_m", [])
            program_j7_mm = last.get("program_j7_mm", [])
            rj_commands = last.get("rj_commands", [])
            ik_errors = last.get("ik_errors", [])
            tcp_pos_error_mm = last.get("tcp_pos_error_mm", [])
            tcp_rot_error_deg = last.get("tcp_rot_error_deg", [])
            program_state = last.get("program_state", {})

            # Sticky stores meter planes; GH outputs should be Rhino mm.
            disp = 1000.0
            if isinstance(program_state, dict):
                ts = to_float(program_state.get("target_scale"), 0.001)
                if abs(ts) > 1e-12:
                    disp = 1.0 / ts
            target_planes_robot = planes_to_world_display(
                last.get("target_planes_robot", []), disp, local_to_world
            )
            tcp_planes_solved = planes_to_world_display(
                last.get("tcp_planes_solved", []), disp, local_to_world
            )

            status = "Solve=False. Holding last program with {} command(s).".format(
                len(rj_commands)
            )

        else:
            status = "Ready. {} TCP target plane(s) loaded. Toggle Solve=True to generate commands.".format(
                len(raw_targets)
            )

    elif previous_solve:
        if last:
            solved = last.get("solved", [])
            program_joint_values_rad = last.get("program_joint_values_rad", [])
            program_joint_values_deg = last.get("program_joint_values_deg", [])
            program_track_m = last.get("program_track_m", [])
            program_j7_mm = last.get("program_j7_mm", [])
            rj_commands = last.get("rj_commands", [])
            ik_errors = last.get("ik_errors", [])
            tcp_pos_error_mm = last.get("tcp_pos_error_mm", [])
            tcp_rot_error_deg = last.get("tcp_rot_error_deg", [])
            program_state = last.get("program_state", {})

            disp = 1000.0
            if isinstance(program_state, dict):
                ts = to_float(program_state.get("target_scale"), 0.001)
                if abs(ts) > 1e-12:
                    disp = 1.0 / ts
            target_planes_robot = planes_to_world_display(
                last.get("target_planes_robot", []), disp, local_to_world
            )
            tcp_planes_solved = planes_to_world_display(
                last.get("tcp_planes_solved", []), disp, local_to_world
            )

            status = "Solve is still True. Not recomputing. Toggle Solve False, then True to resolve."

        else:
            status = "Solve is True, but no previous program exists. Toggle Solve False, then True."

    else:
        deadline_time = time.time() + max_solve_seconds

        urdf_path = get_urdf_path_from_cache(cid)
        joint_info = get_joint_info(cid, urdf_path)

        if not joint_info:
            raise Exception(
                "Could not parse/cache URDF motion joints from: {}".format(
                    urdf_path
                )
            )

        tool_link = choose_tool_link(model)

        if not tool_link:
            raise Exception("Could not identify tool link.")

        # Parsed once and cached. The IK walks this instead of calling COMPAS
        # forward_kinematics per link, which dominated solve time.
        fk_chain = get_fk_chain(cid, urdf_path, tool_link)

        if not fk_chain:
            raise Exception(
                "Could not build an FK chain from '{}' to tool link '{}'.".format(
                    urdf_path,
                    tool_link
                )
            )

        max_seeds = int(to_float(get_optional_input("MaxSeeds", 48), 48))

        if max_seeds < 1:
            max_seeds = 1

        solve_started = time.time()

        speed = to_float(Speed, 5.0)
        accel = to_float(Accel, 5.0)
        decel = to_float(Decel, 5.0)
        ramp = to_float(Ramp, 10.0)
        wrist = to_text(Wrist, "F")

        target_units_mm = to_bool(get_optional_input("TargetUnitsMM", True), True)
        target_scale = guess_target_scale(raw_targets, force_mm=target_units_mm)

        # World Rhino targets -> robot-local frame -> URDF meters for IK.
        targets = [
            scale_plane_origin(
                transform_plane(pl, world_to_local),
                target_scale
            )
            for pl in raw_targets
        ]

        # Flange-local TCP frame, in URDF meters like the rest of the IK.
        tcp_local, tcp_info = resolve_tcp_frame(target_scale)

        # Physical convergence limits, authored in Rhino units / degrees.
        pos_tol = to_float(get_optional_input("PosTolMM", 0.5), 0.5) * target_scale
        rot_tol = math.radians(
            to_float(get_optional_input("RotTolDeg", 0.5), 0.5)
        )
        rot_weight = to_float(get_optional_input("RotationWeight", 0.16), 0.16)

        if pos_tol <= 0.0:
            pos_tol = 0.0005
        if rot_tol <= 0.0:
            rot_tol = math.radians(0.5)
        if rot_weight <= 0.0:
            rot_weight = 0.16

        seed_track_m, seed_joints_rad, seed_source = get_seed_from_calibration(
            home_urdf_deg,
            joint_signs,
            use_calibration_frame,
            track_length_mm,
            track_firmware_inverted
        )

        current_track = seed_track_m
        current_joints = list(seed_joints_rad)
        current_j7_mm = track_m_to_j7_mm(
            current_track,
            track_length_mm,
            track_firmware_inverted
        )

        failures = 0
        command_metadata = []
        program_firmware_joint_values_deg = []

        for target_index, target in enumerate(targets):
            check_deadline(deadline_time)

            target_planes_robot.append(target)

            ok, track_m, joints_rad, err, tcp_plane, iterations = solve_target(
                chain=fk_chain,
                joint_info=joint_info,
                target_plane=target,
                seed_track_m=current_track,
                seed_joints_rad=current_joints,
                tcp_local=tcp_local,
                use_track=use_track,
                deadline_time=deadline_time,
                pos_tol=pos_tol,
                rot_tol=rot_tol,
                rot_weight=rot_weight,
                max_seeds=max_seeds,
                travel_mm=track_length_mm,
                rail_mm=track_rail_length_mm
            )

            # Build final arm/track command from solved URDF pose.
            cmd, j7, joints_urdf_deg, joints_fw_deg = build_rj_command(
                track_m=track_m,
                joints_rad=joints_rad,
                speed=speed,
                accel=accel,
                decel=decel,
                ramp=ramp,
                wrist=wrist,
                home_urdf_deg=home_urdf_deg,
                joint_signs=joint_signs,
                use_calibration_frame=use_calibration_frame,
                track_length_mm=track_length_mm,
                track_firmware_inverted=track_firmware_inverted,
                speed_type="m"
            )

            # Optional legacy behavior: move J7 alone first, then the arm.
            # Default is synchronized: one RJ moves joints + J7 together so the
            # TCP can progress continuously through the target sequence.
            if (
                split_track_motion
                and use_track
                and abs(j7 - current_j7_mm) > track_move_epsilon_mm
            ):
                track_cmd, track_j7, track_urdf_deg, track_fw_deg = build_rj_command(
                    track_m=track_m,
                    joints_rad=current_joints,
                    speed=track_speed_percent,
                    accel=accel,
                    decel=decel,
                    ramp=ramp,
                    wrist=wrist,
                    home_urdf_deg=home_urdf_deg,
                    joint_signs=joint_signs,
                    use_calibration_frame=use_calibration_frame,
                    track_length_mm=track_length_mm,
                    track_firmware_inverted=track_firmware_inverted,
                    speed_type="p"
                )

                rj_commands.append(track_cmd)
                command_metadata.append({
                    "target_index": target_index,
                    "type": "track_preposition",
                    "command": track_cmd,
                    "speed_type": "p",
                    "speed": track_speed_percent,
                    "track_m": track_m,
                    "j7_mm": track_j7,
                    "urdf_track_m": logical_track_m_to_urdf_m(
                        track_m, track_length_mm, track_rail_length_mm
                    ),
                    "urdf_joint_values_deg": track_urdf_deg,
                    "urdf_joint_values_rad": list(current_joints),
                    "firmware_joint_values_deg": track_fw_deg
                })

            rj_commands.append(cmd)
            command_metadata.append({
                "target_index": target_index,
                "type": "track_preposition_followup" if split_track_motion else "synchronized_target",
                "command": cmd,
                "speed_type": "m",
                "speed": speed,
                "track_m": track_m,
                "j7_mm": j7,
                "urdf_track_m": logical_track_m_to_urdf_m(
                    track_m, track_length_mm, track_rail_length_mm
                ),
                "urdf_joint_values_deg": joints_urdf_deg,
                "urdf_joint_values_rad": list(joints_rad),
                "firmware_joint_values_deg": joints_fw_deg,
                "synchronized_track_and_arm": not split_track_motion
            })

            solved.append(ok)
            program_track_m.append(track_m)
            program_j7_mm.append(j7)
            program_joint_values_rad.append(joints_rad)
            program_joint_values_deg.append(joints_urdf_deg)
            program_firmware_joint_values_deg.append(joints_fw_deg)
            ik_errors.append(err)
            tcp_planes_solved.append(tcp_plane)

            # Report the two halves separately so a target that lands on
            # position but misses orientation is visible instead of hidden
            # inside one blended norm.
            if isinstance(tcp_plane, rg.Plane) and tcp_plane.IsValid:
                pe, re = pose_error_parts(tcp_plane, target)
                tcp_pos_error_mm.append(
                    pe / target_scale if abs(target_scale) > 1e-12 else pe
                )
                tcp_rot_error_deg.append(math.degrees(re))
            else:
                tcp_pos_error_mm.append(float("nan"))
                tcp_rot_error_deg.append(float("nan"))

            if not ok:
                failures += 1

            # Sequential solving:
            # each target uses the previous solved URDF pose as its next seed.
            current_track = track_m
            current_joints = list(joints_rad)
            current_j7_mm = j7

        program_state = {
            "target_count": len(targets),
            "command_count": len(rj_commands),
            "failures": failures,
            "tool_link": tool_link,
            "tcp_frame": tcp_info,
            "tcp_source": tcp_info.get("source", ""),
            "pos_tol_mm": pos_tol / target_scale if abs(target_scale) > 1e-12 else pos_tol,
            "rot_tol_deg": math.degrees(rot_tol),
            "rotation_weight": rot_weight,
            "target_units_mm": target_units_mm,
            "use_track": use_track,
            "speed": speed,
            "accel": accel,
            "decel": decel,
            "ramp": ramp,
            "wrist": wrist,
            "target_scale": target_scale,
            "display_scale": (1.0 / target_scale) if abs(target_scale) > 1e-12 else 1.0,
            "base_plane": base_plane,
            "base_plane_source": base_plane_source,
            "urdf_path": urdf_path,
            "motion_joint_names": MOTION_JOINT_NAMES,
            "seed_track_m": seed_track_m,
            "seed_joint_values_rad": seed_joints_rad,
            "seed_source": seed_source,
            "track_limit_reposture_enabled": True,
            "split_track_motion": split_track_motion,
            "track_speed_percent": track_speed_percent,
            "track_move_epsilon_mm": track_move_epsilon_mm,
            "track_firmware_inverted": track_firmware_inverted,
            "track_length_mm": track_length_mm,
            "track_rail_length_mm": track_rail_length_mm,
            "track_urdf_m_at_full_travel": logical_track_m_to_urdf_m(
                1.0, track_length_mm, track_rail_length_mm
            ),
            "use_calibration_frame": use_calibration_frame,
            "home_urdf_deg": home_urdf_deg,
            "home_source": home_source,
            "joint_signs": joint_signs,
            "frame_warnings": frame_warnings,
            "max_solve_seconds": max_solve_seconds,
            "program_firmware_joint_values_deg": program_firmware_joint_values_deg,
            "command_metadata": command_metadata
        }

        output_data = {
            "solved": solved,
            "program_joint_values_rad": program_joint_values_rad,
            "program_joint_values_deg": program_joint_values_deg,
            "program_track_m": program_track_m,
            "program_j7_mm": program_j7_mm,
            "rj_commands": rj_commands,
            "ik_errors": ik_errors,
            "tcp_pos_error_mm": tcp_pos_error_mm,
            "tcp_rot_error_deg": tcp_rot_error_deg,
            "target_planes_robot": target_planes_robot,
            "tcp_planes_solved": tcp_planes_solved,
            "program_state": program_state,
            "program_firmware_joint_values_deg": program_firmware_joint_values_deg,
            "command_metadata": command_metadata
        }

        save_outputs(output_data)

        # Comp4 draws the tool at the same TCP frame the IK solved against.
        if sc is not None:
            sc.sticky["AR4_LAST_PROGRAM_TCP_FRAME"] = dict(tcp_info)

        worst_pos = max([v for v in tcp_pos_error_mm if v == v], default=0.0)
        worst_rot = max([v for v in tcp_rot_error_deg if v == v], default=0.0)

        solve_seconds = time.time() - solve_started
        per_target_ms = (
            solve_seconds * 1000.0 / len(targets) if targets else 0.0
        )

        # Sticky keeps model-meter planes for Comp4 FK.
        # GH outputs are converted back to Rhino millimeters for overlay.
        display_scale = 1.0 / target_scale if abs(target_scale) > 1e-12 else 1.0
        target_planes_robot = planes_to_world_display(
            target_planes_robot, display_scale, local_to_world
        )
        tcp_planes_solved = planes_to_world_display(
            tcp_planes_solved, display_scale, local_to_world
        )

        if failures == 0:
            status = (
                "Solved {} TCP target(s) in {:.2f} s ({:.1f} ms/target). "
                "Generated {} RJ command(s). "
                "TCP: {}. Worst miss {:.3f} mm / {:.3f} deg "
                "(limits {:.3f} mm / {:.3f} deg). "
                "Motion: {}. J7=TrackM*{:.0f}mm inverted={}. "
                "Calibration-frame conversion: {}. "
                "Home source: {}. Seed source: {}. Base plane: {}. "
                "Toggle Solve False before resolving."
            ).format(
                len(targets),
                solve_seconds,
                per_target_ms,
                len(rj_commands),
                tcp_info.get("source", "?"),
                worst_pos,
                worst_rot,
                pos_tol / target_scale if abs(target_scale) > 1e-12 else pos_tol,
                math.degrees(rot_tol),
                "split track-then-arm" if split_track_motion else "synchronized arm+J7",
                track_length_mm,
                track_firmware_inverted,
                use_calibration_frame,
                home_source,
                seed_source,
                base_plane_source
            )
        else:
            status = (
                "Generated {} RJ command(s) in {:.2f} s ({:.1f} ms/target), "
                "but {} target(s) exceeded tolerance. "
                "TCP: {}. Worst miss {:.3f} mm / {:.3f} deg "
                "(limits {:.3f} mm / {:.3f} deg). "
                "Motion: {}. J7=TrackM*{:.0f}mm inverted={}. "
                "Calibration-frame conversion: {}. "
                "Home source: {}. Check tcp_pos_error_mm and tcp_rot_error_deg."
            ).format(
                len(rj_commands),
                solve_seconds,
                per_target_ms,
                failures,
                tcp_info.get("source", "?"),
                worst_pos,
                worst_rot,
                pos_tol / target_scale if abs(target_scale) > 1e-12 else pos_tol,
                math.degrees(rot_tol),
                "split track-then-arm" if split_track_motion else "synchronized arm+J7",
                track_length_mm,
                track_firmware_inverted,
                use_calibration_frame,
                home_source
            )

        if frame_warnings:
            status += " WARNING: " + " | ".join(frame_warnings)

except Exception as e:
    solved = []
    program_joint_values_rad = []
    program_joint_values_deg = []
    program_track_m = []
    program_j7_mm = []
    rj_commands = []
    ik_errors = []
    tcp_pos_error_mm = []
    tcp_rot_error_deg = []
    target_planes_robot = []
    tcp_planes_solved = []
    program_state = {}
    status = "ERROR: {}".format(e)
