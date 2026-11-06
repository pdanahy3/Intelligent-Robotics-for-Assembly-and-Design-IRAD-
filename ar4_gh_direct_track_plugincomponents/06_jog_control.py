#! python3
# r: compas
# r: compas_fab
# r: compas_robots
# r: pyserial

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
J1..J6            : float sliders, URDF degrees
TrackM            : float slider 0..1, fraction of usable track travel
BasePlane         : plane. Where this robot's URDF world sits. Default WorldXY.

Serial (all optional; Send does nothing without Port):
Port              : COM port string
Baud              : default 9600
Send              : bool edge trigger. False -> True sends the current pose.
Speed             : default 15.0
SpeedType         : "p" percent (default), "m" mm/sec, "s" seconds
Accel/Decel/Ramp  : firmware Ac / Dc / Rm, default 10 / 10 / 10
Wrist             : firmware W token, default "F"
EncodeFirmwareOnSend : bool, default True

Capture:
Capture           : bool edge trigger. False -> True logs the current pose.
CaptureLabel      : text tag for the capture, e.g. "tip1" or "axisX"
ClearCaptures     : bool edge trigger. Wipes the capture log.

Display:
DrawRobot / DrawTrack / DrawSkeleton : bools
ToolMeshes        : meshes standing in for the gripper
HideGripper       : bool, defaults True once ToolMeshes is wired
TCPPlane / TCPPoint / ToolZ : flange-local TCP, same contract as Comp3/Comp4

Track geometry:
TrackLengthMM     : usable travel, default 870
TrackRailLengthMM : full rail visual length, default 1000
TrackFirmwareInverted : bool, default sticky / False
HomeJointsDeg / JointSigns : 6 floats each, default sticky


OUTPUTS
-------
robot_geometry / tool_geometry / track_geometry / skeleton_lines
flange_plane      : current flange frame, world display units
tcp_plane         : current TCP frame, world display units
rj_command        : the command Send would push (already firmware encoded)
captures          : the capture log, also in sticky AR4_JOG_CAPTURES
capture_count
capture_labels
jog_state
status
"""

import math
import time

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
    """Flange-local TCP frame in URDF meters, plus a source label."""
    plane = to_plane(gh_input("TCPPlane", None), None)

    if plane is not None:
        return plane_origin_to_meters(plane), "TCPPlane input"

    point = to_plane(gh_input("TCPPoint", None), None)

    if point is not None:
        return (
            plane_origin_to_meters(
                rg.Plane(point.Origin, rg.Vector3d.XAxis, rg.Vector3d.YAxis)
            ),
            "TCPPoint input"
        )

    tool_z_mm = to_float(gh_input("ToolZ", None), None)

    if tool_z_mm is not None and abs(tool_z_mm) > 1e-12:
        return (
            offset_plane_along_z(rg.Plane.WorldXY, tool_z_mm * 0.001),
            "ToolZ legacy ({:.1f} mm)".format(tool_z_mm)
        )

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
    try:
        link_names = [link.name for link in model.links]
    except Exception:
        link_names = []

    for name in ["tool0", "tool_link", "pneumatic_gripper_tool_link", "ee_link", "link_6"]:
        if name in link_names:
            return name

    return link_names[-1] if link_names else None


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
    """Comp1 loads and caches the tool; the pin is the per-robot override."""
    raw = gh_input("ToolMeshes", None)

    if raw is None:
        if sc is not None:
            cached = sc.sticky.get("AR4_TOOL_MESHES", None)

            if isinstance(cached, list):
                return list(cached)

        return []

    try:
        items = list(raw)
    except TypeError:
        items = [raw]

    meshes = []

    for item in items:
        item = unwrap_gh(item)

        if item is None:
            continue

        if isinstance(item, rg.Mesh):
            meshes.append(item)
            continue

        if isinstance(item, rg.Brep):
            for m in (rg.Mesh.CreateFromBrep(item, rg.MeshingParameters.Default) or []):
                meshes.append(m)

    return meshes


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


def send_single_command(port, baud, command, timeout):
    """Open, send one RJ, wait for the position reply, close."""
    import serial

    ser = serial.Serial(
        port=port,
        baudrate=baud,
        timeout=0.1,
        write_timeout=2.0
    )

    # Teensy resets on port open.
    time.sleep(2.0)

    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except Exception:
        pass

    try:
        ser.write((command.strip() + "\n").encode("utf-8"))
        ser.flush()

        buffer = ""
        start = time.time()

        while time.time() - start < timeout:
            if ser.in_waiting > 0:
                buffer += ser.read(ser.in_waiting).decode("utf-8", errors="ignore")

                for line in [l.strip() for l in buffer.splitlines() if l.strip()]:
                    if line == "ER" or line.startswith("EL"):
                        return buffer, "FIRMWARE_" + line[:2]

                    if line.startswith("A") and "B" in line and "P" in line:
                        return buffer, "RP"

            time.sleep(0.02)

        return buffer, "TIMEOUT" if buffer else "TIMEOUT_EMPTY"

    finally:
        try:
            ser.close()
        except Exception:
            pass


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

    joints_deg = [
        to_float(gh_input("J1", 0.0), 0.0),
        to_float(gh_input("J2", 0.0), 0.0),
        to_float(gh_input("J3", 0.0), 0.0),
        to_float(gh_input("J4", 0.0), 0.0),
        to_float(gh_input("J5", 0.0), 0.0),
        to_float(gh_input("J6", 0.0), 0.0)
    ]
    joints_rad = [math.radians(v) for v in joints_deg]

    track_m = clamp(to_float(gh_input("TrackM", 0.0), 0.0), 0.0, 1.0)
    j7_mm = track_m_to_j7_mm(track_m, travel_mm, track_inverted)

    config = make_config(model, track_m, joints_rad, travel_mm, rail_mm)
    frame_by_link = get_frame_by_link(model, config)

    tool_frame = frame_by_link.get(tool_link, None)

    if tool_frame is None:
        raise Exception("FK failed for tool link '{}'.".format(tool_link))

    flange_local = frame_to_local_plane(tool_frame)
    flange_plane = frame_to_plane(tool_frame, display_scale)

    tcp_local, tcp_source = resolve_tcp_frame()
    tcp_plane = apply_tcp_frame(flange_plane, tcp_local, display_scale)

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

    # Send.
    port = to_text(gh_input("Port", ""), "")
    baud = int(to_float(gh_input("Baud", 9600), 9600))
    send_requested = to_bool(gh_input("Send", False), False)

    if edge_triggered("AR4_JOG_SEND_EDGE", send_requested):
        if not port:
            send_note = "Send ignored: no Port"
        else:
            reply, reply_status = send_single_command(
                port,
                baud,
                rj_command,
                to_float(gh_input("SendTimeout", 30.0), 30.0)
            )
            send_note = "sent, reply {}".format(reply_status)
            sc.sticky["AR4_JOG_LAST_REPLY"] = reply
    elif send_requested:
        send_note = "Send held True; toggle False then True to move again"
    else:
        send_note = "Send off"

    jog_state = {
        "cache_id": cache_id,
        "tool_link": tool_link,
        "joints_urdf_deg": list(joints_deg),
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
        "Comp6 jog | J {} | TrackM {:.4f} (J7 {:.1f} mm) | "
        "TCP {} | base {} | encode {} | captures {} | {} | {}"
    ).format(
        [round(v, 2) for v in joints_deg],
        track_m,
        j7_mm,
        tcp_source,
        base_plane_source,
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
