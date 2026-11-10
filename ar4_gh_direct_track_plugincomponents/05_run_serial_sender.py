#! python3
# r: pyserial
# r: compas
# r: compas_robots

# =============================================================================
# GH_PINS — Rhino does not create pins from pasted code. Copy these names into
# the Python Script component UI (Type Hint / Access in COMPONENT_PINS.txt).
# INPUTS:
#   Commands
#   Run
#   Stop
#   Delay
#   WaitForReply
#   TimeoutPerCommand
#   MaxInFlight
#   EncodeFirmwareOnSend
#   HomeJointsDeg
#   JointSigns
#   DrawRobot
# OUTPUTS:
#   sent_commands
#   reply_lines
#   raw_replies
#   parsed_states
#   last_reply
#   last_rp_state
#   run_index
#   completed
#   failed_index
#   live_joint_values_deg
#   live_track_m
#   live_j7_mm
#   flange_plane
#   tcp_plane
#   robot_geometry
#   tool_geometry
#   status
# =============================================================================

"""
AR4 + External Track — Component 5: Run / Serial Sender

Accepts program/sim RJ commands (URDF joint degrees in A..F) and streams them
through Comp1's open SerialBus (sticky AR4_SERIAL_BUS). This component does
not open or close the COM port — leave Comp1 Connect=True.

Before serial send, optionally encodes A..F to firmware degrees:

    firmware = (urdf - HomeJointsDeg) * JointSigns

Measured on this robot:
  HomeJointsDeg = [-16, 0, 0, 0, 22, -10]
  JointSigns    = [-1, 1, 1, 1, 1, 1]

  program A0 E45 F0     ->  physical A-16 E23 F10
  program joint_1 -36   ->  physical A+20   (both reach toward +X / far end)

J1's sign is negative because URDF joint_1 turns opposite to firmware A. A
single home pose cannot separate sign from offset, so both were verified with
a second pose off home. Flipping sign[0] requires flipping home[0] with it or
true home breaks.

Optional GH inputs:
  EncodeFirmwareOnSend : bool, default True (or sticky AR4_ENCODE_FIRMWARE_ON_SEND)
  HomeJointsDeg        : 6 floats, default sticky / [16,0,0,0,22,-10]
  JointSigns           : 6 floats, default sticky / 1,1,1,1,1,1
  MaxInFlight          : int 1-2, default 2. Commands allowed to sit
                         unacknowledged in the controller's queue.
  WaitForReply         : bool, default False. True forces strict
                         call-and-response (MaxInFlight is then 1).
  TimeoutPerCommand    : float seconds, default 300. Must exceed the longest
                         single move, since the reply only arrives once the
                         move completes.

Flow control
------------
The firmware holds three commands (cmdBuffer1..3) and reads serial one byte
per loop() iteration, only while cmdBuffer3 is empty. A move blocks loop(),
so nothing is read for the whole duration of that move. Sending without
waiting for replies therefore backs data up into the USB buffer until a host
write blocks, which appeared as a stall partway through long programs. This
component never leaves more than MaxInFlight commands unacknowledged, so the
controller's queue absorbs the look-ahead and writes never block.

Non-blocking GH solves
----------------------
Each solve only polls the SerialBus ack queue (no multi-second wait, no
time.sleep). If a move is still running, Comp5 schedules another solve in
~40 ms via GH ScheduleSolution so the canvas, Comp1 live pose, and preview
keep updating. TimeoutPerCommand is enforced with timestamps, not by parking
the solve thread.

Note that each RJ command accelerates from and decelerates to a full stop, so
a long point sequence moves in discrete steps regardless of pacing. Continuous
velocity needs the firmware's spline mode (SL / MS / SS), which acknowledges
on receipt rather than on completion. Mid-move joint streaming still depends
on the firmware emitting RP while moving; between moves the preview updates
from each RP as soon as it arrives.
"""

import time
import math

import Rhino.Geometry as rg

try:
    import scriptcontext as sc
except Exception:
    sc = None

# ghenv is injected by the Grasshopper Python Script host when available.
if "ghenv" not in globals():
    ghenv = None


# ------------------------------------------------------------
# Helpers: Grasshopper values
# ------------------------------------------------------------

def unwrap_gh(value):
    if value is None:
        return None

    if hasattr(value, "Value"):
        return value.Value

    return value


def to_bool(value, default=False):
    value = unwrap_gh(value)

    if value is None:
        return default

    if isinstance(value, bool):
        return value

    return str(value).strip().lower() in ["true", "1", "yes", "y"]


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


def to_command_list(values):
    """
    Accepts:
      - a GH list of command strings
      - one multiline panel
      - a single command string

    Returns a clean list of RJ commands.
    """
    if values is None:
        return []

    values = unwrap_gh(values)

    try:
        raw = list(values)
    except TypeError:
        raw = [values]

    result = []

    for item in raw:
        item = unwrap_gh(item)

        if item is None:
            continue

        s = str(item).strip()

        if not s:
            continue

        # Allow one text panel with multiple commands.
        if "\n" in s:
            for line in s.splitlines():
                line = line.strip()

                if line:
                    result.append(line)
        else:
            result.append(s)

    cleaned = []

    for cmd in result:
        cmd = cmd.strip()

        if not cmd:
            continue

        if cmd.upper().startswith("RJ"):
            cleaned.append(cmd)

    return cleaned


DEFAULT_HOME_URDF_DEG = [-16.0, 0.0, 0.0, 0.0, 22.0, -10.0]
DEFAULT_JOINT_SIGNS = [-1.0, 1.0, 1.0, 1.0, 1.0, 1.0]


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


def get_optional_input(name, default=None):
    return unwrap_gh(globals().get(name, default))


def get_encode_settings():
    """Home offset used only when encoding program RJ -> firmware RJ for send."""
    encode = get_optional_input("EncodeFirmwareOnSend", None)
    if encode is None and sc is not None:
        encode = sc.sticky.get("AR4_ENCODE_FIRMWARE_ON_SEND", True)
    encode = to_bool(encode, True)

    home = parse_float_list_input(
        get_optional_input("HomeJointsDeg", None), 6, None
    )
    if home is None and sc is not None:
        home = parse_float_list_input(
            sc.sticky.get("AR4_HOME_URDF_DEG", None), 6, None
        )
    if home is None:
        home = list(DEFAULT_HOME_URDF_DEG)

    signs = parse_float_list_input(
        get_optional_input("JointSigns", None), 6, None
    )
    if signs is None and sc is not None:
        signs = parse_float_list_input(
            sc.sticky.get("AR4_JOINT_SIGNS", None), 6, None
        )
    if signs is None:
        signs = list(DEFAULT_JOINT_SIGNS)

    out_signs = []
    for s in signs[:6]:
        s = to_float(s, 1.0)
        if abs(s) < 1e-12:
            s = 1.0
        out_signs.append(1.0 if s >= 0.0 else -1.0)

    if sc is not None:
        sc.sticky["AR4_ENCODE_FIRMWARE_ON_SEND"] = encode
        sc.sticky["AR4_HOME_URDF_DEG"] = list(home[:6])
        sc.sticky["AR4_JOINT_SIGNS"] = list(out_signs)

    return encode, home[:6], out_signs


def urdf_deg_to_firmware_deg(urdf_deg, home_urdf_deg, joint_signs):
    out = []
    for i in range(6):
        out.append((to_float(urdf_deg[i], 0.0) - home_urdf_deg[i]) * joint_signs[i])
    return out


def replace_rj_joint_token(command, token, next_token, value):
    """
    Replace the numeric value after token (e.g. 'A') up to next_token (e.g. 'B').
    """
    s = str(command)
    start = s.find(token)
    if start < 0:
        return s

    val_start = start + len(token)
    end = s.find(next_token, val_start)
    if end < 0:
        end = len(s)

    return s[:val_start] + "{:.3f}".format(value) + s[end:]


def encode_rj_command_for_firmware(command, home_urdf_deg, joint_signs):
    """
    Rewrite A..F on a program/sim RJ string into firmware degrees.
    J7/J8/J9 and speed tokens are left unchanged.
    """
    s = str(command).strip()
    if not s.upper().startswith("RJ"):
        return s, None

    try:
        j1 = to_float(between(s, "A", "B"))
        j2 = to_float(between(s, "B", "C"))
        j3 = to_float(between(s, "C", "D"))
        j4 = to_float(between(s, "D", "E"))
        j5 = to_float(between(s, "E", "F"))
        j6 = to_float(between(s, "F", "J7"))
    except Exception:
        return s, None

    urdf = [j1, j2, j3, j4, j5, j6]
    fw = urdf_deg_to_firmware_deg(urdf, home_urdf_deg, joint_signs)

    out = s
    pairs = [
        ("A", "B", fw[0]),
        ("B", "C", fw[1]),
        ("C", "D", fw[2]),
        ("D", "E", fw[3]),
        ("E", "F", fw[4]),
        ("F", "J7", fw[5]),
    ]
    for token, nxt, val in pairs:
        out = replace_rj_joint_token(out, token, nxt, val)

    return out, {"urdf_deg": urdf, "firmware_deg": fw}


def encode_command_list_for_firmware(commands, encode, home_urdf_deg, joint_signs):
    if not encode:
        return list(commands), []

    encoded = []
    maps = []
    for cmd in commands:
        out, info = encode_rj_command_for_firmware(cmd, home_urdf_deg, joint_signs)
        encoded.append(out)
        maps.append(info)
    return encoded, maps


# ------------------------------------------------------------
# RJ command validation
# ------------------------------------------------------------

def validate_rj_command(command):
    """
    Checks only the command structure.
    Does not decide whether the motion is physically safe.
    """
    if command is None:
        return False, "Command is None."

    s = str(command).strip()

    if not s.upper().startswith("RJ"):
        return False, "Command does not start with RJ."

    required_tokens = [
        "A",
        "B",
        "C",
        "D",
        "E",
        "F",
        "J7",
        "J8",
        "J9",
        "Ac",
        "Dc",
        "Rm",
        "W",
        "Lm"
    ]

    for token in required_tokens:
        if token not in s:
            return False, "Missing token {}.".format(token)

    # The speed token is S followed by a type letter. The firmware accepts
    # s (seconds), m (mm/sec) and p (percent); requiring a literal "Sm" here
    # used to reject every command as soon as the planner emitted anything else.
    speed_at = s.find("J9")
    speed_at = s.find("S", speed_at if speed_at >= 0 else 0)

    if speed_at < 0 or speed_at + 1 >= len(s):
        return False, "Missing speed token S."

    if s[speed_at + 1] not in ("s", "m", "p"):
        return False, "Speed type S{} is not one of s, m, p.".format(s[speed_at + 1])

    # This catches the earlier malformed external-axis form:
    # J70.500J00.000J00.000
    if "J0" in s:
        if "J00." in s or "J00" in s:
            return False, "Command appears to contain J0. Use J8 and J9 for external axes."

    return True, "OK"


# ------------------------------------------------------------
# RP parsing
# ------------------------------------------------------------

def between(s, start_token, end_token=None):
    start = s.find(start_token)

    if start < 0:
        return ""

    start += len(start_token)

    if end_token is None:
        return s[start:].strip()

    end = s.find(end_token, start)

    if end < 0:
        return s[start:].strip()

    return s[start:end].strip()


def clean_rp_reply(value):
    if value is None:
        return ""

    s = str(value).strip()

    for line in s.splitlines():
        line = line.strip()

        if line.startswith("A") and "B" in line and "C" in line and "P" in line:
            return line

    return ""


def parse_rp_line(rp):
    """
    Parses firmware RP-style position line.

    Firmware reports:
      A-F = J1-J6 degrees
      G-L = Cartesian pose
      M-O = diagnostics
      P-R = J7-J9 external axes

    J7 mapping (shared convention after Teensy J7 CalDir=0):
      firmware J7 = 0        -> TrackM = 0.0 (motor / home)
      firmware J7 = TrackLen -> TrackM = 1.0 (far end)
      TrackM = J7 / TrackLength
      If sticky AR4_TRACK_FIRMWARE_INVERTED is True, TrackM = 1 - J7/TrackLength.
    """
    result = {
        "parsed": False,
        "joint_values_deg": [],
        "joint_values_rad": [],
        "cartesian_pose": [],
        "external_axes_mm": [],
        "j7_mm": 0.0,
        "track_m": 0.0,
        "speed_violation": "",
        "debug": "",
        "flag": "",
        "raw": rp
    }

    if not rp:
        return result

    try:
        A = to_float(between(rp, "A", "B"))
        B = to_float(between(rp, "B", "C"))
        C = to_float(between(rp, "C", "D"))
        D = to_float(between(rp, "D", "E"))
        E = to_float(between(rp, "E", "F"))
        F = to_float(between(rp, "F", "G"))

        joint_values_deg = [A, B, C, D, E, F]
        joint_values_rad = [math.radians(v) for v in joint_values_deg]

        X = to_float(between(rp, "G", "H"))
        Y = to_float(between(rp, "H", "I"))
        Z = to_float(between(rp, "I", "J"))

        Rz = to_float(between(rp, "J", "K"))
        Ry = to_float(between(rp, "K", "L"))
        Rx = to_float(between(rp, "L", "M"))

        speed_violation = between(rp, "M", "N")
        debug = between(rp, "N", "O")
        flag = between(rp, "O", "P")

        P = to_float(between(rp, "P", "Q"))
        Q = to_float(between(rp, "Q", "R"))
        R = to_float(between(rp, "R", None))

        j7_mm = P

        track_length = 870.0
        track_inverted = False
        if sc is not None:
            track_length = to_float(sc.sticky.get("AR4_TRACK_LENGTH_MM", 870.0), 870.0)
            track_inverted = to_bool(sc.sticky.get("AR4_TRACK_FIRMWARE_INVERTED", False), False)
        if track_length <= 0.0:
            track_length = 870.0

        track_m = clamp(j7_mm / track_length, 0.0, 1.0)
        if track_inverted:
            track_m = 1.0 - track_m

        result["parsed"] = True
        result["joint_values_deg"] = joint_values_deg
        result["joint_values_rad"] = joint_values_rad
        result["cartesian_pose"] = [X, Y, Z, Rz, Ry, Rx]
        result["external_axes_mm"] = [P, Q, R]
        result["j7_mm"] = j7_mm
        result["track_m"] = track_m
        result["track_length_mm"] = track_length
        result["track_firmware_inverted"] = track_inverted
        result["speed_violation"] = speed_violation
        result["debug"] = debug
        result["flag"] = flag

        return result

    except Exception:
        return result


# ------------------------------------------------------------
# Serial helpers
# ------------------------------------------------------------

REPLY_RP = "RP"
REPLY_ERROR_ER = "ERROR_ER"
REPLY_ERROR_EL = "ERROR_EL"

# cmdBuffer1..3 in the firmware. One is executing, so two can be staged.
FIRMWARE_QUEUE_DEPTH = 3


def is_rp_line(line):
    return line.startswith("A") and "B" in line and "C" in line and "P" in line


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


RUN_QUEUE_KEY = "AR4_RUN_QUEUE_STATE"
LIVE_POSE_KEY = "AR4_LIVE_POSE"


def request_another_solve(delay_ms=40):
    """
    Schedule a later re-solve so streaming does not freeze the canvas.

    ExpireSolution(True) from inside SolveInstance can re-enter immediately and
    still feels blocked if the previous solve waited on serial. ScheduleSolution
    yields back to Rhino first.
    """
    try:
        if ghenv is None or not hasattr(ghenv, "Component"):
            return
        comp = ghenv.Component
        doc = comp.OnPingDocument()
        if doc is None:
            comp.ExpireSolution(True)
            return

        def _cb(gh_doc):
            try:
                if comp is not None:
                    comp.ExpireSolution(False)
            except Exception:
                pass

        doc.ScheduleSolution(max(1, int(delay_ms)), _cb)
    except Exception:
        try:
            if ghenv is not None:
                ghenv.Component.ExpireSolution(True)
        except Exception:
            pass


def publish_live_pose(joints_deg, track_m, j7_mm, reply_line=""):
    pose = {
        "joint_values_deg": list(joints_deg[:6]) if joints_deg else [],
        "track_m": float(track_m),
        "j7_mm": float(j7_mm),
        "raw": reply_line or "",
        "time": time.time(),
    }
    if sc is not None:
        sc.sticky[LIVE_POSE_KEY] = pose
    return pose


def find_startup_cache():
    if sc is None:
        return None, None, {}
    best = None
    for key in sc.sticky.keys():
        if not str(key).endswith("::startup_state"):
            continue
        state = sc.sticky.get(key)
        if isinstance(state, dict):
            best = state
    if not best:
        return None, None, {}
    cache_id = best.get("cache_id")
    model = sc.sticky.get("{}::model".format(cache_id)) if cache_id else None
    meshes = sc.sticky.get("{}::meshes".format(cache_id), {}) if cache_id else {}
    return cache_id, model, meshes if isinstance(meshes, dict) else {}


def resolve_tcp_local():
    if sc is None:
        return rg.Plane.WorldXY
    for key in ("AR4_TCP_FRAME", "AR4_TOOL_TCP_FRAME"):
        d = sc.sticky.get(key)
        if isinstance(d, dict) and "origin_m" in d:
            o = d["origin_m"]
            return rg.Plane(
                rg.Point3d(o[0], o[1], o[2]),
                rg.Vector3d(*d.get("xaxis", [1, 0, 0])),
                rg.Vector3d(*d.get("yaxis", [0, 1, 0])),
            )
    return rg.Plane.WorldXY


def build_live_geometry(joints_deg, track_m):
    """FK + meshes for GH preview from a live joint pose."""
    out = {
        "flange_plane": rg.Plane.Unset,
        "tcp_plane": rg.Plane.Unset,
        "robot_geometry": [],
        "tool_geometry": [],
    }
    cache_id, model, mesh_cache = find_startup_cache()
    if model is None:
        return out

    travel = 870.0
    rail = 1000.0
    if sc is not None:
        travel = to_float(sc.sticky.get("AR4_TRACK_LENGTH_MM", 870.0), 870.0)
        rail = to_float(sc.sticky.get("AR4_TRACK_RAIL_LENGTH_MM", 1000.0), 1000.0)

    try:
        config = model.zero_configuration()
        names = model.get_configurable_joint_names()
    except Exception:
        return out

    track_urdf = clamp(track_m, 0.0, 1.0) * (travel / rail if rail > 1e-9 else 0.0)
    if "track_joint" in names:
        config["track_joint"] = track_urdf
    for i in range(6):
        n = "joint_{}".format(i + 1)
        if n in names and i < len(joints_deg):
            config[n] = math.radians(joints_deg[i])

    display_scale = 1000.0
    tool_link = None
    try:
        link_names = [lnk.name for lnk in model.links]
    except Exception:
        link_names = []
    for name in ["tool0", "ee_link", "link_6", "pneumatic_gripper_tool_link"]:
        if name in link_names:
            tool_link = name
            break
    if tool_link is None and link_names:
        tool_link = link_names[-1]

    # Base plane
    base = rg.Plane.WorldXY
    if sc is not None and cache_id:
        bp = sc.sticky.get(cache_id + "::base_plane", None)
        if isinstance(bp, rg.Plane) and bp.IsValid:
            base = bp

    def to_world(plane):
        p = rg.Plane(plane)
        p.Transform(rg.Transform.PlaneToPlane(rg.Plane.WorldXY, base))
        return p

    frames = {}
    for name in link_names:
        try:
            fr = model.forward_kinematics(config, link_name=name)
            p = fr.point
            frames[name] = rg.Plane(
                rg.Point3d(float(p.x) * display_scale, float(p.y) * display_scale, float(p.z) * display_scale),
                rg.Vector3d(float(fr.xaxis.x), float(fr.xaxis.y), float(fr.xaxis.z)),
                rg.Vector3d(float(fr.yaxis.x), float(fr.yaxis.y), float(fr.yaxis.z)),
            )
        except Exception:
            pass

    flange = frames.get(tool_link)
    if flange is None:
        return out

    out["flange_plane"] = to_world(flange)
    tcp_local = resolve_tcp_local()
    tcp = rg.Plane(tcp_local)
    tcp.Origin = rg.Point3d(
        tcp.Origin.X * display_scale,
        tcp.Origin.Y * display_scale,
        tcp.Origin.Z * display_scale,
    )
    tcp.Transform(rg.Transform.PlaneToPlane(rg.Plane.WorldXY, flange))
    out["tcp_plane"] = to_world(tcp)

    scale_local = rg.Transform.Scale(rg.Point3d(0, 0, 0), display_scale)
    gripper = set(("gripper_base_link", "gripper_jaw1_link", "gripper_jaw2_link"))
    tool_meshes = []
    if sc is not None:
        tm = sc.sticky.get("AR4_TOOL_MESHES", None)
        if isinstance(tm, list):
            tool_meshes = tm
    hide_gripper = bool(tool_meshes)

    for link_name, local_meshes in mesh_cache.items():
        if link_name not in frames:
            continue
        if hide_gripper and link_name in gripper:
            continue
        xform = rg.Transform.PlaneToPlane(rg.Plane.WorldXY, to_world(frames[link_name]))
        for base_mesh in local_meshes:
            moved = base_mesh.DuplicateMesh()
            moved.Transform(scale_local)
            moved.Transform(xform)
            out["robot_geometry"].append(moved)

    if tool_meshes:
        xform = rg.Transform.PlaneToPlane(rg.Plane.WorldXY, out["flange_plane"])
        for base_mesh in tool_meshes:
            moved = base_mesh.DuplicateMesh()
            moved.Transform(xform)
            out["tool_geometry"].append(moved)

    return out


def pump_run_queue(bus, wait_for_reply, timeout_per_command, max_in_flight, delay):
    """
    Non-blocking queue pump for one GH solve.

    Sends up to the look-ahead window, drains any acks already in the SerialBus
    queue, and returns immediately if the robot is still moving. Real command
    timeouts use pending_sent_at timestamps — the solve thread never sleeps for
    TimeoutPerCommand.
    """
    state = sc.sticky.get(RUN_QUEUE_KEY)
    if not isinstance(state, dict) or not state.get("active"):
        return state, "No active run queue."

    commands = state.get("encoded_commands") or []
    pending = list(state.get("pending") or [])
    pending_sent_at = dict(state.get("pending_sent_at") or {})
    next_i = int(state.get("next_index", 0))
    results = dict(state.get("results") or {})
    sent = list(state.get("sent_commands") or [])
    window = 1 if wait_for_reply else clamp(int(max_in_flight), 1, FIRMWARE_QUEUE_DEPTH - 1)
    now = time.time()
    next_send_after = float(state.get("next_send_after", 0.0) or 0.0)

    def record(index, status_code, line):
        st = parse_rp_line(line if status_code in (REPLY_RP, "RP") else "")
        st["command_index"] = index
        st["reply_status"] = status_code if status_code != "RP" else REPLY_RP
        st["reply_line"] = line
        results[index] = st
        if st.get("parsed"):
            publish_live_pose(
                st.get("joint_values_deg") or [],
                st.get("track_m") or 0.0,
                st.get("j7_mm") or 0.0,
                line
            )

    # Fill the look-ahead window without sleeping on Delay.
    while (
        state.get("active", True)
        and len(pending) < window
        and next_i < len(commands)
        and now >= next_send_after
    ):
        cmd = commands[next_i]
        ok, message = validate_rj_command(cmd)
        if not ok:
            state["active"] = False
            state["failed_index"] = next_i
            state["failure_reason"] = message
            state["completed"] = False
            break
        try:
            bus.write_command(cmd)
        except Exception as e:
            state["active"] = False
            state["failed_index"] = next_i
            state["failure_reason"] = str(e)
            state["completed"] = False
            break
        sent.append(cmd.strip())
        pending.append(next_i)
        pending_sent_at[next_i] = now
        next_i += 1
        if delay > 0.0:
            next_send_after = now + delay
            break

    # Drain every ack that is already buffered. Never wait for the move.
    while pending and state.get("active", True):
        if hasattr(bus, "poll_ack"):
            status_code, line = bus.poll_ack(accept_any_line=False)
        else:
            status_code, line = bus.wait_ack(0.0, accept_any_line=False)

        if str(status_code).startswith("TIMEOUT"):
            oldest = pending[0]
            age = now - float(pending_sent_at.get(oldest, now))
            if age > float(timeout_per_command):
                index = pending.pop(0)
                pending_sent_at.pop(index, None)
                record(index, status_code, line)
                state["active"] = False
                state["failed_index"] = index
                state["failure_reason"] = "Command {} -> {} after {:.1f}s".format(
                    index, status_code, age
                )
                state["completed"] = False
            break

        index = pending.pop(0)
        pending_sent_at.pop(index, None)
        if status_code == "RP":
            status_code = REPLY_RP
        record(index, status_code, line)
        if status_code in (REPLY_ERROR_ER, REPLY_ERROR_EL):
            state["active"] = False
            state["failed_index"] = index
            state["failure_reason"] = "Command {} -> {}".format(index, status_code)
            state["completed"] = False
            break

    state["pending"] = pending
    state["pending_sent_at"] = pending_sent_at
    state["next_index"] = next_i
    state["next_send_after"] = next_send_after
    state["results"] = results
    state["sent_commands"] = sent

    if state.get("active") and next_i >= len(commands) and not pending:
        state["active"] = False
        state["completed"] = state.get("failed_index", -1) < 0
        state["failure_reason"] = state.get("failure_reason", "")

    sc.sticky[RUN_QUEUE_KEY] = state

    if state.get("active"):
        # Poll again soon; faster when an ack may already be inbound.
        request_another_solve(25 if pending else 40)
        oldest_age = 0.0
        if pending:
            oldest_age = now - float(pending_sent_at.get(pending[0], now))
        msg = "Streaming {}/{} (in flight {}, oldest {:.1f}s).".format(
            len(sent), len(commands), len(pending), oldest_age
        )
    elif state.get("completed"):
        msg = "Run complete. Sent {} of {}.".format(len(sent), len(commands))
    else:
        msg = "Run stopped. Sent {} of {}. Failed {}. {}".format(
            len(sent),
            len(commands),
            state.get("failed_index", -1),
            state.get("failure_reason", ""),
        )
    return state, msg


# ------------------------------------------------------------
# Outputs / defaults
# ------------------------------------------------------------

sent_commands = []
reply_lines = []
raw_replies = []
parsed_states = []
last_reply = ""
last_rp_state = {}
run_index = -1
completed = False
failed_index = -1
live_joint_values_deg = []
live_track_m = 0.0
live_j7_mm = 0.0
flange_plane = rg.Plane.Unset
tcp_plane = rg.Plane.Unset
robot_geometry = []
tool_geometry = []
status = ""


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------

try:
    if sc is None:
        raise Exception("scriptcontext is not available.")

    bus = get_connected_bus()
    port = getattr(bus, "port", "") if bus is not None else ""
    baud = getattr(bus, "baud", 0) if bus is not None else 0

    commands = to_command_list(Commands)
    program_commands = list(commands)

    encode_on_send, home_urdf_deg, joint_signs = get_encode_settings()
    send_commands, encode_maps = encode_command_list_for_firmware(
        program_commands,
        encode_on_send,
        home_urdf_deg,
        joint_signs
    )

    run = to_bool(Run, False)
    stop = to_bool(Stop, False)

    delay = to_float(Delay, 0.0)
    wait_for_reply = to_bool(WaitForReply, False)

    # Must exceed the longest single move in the program, because the firmware
    # only replies once that move has finished. A slow traverse of the full
    # track is minutes, so the old 30s default aborted mid-program.
    timeout_per_command = to_float(TimeoutPerCommand, 300.0)
    max_in_flight = int(to_float(get_optional_input("MaxInFlight", 2), 2))

    if delay < 0.0:
        delay = 0.0

    if timeout_per_command < 1.0:
        timeout_per_command = 1.0

    # Store latest visible command lists for debugging.
    sc.sticky["AR4_LAST_RUN_INPUT_COMMANDS"] = program_commands
    sc.sticky["AR4_LAST_RUN_ENCODED_COMMANDS"] = send_commands
    sc.sticky["AR4_LAST_RUN_ENCODE_MAPS"] = encode_maps

    encode_note = (
        "encode ON home={} signs={}".format(
            [round(v, 1) for v in home_urdf_deg],
            [int(v) for v in joint_signs]
        )
        if encode_on_send
        else "encode OFF (sending program RJ as-is)"
    )

    queue_state = sc.sticky.get(RUN_QUEUE_KEY) if sc is not None else None

    if stop:
        if isinstance(queue_state, dict):
            queue_state["active"] = False
            sc.sticky[RUN_QUEUE_KEY] = queue_state
        sc.sticky["AR4_RUN_LAST_STATE"] = {
            "sent_commands": [],
            "program_commands": [],
            "reply_lines": [],
            "raw_replies": [],
            "parsed_states": [],
            "completed": False,
            "failed_index": -1,
            "status": "Stopped."
        }
        status = "Stop=True. Run queue cleared."

    elif bus is None:
        status = "Serial bus not connected. Set Comp1 Connect=True and leave it on."

    else:
        edge_key = "AR4_RUN_COMPONENT_LAST_RUN_STATE"
        previous_run = sc.sticky.get(edge_key, False)
        sc.sticky[edge_key] = run

        # Start a new streaming queue on Run False->True.
        if run and not previous_run:
            if not program_commands:
                status = "No RJ commands provided."
            else:
                validation_errors = []
                for i, cmd in enumerate(send_commands):
                    ok, msg = validate_rj_command(cmd)
                    if not ok:
                        validation_errors.append("Index {}: {}".format(i, msg))
                if validation_errors:
                    status = "Command validation failed. {}".format(
                        "; ".join(validation_errors[:3])
                    )
                else:
                    bus.clear_acks()
                    queue_state = {
                        "active": True,
                        "program_commands": program_commands,
                        "encoded_commands": send_commands,
                        "next_index": 0,
                        "pending": [],
                        "sent_commands": [],
                        "results": {},
                        "failed_index": -1,
                        "failure_reason": "",
                        "completed": False,
                        "encode_note": encode_note,
                        "port": port,
                        "baud": baud,
                    }
                    sc.sticky[RUN_QUEUE_KEY] = queue_state
                    status = "Run started ({} cmds, {}).".format(
                        len(send_commands), encode_note
                    )

        # Advance an active queue every solve (ExpireSolution keeps it moving).
        queue_state = sc.sticky.get(RUN_QUEUE_KEY)
        if isinstance(queue_state, dict) and queue_state.get("active"):
            queue_state, status = pump_run_queue(
                bus,
                wait_for_reply,
                timeout_per_command,
                max_in_flight,
                delay
            )

        # Mirror queue / last state into outputs.
        if isinstance(queue_state, dict):
            sent_commands = list(queue_state.get("sent_commands") or [])
            results = queue_state.get("results") or {}
            parsed_states = []
            reply_lines = []
            raw_replies = []
            for i in range(len(sent_commands)):
                st = results.get(i) or {
                    "parsed": False,
                    "command_index": i,
                    "reply_status": "PENDING",
                    "reply_line": "",
                }
                parsed_states.append(st)
                line = st.get("reply_line", "")
                reply_lines.append([line] if line else [])
                raw_replies.append(line)
            if raw_replies:
                last_reply = raw_replies[-1]
            if parsed_states:
                last_rp_state = parsed_states[-1]
                run_index = last_rp_state.get("command_index", -1)
            completed = bool(queue_state.get("completed"))
            failed_index = int(queue_state.get("failed_index", -1))
            sc.sticky["AR4_RUN_LAST_STATE"] = {
                "sent_commands": sent_commands,
                "program_commands": queue_state.get("program_commands", []),
                "encoded_commands": queue_state.get("encoded_commands", []),
                "reply_lines": reply_lines,
                "raw_replies": raw_replies,
                "parsed_states": parsed_states,
                "completed": completed,
                "failed_index": failed_index,
                "status": status,
            }
        elif not status:
            status = "Ready. {} command(s) loaded ({}). Toggle Run True to stream.".format(
                len(program_commands),
                encode_note
            )

    # Live pose / geometry for GH preview (bus RP or last sticky pose).
    pose = None
    if bus is not None and hasattr(bus, "snapshot_pose"):
        pose = bus.snapshot_pose()
    if (not pose or not pose.get("parsed")) and sc is not None:
        pose = sc.sticky.get(LIVE_POSE_KEY)

    if isinstance(pose, dict) and (pose.get("parsed") or pose.get("joint_values_deg")):
        live_joint_values_deg = list(pose.get("joint_values_deg") or [])
        live_track_m = float(pose.get("track_m") or 0.0)
        live_j7_mm = float(pose.get("j7_mm") or 0.0)
        if to_bool(get_optional_input("DrawRobot", True), True) and live_joint_values_deg:
            geom = build_live_geometry(live_joint_values_deg, live_track_m)
            flange_plane = geom["flange_plane"]
            tcp_plane = geom["tcp_plane"]
            robot_geometry = geom["robot_geometry"]
            tool_geometry = geom["tool_geometry"]

except Exception as e:
    sent_commands = []
    reply_lines = []
    raw_replies = []
    parsed_states = []
    last_reply = ""
    last_rp_state = {}
    run_index = -1
    completed = False
    failed_index = -1
    live_joint_values_deg = []
    live_track_m = 0.0
    live_j7_mm = 0.0
    flange_plane = rg.Plane.Unset
    tcp_plane = rg.Plane.Unset
    robot_geometry = []
    tool_geometry = []
    status = "ERROR: {}".format(e)