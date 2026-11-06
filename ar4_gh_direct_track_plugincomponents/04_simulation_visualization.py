#! python3
# r: compas
# r: compas_fab
# r: compas_robots

"""
AR4 + External Track — Component 4: Simulation Visualization
Grasshopper Python 3 / Rhino 8

This component does NOT talk to serial. It has no Port/Baud inputs.

Confirmed pipeline:
  Comp3 program  : RJ in URDF / geometric degrees (NO home offset)
  Comp4 this sim : same RJ numbers -> FK (NO home offset)
  Comp5 send     : encodes HomeJointsDeg only when talking to the robot

Do NOT apply HomeJointsDeg here. That would double-shift the pose relative
to Comp3 and Rhino targets (sim looks "backwards" vs the program).

Expected GH inputs (all optional except Commands):
  Commands          : RJ command list (from Comp3, Panel, or any GH list)
  Time              : float scrubber 0..1 or index
  DrawRobot         : bool (default True)
  DrawTrack         : bool (default True)
  DrawPath          : bool (default True)
  DrawTargets       : bool (default True)
  DrawSkeleton      : bool (default False)
  TrackLengthMM     : usable J7 travel mm (default sticky / 870)
  TrackRailLengthMM : full rail visual length in mm (default 1000)
  TrackFirmwareInverted : bool (default sticky / False)
  BasePlane         : plane. Where this robot's URDF world sits.

TCP and tooling:
  TCPPlane          : plane. Full flange-local tool frame, Rhino units.
  TCPPoint          : point. Flange-local tool tip, orientation left on flange.
  ToolZ             : legacy scalar mm along flange Z.
  Unwired, the sim draws the TCP frame Comp3 solved against, so the simulated
  tool always matches the program.

  ToolMeshes        : meshes standing in for the gripper, authored around the
                      Rhino origin as though the flange sat on WorldXY.
  HideGripper       : bool. Defaults True once ToolMeshes is wired, which drops
                      the URDF gripper links from the drawing only.

Outputs add tool_geometry alongside robot_geometry.
"""

import math

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


def gh_input(name, default=None):
    """
    Read a GH input by name without raising NameError when the input
    pin is missing from the component.
    """
    return unwrap_gh(globals().get(name, default))


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


def clamp(value, low, high):
    if value < low:
        return low

    if value > high:
        return high

    return value


def to_command_list(values):
    """
    Accept a Grasshopper list, a single command string, or multiline text.
    Return a clean list of RJ command strings.
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

        # Allow one panel with many commands.
        if "\n" in s:
            for line in s.splitlines():
                line = line.strip()

                if line:
                    result.append(line)
        else:
            result.append(s)

    # Keep only likely RJ commands.
    cleaned = []

    for cmd in result:
        if cmd.strip().upper().startswith("RJ"):
            cleaned.append(cmd.strip())

    return cleaned


# ------------------------------------------------------------
# Helpers: cached startup model
# ------------------------------------------------------------

def cache_timestamp_from_id(cache_id):
    try:
        return int(str(cache_id).split("_")[-1])
    except Exception:
        return 0


def find_latest_startup_cache():
    """
    Startup component stores:
      <cache_id>::model
      <cache_id>::meshes

    This searches sticky and returns the newest cache_id.
    """
    if sc is None:
        return None, None, None

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
        return None, None, None

    candidates.sort()
    _, cid, model = candidates[-1]

    meshes = sc.sticky.get(cid + "::meshes", {})

    return cid, model, meshes


# ------------------------------------------------------------
# Helpers: command parsing
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


def normalize_rj_command(command):
    """Whitespace-insensitive RJ compare key."""
    if command is None:
        return ""
    return "".join(str(command).split()).upper()


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


def get_calibration_frame_settings(program_state=None):
    """
    Sim never applies HomeJointsDeg. A..F on Commands are URDF/program degrees
    (same as Comp3). Comp5 encodes for the physical robot only.
    """
    default_home = [-16.0, 0.0, 0.0, 0.0, 22.0, -10.0]
    default_signs = [-1.0, 1.0, 1.0, 1.0, 1.0, 1.0]

    # Force off — do not inherit GH pin / sticky True from earlier experiments.
    use_frame = False

    home = parse_float_list_input(gh_input("HomeJointsDeg", None), 6, None)
    if home is None and isinstance(program_state, dict):
        home = parse_float_list_input(program_state.get("home_urdf_deg"), 6, None)
    if home is None and sc is not None:
        home = parse_float_list_input(sc.sticky.get("AR4_HOME_URDF_DEG", None), 6, None)
    if home is None:
        home = list(default_home)

    signs = parse_float_list_input(gh_input("JointSigns", None), 6, None)
    if signs is None and isinstance(program_state, dict):
        signs = parse_float_list_input(program_state.get("joint_signs"), 6, None)
    if signs is None and sc is not None:
        signs = parse_float_list_input(sc.sticky.get("AR4_JOINT_SIGNS", None), 6, None)
    if signs is None:
        signs = list(default_signs)

    out_signs = []
    for s in signs[:6]:
        s = to_float(s, 1.0)
        if abs(s) < 1e-12:
            s = 1.0
        out_signs.append(1.0 if s >= 0.0 else -1.0)

    if sc is not None:
        sc.sticky["AR4_SIM_USE_CALIBRATION_FRAME"] = False

    return use_frame, home[:6], out_signs


def firmware_deg_to_urdf_deg(firmware_deg, home_urdf_deg, joint_signs, use_frame):
    vals = list(firmware_deg[:6])
    while len(vals) < 6:
        vals.append(0.0)

    if not use_frame:
        return vals[:6]

    out = []
    for i in range(6):
        s = joint_signs[i] if i < len(joint_signs) else 1.0
        if abs(s) < 1e-12:
            s = 1.0
        out.append(home_urdf_deg[i] + vals[i] / s)
    return out


def parse_rj_command(
    command,
    track_length_mm=870.0,
    track_firmware_inverted=False,
    use_calibration_frame=False,
    home_urdf_deg=None,
    joint_signs=None
):
    """
    Parse an RJ command into joint/track pose for FK.

    A..F are URDF/program degrees (same as Comp3). Home offset is NOT applied
    here — Comp5 encodes when sending to the robot.
    """

    result = {
        "valid": False,
        "command": command,
        "joints_deg": [0.0] * 6,
        "joints_rad": [0.0] * 6,
        "j7_mm": 0.0,
        "track_m": 0.0,
        "j8": 0.0,
        "j9": 0.0,
        "speed": 0.0,
        "accel": 0.0,
        "decel": 0.0,
        "ramp": 0.0,
        "wrist": "",
        "loop_mode": "",
        "source": "rj_parse"
    }

    if command is None:
        return result

    s = str(command).strip()

    if not s.upper().startswith("RJ"):
        return result

    try:
        j1 = to_float(between(s, "A", "B"))
        j2 = to_float(between(s, "B", "C"))
        j3 = to_float(between(s, "C", "D"))
        j4 = to_float(between(s, "D", "E"))
        j5 = to_float(between(s, "E", "F"))
        j6 = to_float(between(s, "F", "J7"))

        j7 = to_float(between(s, "J7", "J8"))
        j8 = to_float(between(s, "J8", "J9"))
        j9 = to_float(between(s, "J9", "S"))

        speed = to_float(between(s, "Sm", "Ac"))
        if abs(speed) < 1e-12:
            speed = to_float(between(s, "Sp", "Ac"))
        accel = to_float(between(s, "Ac", "Dc"))
        decel = to_float(between(s, "Dc", "Rm"))

        # Some commands may have Rnd in other branches, but our RJ builder uses Rm directly before W.
        if "Rnd" in s:
            ramp = to_float(between(s, "Rm", "Rnd"))
            wrist = between(s, "W", "Lm")
        else:
            ramp = to_float(between(s, "Rm", "W"))
            wrist = between(s, "W", "Lm")

        loop_mode = between(s, "Lm", None)

        joints_firmware_deg = [j1, j2, j3, j4, j5, j6]

        if home_urdf_deg is None:
            home_urdf_deg = [-16.0, 0.0, 0.0, 0.0, 22.0, -10.0]
        if joint_signs is None:
            joint_signs = [-1.0, 1.0, 1.0, 1.0, 1.0, 1.0]

        joints_deg = firmware_deg_to_urdf_deg(
            joints_firmware_deg,
            home_urdf_deg,
            joint_signs,
            use_calibration_frame
        )
        joints_rad = [math.radians(v) for v in joints_deg]

        length = track_length_mm if abs(track_length_mm) > 1e-9 else 870.0
        track_m = clamp(j7 / length, 0.0, 1.0)
        if track_firmware_inverted:
            track_m = 1.0 - track_m

        result["valid"] = True
        result["joints_firmware_deg"] = joints_firmware_deg
        result["joints_deg"] = joints_deg
        result["joints_rad"] = joints_rad
        result["j7_mm"] = j7
        result["track_m"] = track_m
        result["j8"] = j8
        result["j9"] = j9
        result["speed"] = speed
        result["accel"] = accel
        result["decel"] = decel
        result["ramp"] = ramp
        result["wrist"] = wrist
        result["loop_mode"] = loop_mode
        result["use_calibration_frame"] = use_calibration_frame

        return result

    except Exception:
        return result


def get_track_travel_length_mm(program_state=None):
    """Usable J7 travel (mm). GH TrackLengthMM, else sticky/program_state, else 870."""
    travel = to_float(gh_input("TrackLengthMM", None), None)
    if travel is not None and travel > 0.0:
        return travel

    if sc is not None:
        sticky = sc.sticky.get("AR4_TRACK_LENGTH_MM", None)
        if sticky is not None:
            travel = to_float(sticky, 0.0)
            if travel > 0.0:
                return travel

    if isinstance(program_state, dict):
        travel = to_float(program_state.get("track_length_mm"), 0.0)
        if travel > 0.0:
            return travel

    return 870.0


def get_track_firmware_inverted(program_state=None):
    raw = gh_input("TrackFirmwareInverted", None)
    if raw is not None:
        return to_bool(raw, False)

    if isinstance(program_state, dict) and "track_firmware_inverted" in program_state:
        return to_bool(program_state.get("track_firmware_inverted"), False)

    if sc is not None:
        sticky = sc.sticky.get("AR4_TRACK_FIRMWARE_INVERTED", None)
        if sticky is not None:
            return to_bool(sticky, False)

    return False


# ------------------------------------------------------------
# TCP frame
#
# Flange-local rigid transform to the tool tip, origin in URDF meters.
# Same dict contract as Comp3 and Comp7.
# ------------------------------------------------------------

GRIPPER_LINK_NAMES = (
    "gripper_base_link",
    "gripper_jaw1_link",
    "gripper_jaw2_link"
)


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


def plane_origin_to_meters(plane, mm_per_unit=0.001):
    p = rg.Plane(plane)
    p.Origin = rg.Point3d(
        p.Origin.X * mm_per_unit,
        p.Origin.Y * mm_per_unit,
        p.Origin.Z * mm_per_unit
    )
    return p


def resolve_tcp_frame(program_state=None):
    """
    Returns (flange-local TCP plane in URDF meters, source label).

    Comp3 publishes the frame it actually solved against, and that is what the
    sim should draw — otherwise the simulated tool sits somewhere the IK never
    aimed. The pins exist only to override that.
    """
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

    if isinstance(program_state, dict):
        d = program_state.get("tcp_frame", None)

        if isinstance(d, dict):
            return tcp_dict_to_plane(d), "Comp3 " + str(d.get("source", ""))

    if sc is not None:
        d = sc.sticky.get("AR4_LAST_PROGRAM_TCP_FRAME", None)

        if isinstance(d, dict):
            return tcp_dict_to_plane(d), "Comp3 sticky " + str(d.get("source", ""))

        d = sc.sticky.get("AR4_TCP_FRAME", None)

        if isinstance(d, dict):
            return tcp_dict_to_plane(d), "Comp7 " + str(d.get("source", ""))

        d = sc.sticky.get("AR4_TOOL_TCP_FRAME", None)

        if isinstance(d, dict):
            return tcp_dict_to_plane(d), str(d.get("source", "Comp1 tool"))

    return rg.Plane.WorldXY, "flange (no tool)"


def apply_tcp_frame(flange_plane, tcp_local, display_scale):
    """
    Carry the flange-local TCP frame onto a flange plane.

    tcp_local is in URDF meters while flange_plane is already in display
    units, so the offset is scaled before it is applied.
    """
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


def get_tool_meshes():
    """
    Meshes standing in for the gripper, authored in Rhino units around the
    origin as though the flange were sitting on WorldXY.

    Comp1 loads the tool and caches it; the pin is the per-robot override for
    when two robots share a cache_id but carry different tooling.
    """
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
    """Ride the authored tool geometry on the flange, in display units."""
    if not tool_meshes or flange_plane is None:
        return []

    geometry = []
    xform = rg.Transform.PlaneToPlane(rg.Plane.WorldXY, flange_plane)

    for base_mesh in tool_meshes:
        moved = base_mesh.DuplicateMesh()
        moved.Transform(xform)
        geometry.append(moved)

    return geometry


def get_track_rail_length_mm(program_state=None):
    """
    Full rail length drawn in Rhino (mm). Usable robot travel can be shorter.
    Optional GH input TrackRailLengthMM, else sticky, else 1000.
    """
    rail = to_float(gh_input("TrackRailLengthMM", None), None)
    if rail is not None and rail > 0.0:
        return rail

    if sc is not None:
        sticky = sc.sticky.get("AR4_TRACK_RAIL_LENGTH_MM", None)
        if sticky is not None:
            rail = to_float(sticky, 0.0)
            if rail > 0.0:
                return rail

    if isinstance(program_state, dict):
        rail = to_float(program_state.get("track_rail_length_mm"), 0.0)
        if rail > 0.0:
            return rail

    return 1000.0


def track_pose_to_urdf_meters(track_m, travel_mm=870.0, rail_mm=1000.0):
    """
    Map logical TrackM onto the URDF prismatic joint (meters along X).

    Must stay identical to Comp3 logical_track_m_to_urdf_m so the sim carriage
    sits where the IK solved. Firmware J7 is never used to place the carriage:
    when TrackFirmwareInverted is True, J7 runs opposite to TrackM and driving
    FK from it would mirror the sim along the rail.

    TrackM 0..1 = fraction of usable travel (0..travel_mm). Rail visual length
    is rail_mm (default 1000), so TrackM=1 sits at travel_mm, not the rail end.
    """
    travel_mm = abs(to_float(travel_mm, 870.0))
    if travel_mm < 1e-9:
        travel_mm = 870.0

    rail_mm = abs(to_float(rail_mm, 1000.0))
    if rail_mm < 1e-9:
        rail_mm = 1000.0

    return clamp(to_float(track_m, 0.0), 0.0, 1.0) * (travel_mm / rail_mm)


def track_m_to_j7_mm(track_m, travel_mm, track_firmware_inverted):
    """Logical TrackM -> firmware J7 mm. Mirrors Comp3 track_m_to_j7_mm."""
    travel_mm = abs(to_float(travel_mm, 870.0))
    if travel_mm < 1e-9:
        travel_mm = 870.0

    t = clamp(to_float(track_m, 0.0), 0.0, 1.0)

    if track_firmware_inverted:
        return (1.0 - t) * travel_mm

    return t * travel_mm

def pose_from_metadata(md, track_length_mm, track_firmware_inverted, command):
    """Build a pose record from Comp3 command_metadata if URDF joints are present."""
    if not isinstance(md, dict):
        return None

    joints_rad = None
    try:
        vals = list(md.get("urdf_joint_values_rad", []))
        if len(vals) >= 6:
            joints_rad = [to_float(v, 0.0) for v in vals[:6]]
    except Exception:
        joints_rad = None

    if joints_rad is None:
        try:
            vals = list(md.get("urdf_joint_values_deg", []))
            if len(vals) >= 6:
                joints_rad = [math.radians(to_float(v, 0.0)) for v in vals[:6]]
        except Exception:
            joints_rad = None

    if joints_rad is None:
        return None

    track_m = None
    if "track_m" in md and md.get("track_m") is not None:
        track_m = to_float(md.get("track_m"), 0.0)

    j7_mm = None
    if "j7_mm" in md and md.get("j7_mm") is not None:
        j7_mm = to_float(md.get("j7_mm"), 0.0)

    if track_m is None:
        if j7_mm is None:
            j7_mm = 0.0
        track_m = (
            clamp(j7_mm / track_length_mm, 0.0, 1.0)
            if abs(track_length_mm) > 1e-9
            else 0.0
        )
        if track_firmware_inverted:
            track_m = 1.0 - track_m

    j7_resolved = track_m_to_j7_mm(
        track_m,
        track_length_mm,
        track_firmware_inverted
    )

    return {
        "valid": True,
        "command": command,
        "joints_rad": joints_rad[:6],
        "joints_deg": [math.degrees(v) for v in joints_rad[:6]],
        "track_m": clamp(track_m, 0.0, 1.0),
        "j7_mm": j7_resolved,
        "source": "command_metadata_urdf"
    }


def find_matching_metadata(cmd, index, metadata, sticky_commands):
    """
    Use sticky Comp3 metadata only when the command string matches.
    Never apply metadata by index alone — that breaks Panel/manual RJ lists.
    """
    key = normalize_rj_command(cmd)
    if not key:
        return None

    if index < len(metadata) and isinstance(metadata[index], dict):
        md = metadata[index]
        md_cmd = md.get("command")
        if md_cmd is None and index < len(sticky_commands):
            md_cmd = sticky_commands[index]
        if normalize_rj_command(md_cmd) == key:
            return md

    for i, md in enumerate(metadata):
        if not isinstance(md, dict):
            continue
        md_cmd = md.get("command")
        if md_cmd is None and i < len(sticky_commands):
            md_cmd = sticky_commands[i]
        if normalize_rj_command(md_cmd) == key:
            return md

    return None


def load_program_pose_sequence(commands):
    """
    Build one pose record per GH Commands entry.

    GH Commands are the simulation source of truth.
    Sticky Comp3 metadata enriches a command only when that exact RJ string
    matches the last program; otherwise the RJ string is parsed directly.
    """
    program_state = {}
    if sc is not None:
        state = sc.sticky.get("AR4_LAST_PROGRAM_STATE", {})
        if isinstance(state, dict):
            program_state = state

    track_length_mm = get_track_travel_length_mm(program_state)
    track_firmware_inverted = get_track_firmware_inverted(program_state)
    tcp_local, tcp_source = resolve_tcp_frame(program_state)
    use_calibration_frame, home_urdf_deg, joint_signs = get_calibration_frame_settings(
        program_state
    )

    metadata = []
    sticky_commands = []
    if sc is not None:
        meta = sc.sticky.get("AR4_LAST_PROGRAM_COMMAND_METADATA", None)
        if isinstance(meta, list):
            metadata = meta
        elif isinstance(program_state.get("command_metadata"), list):
            metadata = program_state.get("command_metadata")

        sticky_cmds = sc.sticky.get("AR4_LAST_PROGRAM_COMMANDS", None)
        if isinstance(sticky_cmds, list):
            sticky_commands = sticky_cmds

    poses = []
    matched_metadata_count = 0

    for i, cmd in enumerate(commands):
        record = {
            "valid": False,
            "command": cmd,
            "joints_deg": [0.0] * 6,
            "joints_rad": [0.0] * 6,
            "track_m": 0.0,
            "j7_mm": 0.0,
            "source": "none"
        }

        md = find_matching_metadata(cmd, i, metadata, sticky_commands)
        if md is not None:
            from_md = pose_from_metadata(
                md, track_length_mm, track_firmware_inverted, cmd
            )
            if from_md is not None:
                poses.append(from_md)
                matched_metadata_count += 1
                continue

        parsed = parse_rj_command(
            cmd,
            track_length_mm,
            track_firmware_inverted,
            use_calibration_frame=use_calibration_frame,
            home_urdf_deg=home_urdf_deg,
            joint_signs=joint_signs
        )
        if parsed["valid"]:
            parsed["source"] = (
                "rj_parse_calibrated" if use_calibration_frame else "rj_parse"
            )
            poses.append(parsed)
        else:
            poses.append(record)

    program_state = dict(program_state)
    program_state["_matched_metadata_count"] = matched_metadata_count
    program_state["_pose_from_rj_parse"] = matched_metadata_count == 0
    program_state["use_calibration_frame"] = use_calibration_frame
    program_state["home_urdf_deg"] = list(home_urdf_deg)
    program_state["joint_signs"] = list(joint_signs)
    program_state["tcp_source"] = tcp_source

    return (
        poses,
        tcp_local,
        tcp_source,
        track_length_mm,
        track_firmware_inverted,
        program_state
    )


# ------------------------------------------------------------
# Helpers: COMPAS / Rhino geometry
# ------------------------------------------------------------

def make_config(model, track_m, joints_rad, travel_mm=870.0, rail_mm=1000.0):
    """
    track_m is logical usable-travel fraction 0..1.
    URDF track_joint is meters along the full rail; max usable = travel_mm/rail_mm.
    """
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


# ------------------------------------------------------------
# Robot base plane
# ------------------------------------------------------------

# FK runs in the robot's own URDF frame. Every piece of display geometry passes
# through frame_to_plane / frame_point / scale_plane_to_display, so applying the
# base transform there places the whole robot without touching the FK or the
# draw functions. None means the robot sits on WorldXY (previous behaviour).
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


def resolve_base_plane(cache_id, program_state):
    """
    BasePlane pin wins, then the plane Comp3 solved against, then Comp1's
    per-cache plane, then WorldXY. Wire the pin when several robots share a
    URDF, since they also share the cache_id.
    """
    plane = to_plane(gh_input("BasePlane", None), None)

    if plane is not None:
        return plane, "BasePlane input"

    if isinstance(program_state, dict):
        plane = to_plane(program_state.get("base_plane", None), None)

        if plane is not None:
            return plane, "Comp3 program_state"

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


def frame_to_plane(frame, display_scale=1.0):
    """
    COMPAS frames are in URDF meters. Rhino document units are millimeters,
    so display_scale should normally be 1000.
    """
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


def scale_plane_to_display(plane, display_scale):
    if plane is None or not getattr(plane, "IsValid", False):
        return plane
    s = float(display_scale)
    if abs(s - 1.0) < 1e-12:
        return to_world_plane(rg.Plane(plane))
    p = rg.Plane(plane)
    p.Origin = rg.Point3d(p.Origin.X * s, p.Origin.Y * s, p.Origin.Z * s)
    return to_world_plane(p)


def get_display_scale(program_state):
    """
    URDF/COMPAS = meters. Rhino targets in this project = millimeters.
    Display geometry must be scaled back to mm so it overlays Rhino targets.
    """
    if not isinstance(program_state, dict):
        return 1000.0

    if to_bool(program_state.get("target_units_mm"), True):
        return 1000.0

    target_scale = to_float(program_state.get("target_scale"), 0.001)
    if abs(target_scale) > 1e-12 and abs(target_scale - 1.0) > 1e-9:
        return 1.0 / target_scale

    return 1.0


def choose_tool_link(model):
    try:
        link_names = [link.name for link in model.links]
    except Exception:
        link_names = []

    preferred = [
        "tool0",
        "tool_link",
        "pneumatic_gripper_tool_link",
        "ee_link",
        "link_6"
    ]

    for name in preferred:
        if name in link_names:
            return name

    if link_names:
        return link_names[-1]

    return None


def offset_plane_along_z(plane, distance):
    p = rg.Plane(plane)

    z = rg.Vector3d(p.ZAxis)
    if z.Length > 1e-12:
        z.Unitize()

    p.Origin = p.Origin + z * distance

    return p


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
    """
    Mesh geometry is authored in meters. Scale local meshes to display units,
    then place them on display-scaled link planes.

    exclude_links drops links from the drawing only; they stay in the URDF so
    FK and the joint chain are untouched.
    """
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
    """
    Draw full rail at rail_length_mm (Rhino units when display_scale=1000),
    and carriage at its FK frame (which only travels usable length).
    """
    geometry = []
    s = float(display_scale)
    rail_mm = abs(to_float(rail_length_mm, 1000.0))
    if rail_mm < 1e-9:
        rail_mm = 1000.0

    # URDF proxy sizes are in meters; convert via display_scale to Rhino units.
    rail_len_m = rail_mm / 1000.0
    rail_pad_m = 0.05  # slight overhang past usable ends

    if "track_base_link" in frame_by_link:
        base_plane = frame_to_plane(frame_by_link["track_base_link"], display_scale)
    else:
        base_plane = rg.Plane.WorldXY

    if "track_carriage_link" in frame_by_link:
        carriage_plane = frame_to_plane(frame_by_link["track_carriage_link"], display_scale)
    else:
        carriage_plane = rg.Plane.WorldXY

    rail = make_box_at_plane(
        base_plane,
        ((rail_len_m * 0.5) * s, 0.0, -0.04 * s),
        ((rail_len_m + rail_pad_m) * s, 0.08 * s, 0.05 * s)
    )

    carriage = make_box_at_plane(
        carriage_plane,
        (0.0, 0.0, -0.005 * s),
        (0.14 * s, 0.14 * s, 0.07 * s)
    )

    geometry.append(rail)
    geometry.append(carriage)

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


def make_path_curve(points):
    if len(points) < 2:
        return None

    poly = rg.Polyline(points)

    return rg.PolylineCurve(poly)


def plane_marker(plane, size):
    """
    Draw plane axes as simple lines.
    X and Y axes are drawn; Z is drawn half-length.
    size is in the same units as plane.Origin (mm when display_scale=1000).
    """
    lines = []

    origin = plane.Origin

    x = rg.Vector3d(plane.XAxis)
    y = rg.Vector3d(plane.YAxis)
    z = rg.Vector3d(plane.ZAxis)

    x.Unitize()
    y.Unitize()
    z.Unitize()

    lines.append(rg.Line(origin, origin + x * size))
    lines.append(rg.Line(origin, origin + y * size))
    lines.append(rg.Line(origin, origin + z * (size * 0.5)))

    return lines


# ------------------------------------------------------------
# Outputs / defaults
# ------------------------------------------------------------

active_index = -1
active_command = None
active_joint_values_deg = []
active_joint_values_rad = []
active_track_m = 0.0
active_j7_mm = 0.0

robot_geometry = []
tool_geometry = []
track_geometry = []
tcp_planes = []
tcp_path = None
target_geometry = []
skeleton_lines = []

simulation_state = {}
status = ""


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------

try:
    if sc is None:
        raise Exception("scriptcontext is not available.")

    # Never use bare GH names here — avoids NameError if pins differ.
    commands = to_command_list(gh_input("Commands", None))

    draw_robot = to_bool(gh_input("DrawRobot", True), True)
    draw_track = to_bool(gh_input("DrawTrack", True), True)
    draw_path = to_bool(gh_input("DrawPath", True), True)
    draw_targets = to_bool(gh_input("DrawTargets", True), True)
    draw_skeleton = to_bool(gh_input("DrawSkeleton", False), False)
    time_value = to_float(gh_input("Time", 0.0), 0.0)

    if not commands:
        status = (
            "Comp4 simulation | No RJ commands on Commands input. "
            "Wire Comp3 rj_commands or paste RJ strings from a Panel."
        )

    else:
        cache_id, model, mesh_cache = find_latest_startup_cache()

        if model is None:
            raise Exception("No cached robot model found. Run the Startup component first.")

        tool_link = choose_tool_link(model)

        if not tool_link:
            raise Exception("Could not identify a tool/end-effector link.")

        (
            poses,
            tcp_local,
            tcp_source,
            track_length_mm,
            track_firmware_inverted,
            program_state
        ) = load_program_pose_sequence(commands)

        display_scale = get_display_scale(program_state)
        track_rail_length_mm = get_track_rail_length_mm(program_state)
        if sc is not None:
            sc.sticky["AR4_TRACK_RAIL_LENGTH_MM"] = track_rail_length_mm
            sc.sticky["AR4_TRACK_LENGTH_MM"] = track_length_mm

        # Must be set before any display geometry is built.
        base_plane, base_plane_source = resolve_base_plane(cache_id, program_state)
        set_base_xform(base_plane)

        tool_meshes = get_tool_meshes()

        # Default to hiding the URDF gripper as soon as a tool mesh is wired,
        # otherwise the servo gripper and the tool draw on top of each other.
        hide_gripper = to_bool(
            gh_input("HideGripper", bool(tool_meshes)),
            bool(tool_meshes)
        )
        excluded_links = GRIPPER_LINK_NAMES if hide_gripper else ()

        parsed = [p for p in poses if p.get("valid")]

        if not parsed:
            status = (
                "Comp4 simulation | Commands provided, but none parsed as RJ. "
                "Expected strings starting with RJ (A..F, J7..)."
            )

        else:
            # Prefer full pose list aligned to commands (including invalid slots skipped above).
            # Re-index against all poses that are valid; keep command list length for Time.
            pose_by_command = poses

            # Determine active command index.
            t = time_value

            if len(commands) == 1:
                active_index = 0

            elif 0.0 <= t <= 1.0:
                active_index = int(round(t * (len(commands) - 1)))
                active_index = int(clamp(active_index, 0, len(commands) - 1))

            else:
                active_index = int(round(t))
                active_index = int(clamp(active_index, 0, len(commands) - 1))

            # Build TCP planes for all commands that have valid poses.
            # FK is in meters; scale to Rhino mm for overlay with authored targets.
            tcp_points = []

            for p in pose_by_command:
                if not p.get("valid"):
                    tcp_planes.append(rg.Plane.Unset)
                    continue

                config = make_config(
                    model,
                    p["track_m"],
                    p["joints_rad"],
                    travel_mm=track_length_mm,
                    rail_mm=track_rail_length_mm
                )

                frame = safe_fk(model, config, tool_link)

                if frame is None:
                    tcp_planes.append(rg.Plane.Unset)
                    continue

                flange_plane = frame_to_plane(frame, display_scale)
                tcp_plane = apply_tcp_frame(flange_plane, tcp_local, display_scale)

                tcp_planes.append(tcp_plane)
                tcp_points.append(tcp_plane.Origin)

            if draw_path:
                tcp_path = make_path_curve(tcp_points)

            # Active state.
            active = pose_by_command[active_index]
            if not active.get("valid"):
                # Fall back to nearest valid pose.
                for j in range(len(pose_by_command)):
                    if pose_by_command[j].get("valid"):
                        active = pose_by_command[j]
                        active_index = j
                        break

            active_command = active.get("command")
            active_joint_values_deg = active.get("joints_deg", [])
            active_joint_values_rad = active.get("joints_rad", [])
            active_track_m = active.get("track_m", 0.0)
            active_j7_mm = active.get("j7_mm", 0.0)
            active_urdf_track_m = track_pose_to_urdf_meters(
                track_m=active_track_m,
                travel_mm=track_length_mm,
                rail_mm=track_rail_length_mm
            )

            active_config = make_config(
                model,
                active_track_m,
                active_joint_values_rad,
                travel_mm=track_length_mm,
                rail_mm=track_rail_length_mm
            )

            frame_by_link = get_frame_by_link(model, active_config)

            if draw_robot:
                robot_geometry = draw_robot_meshes(
                    mesh_cache,
                    frame_by_link,
                    display_scale,
                    exclude_links=excluded_links
                )

            if tool_meshes and tool_link in frame_by_link:
                tool_geometry = draw_tool_meshes(
                    tool_meshes,
                    frame_to_plane(frame_by_link[tool_link], display_scale)
                )

            if draw_track:
                track_geometry = draw_track_proxy(
                    frame_by_link,
                    display_scale,
                    rail_length_mm=track_rail_length_mm
                )

            if draw_skeleton:
                skeleton_lines = draw_skeleton_lines(model, frame_by_link, display_scale)

            # Sticky Comp3 target planes only when this command list matched that program.
            # Otherwise draw TCP planes from the simulated Commands themselves.
            if draw_targets:
                matched_meta = int(program_state.get("_matched_metadata_count", 0) or 0)
                use_program_targets = matched_meta > 0
                target_planes = None

                if use_program_targets and sc is not None:
                    target_planes = sc.sticky.get("AR4_LAST_PROGRAM_TARGET_PLANES", None)

                if not target_planes:
                    for pl in tcp_planes:
                        if isinstance(pl, rg.Plane):
                            target_geometry.extend(plane_marker(pl, 40.0))
                else:
                    for pl in target_planes:
                        if isinstance(pl, rg.Plane):
                            pl_disp = scale_plane_to_display(pl, display_scale)
                            target_geometry.extend(plane_marker(pl_disp, 40.0))

            meta_count = 0
            if sc is not None:
                meta = sc.sticky.get("AR4_LAST_PROGRAM_COMMAND_METADATA", None)
                if isinstance(meta, list):
                    meta_count = len(meta)

            matched_meta = int(program_state.get("_matched_metadata_count", 0) or 0)

            simulation_state = {
                "cache_id": cache_id,
                "command_count": len(commands),
                "valid_pose_count": len(parsed),
                "metadata_count": meta_count,
                "matched_metadata_count": matched_meta,
                "active_index": active_index,
                "active_command": active_command,
                "active_track_m": active_track_m,
                "active_j7_mm": active_j7_mm,
                "active_urdf_track_m": active_urdf_track_m,
                "active_joint_values_deg": active_joint_values_deg,
                "active_joint_values_rad": active_joint_values_rad,
                "active_pose_source": active.get("source"),
                "tool_link": tool_link,
                "tcp_source": tcp_source,
                "tcp_origin_m": [
                    tcp_local.Origin.X,
                    tcp_local.Origin.Y,
                    tcp_local.Origin.Z
                ],
                "tool_mesh_count": len(tool_meshes),
                "gripper_hidden": bool(hide_gripper),
                "display_scale": display_scale,
                "display_units": "mm" if abs(display_scale - 1000.0) < 1e-6 else "model",
                "track_firmware_inverted": track_firmware_inverted,
                "track_length_mm": track_length_mm,
                "track_rail_length_mm": track_rail_length_mm,
                "base_plane": base_plane,
                "base_plane_source": base_plane_source,
                "use_calibration_frame": False,
                "home_urdf_deg": list(program_state.get("home_urdf_deg", [])),
                "pose_source_note": (
                    "GH Commands = URDF/program degrees (no home offset). "
                    "Comp5 encodes for robot. meta match {}/{}. rail={}mm travel={}mm"
                ).format(
                    matched_meta,
                    len(commands),
                    track_rail_length_mm,
                    track_length_mm
                )
            }

            status = (
                "Comp4 simulation | "
                "active index {}/{} | "
                "source={} | "
                "home decode=OFF | "
                "meta match {}/{} | "
                "rail {:.0f} mm | travel {:.0f} mm | "
                "TrackM {:.4f} | J7 {:.1f} mm | URDF track {:.4f} m | "
                "base {} | TCP {} | tool meshes {}{} | "
                "robot geometry {} | path points {}"
            ).format(
                active_index,
                len(commands) - 1,
                active.get("source"),
                matched_meta,
                len(commands),
                track_rail_length_mm,
                track_length_mm,
                active_track_m,
                active_j7_mm,
                active_urdf_track_m,
                base_plane_source,
                tcp_source,
                len(tool_meshes),
                " (gripper hidden)" if hide_gripper else "",
                len(robot_geometry),
                len(tcp_points)
            )

except Exception as e:
    active_index = -1
    active_command = None
    active_joint_values_deg = []
    active_joint_values_rad = []
    active_track_m = 0.0
    active_j7_mm = 0.0

    robot_geometry = []
    tool_geometry = []
    track_geometry = []
    tcp_planes = []
    tcp_path = None
    target_geometry = []
    skeleton_lines = []

    simulation_state = {}
    status = "Comp4 simulation ERROR: {}".format(e)