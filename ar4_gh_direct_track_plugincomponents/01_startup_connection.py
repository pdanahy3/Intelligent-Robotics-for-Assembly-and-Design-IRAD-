#! python3
# r: pyserial
# r: compas
# r: compas_fab
# r: compas_robots

# =============================================================================
# GH_PINS — Rhino does not create pins from pasted code. Copy these names into
# the Python Script component UI (Type Hint / Access in COMPONENT_PINS.txt).
# INPUTS:
#   Load
#   Reload
#   URDF
#   PackageRoot
#   PreferredPort
#   Baud
#   TestCommand
#   Connect
#   BasePlane
#   ToolMeshes
#   ToolTCPPlane
#   ToolTCPPoint
# OUTPUTS:
#   cache_id
#   robot_model
#   robot_name
#   joint_names
#   motion_joint_names
#   link_names
#   mesh_link_names
#   mesh_count
#   available_ports
#   active_port
#   connected
#   handshake_reply
#   live_joint_values_deg
#   live_j7_mm
#   live_track_m
#   live_rp_raw
#   base_plane
#   tool_meshes
#   tool_tcp_plane
#   startup_state
#   status
# =============================================================================

"""
AR4 + External Track — Component 1: Startup / Connection

Loads and caches the robot, the tool and the robot's placement, then seeds
the sticky defaults every other component reads.

SERIAL OWNERSHIP
----------------
This component owns the COM port. Leave Connect=True for the session; the
handle stays open in sticky AR4_SERIAL_BUS and Comp2 / Comp5 / Comp6 send
through it. Set Connect=False to disconnect. Opening the port resets the
Teensy, so only Comp1 should ever open it.

TOOL INPUTS (all optional)
--------------------------
ToolMeshes    : Mesh parameter, List access. Tool geometry authored around
                the Rhino origin as though the flange were sitting on
                WorldXY, in document units. Breps are meshed on the way in.
ToolTCPPlane  : nominal flange-local TCP frame, Rhino units.
ToolTCPPoint  : nominal flange-local tool tip. Fixes where the tip is but not
                how the tool is turned, so target plane orientation still
                resolves against the flange until Comp7 measures the frame.

The tool is cached to AR4_TOOL_MESHES and AR4_TOOL_TCP_FRAME. Comp3 / Comp4 /
Comp5 / Comp6 read those from sticky only — they do not take ToolMeshes or
TCP pins. Comp7 calibration (AR4_TCP_FRAME) overrides the nominal Comp1 TCP
when present.
"""

import os
import time
import math
import struct
import threading
import xml.etree.ElementTree as ET

import Rhino.Geometry as rg

try:
    import scriptcontext as sc
except Exception:
    sc = None

SERIAL_BUS_KEY = "AR4_SERIAL_BUS"
# Raw pyserial handle mirror so a leaked bus can still release COM on retry.
SERIAL_HANDLE_KEY = "AR4_SERIAL_PORT_HANDLE"


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


def safe_path(value):
    value = to_text(value, "")
    return value.strip().strip('"').strip("'")


def get_optional_input(name, default=None):
    """Reads a GH input that may not exist yet on older definitions."""
    return unwrap_gh(globals().get(name, default))


def to_plane(value, default=None):
    """
    Robot base plane in Rhino document units (mm).

    Origin is where URDF 'world' sits; the plane axes orient the whole robot,
    so several robots can share one URDF at different locations/rotations.
    """
    value = unwrap_gh(value)

    if value is None:
        return default

    if isinstance(value, rg.Plane):
        if value.IsValid:
            return rg.Plane(value)
        return default

    # Accept a Point3d/point-like as origin-only placement.
    try:
        return rg.Plane(
            rg.Point3d(float(value.X), float(value.Y), float(value.Z)),
            rg.Vector3d.XAxis,
            rg.Vector3d.YAxis
        )
    except Exception:
        return default


def make_cache_id(urdf_path):
    base = os.path.basename(urdf_path)
    root, _ = os.path.splitext(base)

    try:
        stamp = int(os.path.getmtime(urdf_path))
    except Exception:
        stamp = 0

    clean = "".join([c if c.isalnum() else "_" for c in root])

    return "AR4_STARTUP_{}_{}".format(clean, stamp)


# ------------------------------------------------------------
# Helpers: URDF / mesh loading
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


def resolve_mesh_path(filename, urdf_path, package_root):
    if not filename:
        return None

    filename = filename.strip()

    # package://annin_ar4_description/meshes/...
    if filename.startswith("package://"):
        rel = filename.replace("package://", "", 1)
        parts = rel.replace("\\", "/").split("/")
        return os.path.join(package_root, *parts)

    # file:///C:/...
    if filename.startswith("file:///"):
        return filename.replace("file:///", "", 1)

    # file://C:/...
    if filename.startswith("file://"):
        return filename.replace("file://", "", 1)

    # absolute path
    if os.path.isabs(filename):
        return filename

    # relative to URDF folder
    return os.path.join(os.path.dirname(urdf_path), filename)


def add_triangle(mesh, v0, v1, v2):
    i0 = mesh.Vertices.Add(float(v0[0]), float(v0[1]), float(v0[2]))
    i1 = mesh.Vertices.Add(float(v1[0]), float(v1[1]), float(v1[2]))
    i2 = mesh.Vertices.Add(float(v2[0]), float(v2[1]), float(v2[2]))
    mesh.Faces.AddFace(i0, i1, i2)


def load_binary_stl(path):
    mesh = rg.Mesh()

    with open(path, "rb") as f:
        data = f.read()

    if len(data) < 84:
        return None

    tri_count = struct.unpack("<I", data[80:84])[0]
    expected = 84 + tri_count * 50

    if expected > len(data):
        return None

    offset = 84

    for _ in range(tri_count):
        # normal = data[offset:offset+12]
        offset += 12

        v0 = struct.unpack("<fff", data[offset:offset + 12])
        offset += 12

        v1 = struct.unpack("<fff", data[offset:offset + 12])
        offset += 12

        v2 = struct.unpack("<fff", data[offset:offset + 12])
        offset += 12

        # attribute byte count
        offset += 2

        add_triangle(mesh, v0, v1, v2)

    mesh.Normals.ComputeNormals()
    mesh.Compact()

    return mesh


def load_ascii_stl(path):
    mesh = rg.Mesh()
    tri = []

    with open(path, "r", errors="ignore") as f:
        for line in f:
            s = line.strip()

            if s.lower().startswith("vertex"):
                parts = s.split()

                if len(parts) >= 4:
                    v = (float(parts[1]), float(parts[2]), float(parts[3]))
                    tri.append(v)

                    if len(tri) == 3:
                        add_triangle(mesh, tri[0], tri[1], tri[2])
                        tri = []

    if mesh.Faces.Count == 0:
        return None

    mesh.Normals.ComputeNormals()
    mesh.Compact()

    return mesh


def load_stl(path):
    # Try binary first.
    try:
        mesh = load_binary_stl(path)

        if mesh is not None and mesh.Faces.Count > 0:
            return mesh
    except Exception:
        pass

    # Then ASCII.
    try:
        mesh = load_ascii_stl(path)

        if mesh is not None and mesh.Faces.Count > 0:
            return mesh
    except Exception:
        pass

    return None


def load_obj(path):
    verts = []
    mesh = rg.Mesh()

    with open(path, "r", errors="ignore") as f:
        for line in f:
            s = line.strip()

            if not s or s.startswith("#"):
                continue

            if s.startswith("v "):
                parts = s.split()

                if len(parts) >= 4:
                    verts.append((
                        float(parts[1]),
                        float(parts[2]),
                        float(parts[3])
                    ))

            elif s.startswith("f "):
                parts = s.split()[1:]
                indices = []

                for part in parts:
                    token = part.split("/")[0]

                    if not token:
                        continue

                    idx = int(token)

                    if idx < 0:
                        idx = len(verts) + idx
                    else:
                        idx = idx - 1

                    indices.append(idx)

                if len(indices) >= 3:
                    # triangulate fan
                    for i in range(1, len(indices) - 1):
                        v0 = verts[indices[0]]
                        v1 = verts[indices[i]]
                        v2 = verts[indices[i + 1]]
                        add_triangle(mesh, v0, v1, v2)

    if mesh.Faces.Count == 0:
        return None

    mesh.Normals.ComputeNormals()
    mesh.Compact()

    return mesh


def load_mesh_file(path):
    ext = os.path.splitext(path)[1].lower()

    if ext == ".stl":
        return load_stl(path)

    if ext == ".obj":
        return load_obj(path)

    # DAE and other formats are not loaded here.
    return None


def transform_mesh_from_urdf_visual(mesh, xyz, rpy, scale):
    moved = mesh.DuplicateMesh()

    sx, sy, sz = scale
    tx, ty, tz = xyz
    roll, pitch, yaw = rpy

    # Apply mesh scale manually.
    for i in range(moved.Vertices.Count):
        p = moved.Vertices[i]
        p.X *= sx
        p.Y *= sy
        p.Z *= sz
        moved.Vertices.SetVertex(i, p)

    origin = rg.Point3d(0, 0, 0)

    # URDF rpy convention: roll X, pitch Y, yaw Z.
    moved.Transform(rg.Transform.Rotation(roll, rg.Vector3d.XAxis, origin))
    moved.Transform(rg.Transform.Rotation(pitch, rg.Vector3d.YAxis, origin))
    moved.Transform(rg.Transform.Rotation(yaw, rg.Vector3d.ZAxis, origin))
    moved.Transform(rg.Transform.Translation(tx, ty, tz))

    moved.Normals.ComputeNormals()
    moved.Compact()

    return moved


def parse_mesh_cache_from_urdf(urdf_path, package_root):
    mesh_cache = {}
    mesh_load_report = []

    if not os.path.exists(urdf_path):
        return mesh_cache, ["URDF file does not exist: {}".format(urdf_path)]

    try:
        tree = ET.parse(urdf_path)
        root = tree.getroot()
    except Exception as e:
        return mesh_cache, ["Could not parse URDF XML: {}".format(e)]

    for link in root.findall("link"):
        link_name = link.attrib.get("name", "")

        if not link_name:
            continue

        for visual in link.findall("visual"):
            origin = visual.find("origin")

            if origin is not None:
                xyz = parse_float_list(origin.attrib.get("xyz", None), [0.0, 0.0, 0.0])
                rpy = parse_float_list(origin.attrib.get("rpy", None), [0.0, 0.0, 0.0])
            else:
                xyz = [0.0, 0.0, 0.0]
                rpy = [0.0, 0.0, 0.0]

            geometry = visual.find("geometry")

            if geometry is None:
                continue

            mesh_node = geometry.find("mesh")

            if mesh_node is None:
                # Primitive boxes/cylinders are handled by the visualization component.
                continue

            filename = mesh_node.attrib.get("filename", "")
            scale = parse_float_list(mesh_node.attrib.get("scale", None), [1.0, 1.0, 1.0])

            mesh_path = resolve_mesh_path(filename, urdf_path, package_root)

            if not mesh_path or not os.path.exists(mesh_path):
                mesh_load_report.append(
                    "Missing mesh for link {}: {}".format(link_name, mesh_path)
                )
                continue

            mesh = load_mesh_file(mesh_path)

            if mesh is None:
                mesh_load_report.append(
                    "Could not load mesh format for link {}: {}".format(link_name, mesh_path)
                )
                continue

            mesh = transform_mesh_from_urdf_visual(mesh, xyz, rpy, scale)

            if link_name not in mesh_cache:
                mesh_cache[link_name] = []

            mesh_cache[link_name].append(mesh)

    return mesh_cache, mesh_load_report


# ------------------------------------------------------------
# Tool definition
#
# A tool is the geometry bolted to the flange plus the nominal TCP it is
# expected to reach. Both are flange-local, authored in Rhino units around
# the origin as though the flange were sitting on WorldXY.
#
# This belongs here rather than in Comp4/Comp6 because file reads and Brep
# meshing then happen once per Load instead of on every Grasshopper solve,
# and because the sim and the jog component cannot disagree about which
# tool is fitted when they both read one cache.
# ------------------------------------------------------------

def to_mesh_list(value):
    """Meshes and Breps off a GH Mesh parameter -> list of meshes."""
    value = unwrap_gh(value)

    if value is None:
        return []

    try:
        items = list(value)
    except TypeError:
        items = [value]

    meshes = []

    for item in items:
        item = unwrap_gh(item)

        if item is None:
            continue

        if isinstance(item, rg.Mesh):
            meshes.append(item.DuplicateMesh())
        elif isinstance(item, rg.Brep):
            for m in (rg.Mesh.CreateFromBrep(item, rg.MeshingParameters.Default) or []):
                meshes.append(m)

    return meshes


def build_tool_tcp_frame(plane_value, point_value):
    """
    Nominal flange-local TCP -> the dict Comp3/Comp4/Comp6/Comp7 share.

    Origins are stored in URDF meters to match the rest of that contract,
    even though they are authored here in Rhino millimeters.

    A point alone fixes where the tip sits but not how the tool is turned,
    so the axes stay parallel to the flange and target plane orientation
    still resolves against the flange until Comp7 measures the real frame.
    """
    plane = to_plane(plane_value, None)
    source = "Comp1 ToolTCPPlane"

    if plane is None:
        point = to_plane(point_value, None)

        if point is None:
            return None, ""

        plane = rg.Plane(point.Origin, rg.Vector3d.XAxis, rg.Vector3d.YAxis)
        source = "Comp1 ToolTCPPoint (no orientation)"

    return {
        "origin_m": [
            float(plane.Origin.X) * 0.001,
            float(plane.Origin.Y) * 0.001,
            float(plane.Origin.Z) * 0.001
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
    }, source


def infer_package_name_from_urdf(urdf_path):
    try:
        text = open(urdf_path, "r", errors="ignore").read()
    except Exception:
        return "annin_ar4_description"

    marker = "package://"

    idx = text.find(marker)

    if idx < 0:
        return "annin_ar4_description"

    start = idx + len(marker)
    rest = text[start:]
    parts = rest.replace("\\", "/").split("/")

    if parts:
        return parts[0]

    return "annin_ar4_description"


# ------------------------------------------------------------
# Helpers: COM port / serial handshake
# ------------------------------------------------------------

def scan_ports():
    ports = []
    names = []

    try:
        import serial.tools.list_ports

        for p in serial.tools.list_ports.comports():
            label = "{} | {} | {}".format(p.device, p.description, p.hwid)
            ports.append(label)

            if p.device not in names:
                names.append(p.device)

    except Exception as e:
        ports.append("pyserial scan error: {}".format(e))

    # Fallback through .NET SerialPort if available.
    try:
        from System.IO.Ports import SerialPort

        for name in SerialPort.GetPortNames():
            label = "{} | .NET SerialPort".format(name)
            ports.append(label)

            if name not in names:
                names.append(name)

    except Exception:
        pass

    return ports, names


def choose_port(preferred_port, available_port_labels, available_port_names):
    preferred_port = preferred_port.strip()

    if preferred_port and preferred_port in available_port_names:
        return preferred_port

    # Prefer Teensy USB VID/PID used by the AR4 controller.
    for label in available_port_labels:
        if "VID:PID=16C0:0483" in label:
            return label.split("|")[0].strip()

    # Prefer USB serial devices.
    for label in available_port_labels:
        lower = label.lower()

        if "usb" in lower or "teensy" in lower:
            return label.split("|")[0].strip()

    if available_port_names:
        return available_port_names[0]

    return ""


def is_rp_line(line):
    return (
        isinstance(line, str)
        and line.startswith("A")
        and "B" in line
        and "C" in line
        and "P" in line
    )


def parse_live_rp(rp, track_length=870.0, track_inverted=False):
    """Minimal RP parse for Comp1 live-pose outputs."""
    result = {
        "parsed": False,
        "joint_values_deg": [],
        "j7_mm": 0.0,
        "track_m": 0.0,
        "raw": rp or "",
    }

    if not rp:
        return result

    def between(s, a, b=None):
        start = s.find(a)
        if start < 0:
            return ""
        start += len(a)
        if b is None:
            return s[start:].strip()
        end = s.find(b, start)
        if end < 0:
            return s[start:].strip()
        return s[start:end].strip()

    try:
        joints = [
            float(between(rp, "A", "B")),
            float(between(rp, "B", "C")),
            float(between(rp, "C", "D")),
            float(between(rp, "D", "E")),
            float(between(rp, "E", "F")),
            float(between(rp, "F", "G")),
        ]
        j7 = float(between(rp, "P", "Q"))
        length = track_length if track_length > 0.0 else 870.0
        track_m = max(0.0, min(1.0, j7 / length))
        if track_inverted:
            track_m = 1.0 - track_m
        result["parsed"] = True
        result["joint_values_deg"] = joints
        result["j7_mm"] = j7
        result["track_m"] = track_m
    except Exception:
        pass

    return result


class SerialBus(object):
    """
    Process-wide serial owner for one robot controller.

    Comp1 opens this once and leaves it in sticky AR4_SERIAL_BUS. Comp2 / Comp5 /
    Comp6 must send through write_command / wait_ack / send_and_wait and must
    never open the COM port themselves — a second open resets the Teensy and
    races the reader thread.
    """

    def __init__(self):
        self.lock = threading.RLock()
        self.ser = None
        self.port = ""
        self.baud = 0
        self.generation = 0
        self._stop = threading.Event()
        self._thread = None
        self._rx = ""
        self._ack_queue = []
        self._ack_event = threading.Event()
        self.latest_rp = ""
        self.latest_pose = parse_live_rp("")
        self.last_error = ""
        self.connected_at = 0.0

    def is_connected(self):
        with self.lock:
            return self.ser is not None and getattr(self.ser, "is_open", False)

    def connect(self, port, baud, settle_s=2.0):
        import serial

        port = str(port or "").strip()
        baud = int(baud)

        if not port:
            raise Exception("No COM port to connect.")

        with self.lock:
            already = (
                self.ser is not None
                and getattr(self.ser, "is_open", False)
                and self.port == port
                and self.baud == baud
            )
        if already:
            return False

        # Opening USB CDC resets the Teensy. Do it only when the port changes.
        self.disconnect()

        try:
            ser = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=0.05,
                write_timeout=30.0,
            )
        except Exception as e:
            # A previous Comp1 solve may have orphaned the COM handle after an
            # isinstance(SerialBus) sticky replace. Close any mirrored handle
            # and retry once.
            msg = str(e).lower()
            if "access is denied" in msg or "permissionerror" in msg:
                _force_release_serial_handle()
                try:
                    ser = serial.Serial(
                        port=port,
                        baudrate=baud,
                        timeout=0.05,
                        write_timeout=30.0,
                    )
                except Exception as e2:
                    raise Exception(
                        "could not open port '{}': {}. "
                        "COM is still held — unplug the Teensy USB or restart "
                        "Rhino once, then set Connect=True again.".format(port, e2)
                    )
            else:
                raise

        time.sleep(max(0.0, float(settle_s)))

        try:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
        except Exception:
            pass

        with self.lock:
            self.ser = ser
            self.port = port
            self.baud = baud
            self.generation += 1
            self.last_error = ""
            self.connected_at = time.time()
            self._rx = ""
            self._ack_queue = []
            self._ack_event.clear()
            self._stop.clear()
            gen = self.generation
            self._thread = threading.Thread(
                target=self._reader_loop,
                args=(gen,),
                name="AR4SerialBus-{}".format(port),
            )
            self._thread.daemon = True
            self._thread.start()

        if sc is not None:
            sc.sticky[SERIAL_HANDLE_KEY] = ser

        return True

    def disconnect(self):
        with self.lock:
            self._stop.set()
            self.generation += 1
            thread = self._thread
            ser = self.ser
            self._thread = None
            self.ser = None
            self.port = ""
            self.baud = 0
            self._ack_queue = []
            self._ack_event.set()

        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=1.0)

        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass

        if sc is not None:
            held = sc.sticky.get(SERIAL_HANDLE_KEY, None)
            if held is ser or held is not None:
                try:
                    if held is not ser and getattr(held, "is_open", False):
                        held.close()
                except Exception:
                    pass
                try:
                    sc.sticky.pop(SERIAL_HANDLE_KEY, None)
                except Exception:
                    pass

        self._ack_event.clear()

    def _reader_loop(self, generation):
        while not self._stop.is_set():
            with self.lock:
                if generation != self.generation or self.ser is None:
                    return
                ser = self.ser

            try:
                waiting = ser.in_waiting
            except Exception as e:
                self.last_error = str(e)
                time.sleep(0.02)
                continue

            if waiting and waiting > 0:
                try:
                    chunk = ser.read(waiting).decode("utf-8", errors="ignore")
                except Exception as e:
                    self.last_error = str(e)
                    time.sleep(0.02)
                    continue

                with self.lock:
                    if generation != self.generation:
                        return
                    self._rx += chunk
                    while "\n" in self._rx:
                        line, self._rx = self._rx.split("\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        self._handle_line_locked(line)
            else:
                time.sleep(0.005)

    def _handle_line_locked(self, line):
        track_length = 870.0
        track_inverted = False
        if sc is not None:
            try:
                track_length = float(sc.sticky.get("AR4_TRACK_LENGTH_MM", 870.0) or 870.0)
            except Exception:
                track_length = 870.0
            track_inverted = str(
                sc.sticky.get("AR4_TRACK_FIRMWARE_INVERTED", False)
            ).strip().lower() in ["true", "1", "yes", "y"]

        if is_rp_line(line):
            self.latest_rp = line
            self.latest_pose = parse_live_rp(line, track_length, track_inverted)

        # Every line is queued. wait_ack filters for RP/ER/EL unless
        # accept_any_line is set (handshake / TL).
        self._ack_queue.append(line)
        self._ack_event.set()

    def write_command(self, command):
        text = str(command).strip()
        if not text:
            raise Exception("Empty serial command.")

        with self.lock:
            if self.ser is None or not getattr(self.ser, "is_open", False):
                raise Exception(
                    "Serial bus is not connected. Set Comp1 Connect=True and leave it on."
                )
            self.ser.write((text + "\n").encode("utf-8"))
            self.ser.flush()

    def clear_acks(self):
        with self.lock:
            self._ack_queue = []
            self._ack_event.clear()

    def wait_ack(self, timeout, accept_any_line=False):
        """
        Wait for one terminal reply (RP / ER / EL), or any newline if
        accept_any_line is True (handshake / TL).

        timeout <= 0 polls the queue once and returns immediately
        (TIMEOUT_EMPTY if nothing is ready). Comp5 must use that so the
        Grasshopper solve thread never blocks for a whole robot move.
        """
        timeout = float(timeout)
        if timeout <= 0.0:
            deadline = time.time()
        else:
            deadline = time.time() + timeout
        other = []

        while True:
            with self.lock:
                while self._ack_queue:
                    line = self._ack_queue.pop(0)
                    if line == "ER":
                        return "ERROR_ER", line
                    if line.startswith("EL"):
                        return "ERROR_EL", line
                    if is_rp_line(line):
                        return "RP", line
                    if accept_any_line:
                        return "LINE", line
                    other.append(line)
                self._ack_event.clear()

            remaining = deadline - time.time()
            if remaining <= 0.0:
                if other:
                    return "TIMEOUT_WITH_DATA", " | ".join(other)
                return "TIMEOUT_EMPTY", ""

            self._ack_event.wait(min(0.05, remaining))

    def poll_ack(self, accept_any_line=False):
        """Non-blocking single check for a terminal reply."""
        return self.wait_ack(0.0, accept_any_line=accept_any_line)

    def send_and_wait(self, command, timeout, accept_any_line=False):
        self.clear_acks()
        self.write_command(command)
        return self.wait_ack(timeout, accept_any_line=accept_any_line)

    def snapshot_pose(self):
        with self.lock:
            pose = dict(self.latest_pose)
            pose["raw"] = self.latest_rp
            return pose


def _force_release_serial_handle():
    """Best-effort close of a leaked pyserial handle kept in sticky."""
    if sc is None:
        return
    held = sc.sticky.get(SERIAL_HANDLE_KEY, None)
    if held is not None:
        try:
            held.close()
        except Exception:
            pass
        try:
            sc.sticky.pop(SERIAL_HANDLE_KEY, None)
        except Exception:
            pass
    bus = sc.sticky.get(SERIAL_BUS_KEY, None)
    if bus is not None and callable(getattr(bus, "disconnect", None)):
        try:
            if callable(getattr(bus, "is_connected", None)) and bus.is_connected():
                bus.disconnect()
        except Exception:
            pass


def _bus_looks_usable(bus):
    """
    Duck-type check. GH re-executes this script every solve, which creates a
    new SerialBus class object each time — isinstance(sticky_bus, SerialBus)
    is therefore almost always False across solves. Using isinstance orphaned
    the open COM handle (Access Denied on the next connect).
    """
    return (
        bus is not None
        and callable(getattr(bus, "is_connected", None))
        and callable(getattr(bus, "connect", None))
        and callable(getattr(bus, "disconnect", None))
        and callable(getattr(bus, "send_and_wait", None))
        and callable(getattr(bus, "write_command", None))
        and callable(getattr(bus, "wait_ack", None))
        and callable(getattr(bus, "snapshot_pose", None))
    )


def get_or_create_bus():
    if sc is None:
        raise Exception("scriptcontext is not available.")
    bus = sc.sticky.get(SERIAL_BUS_KEY)
    if _bus_looks_usable(bus):
        return bus

    # Whatever is in sticky is unusable — release its port before replacing.
    if bus is not None and callable(getattr(bus, "disconnect", None)):
        try:
            bus.disconnect()
        except Exception:
            pass

    bus = SerialBus()
    sc.sticky[SERIAL_BUS_KEY] = bus
    return bus


def get_connected_bus():
    """Used by Comp2 / Comp5 / Comp6. Returns None if Comp1 is not connected."""
    if sc is None:
        return None
    bus = sc.sticky.get(SERIAL_BUS_KEY)
    if not _bus_looks_usable(bus):
        return None
    try:
        if bus.is_connected():
            return bus
    except Exception:
        return None
    return None


# ------------------------------------------------------------
# Defaults / outputs
# ------------------------------------------------------------

cache_id = None
robot_model = None
robot_name = None
joint_names = []
motion_joint_names = []
link_names = []
mesh_link_names = []
mesh_count = 0
available_ports = []
active_port = ""
connected = False
handshake_reply = []
live_joint_values_deg = []
live_j7_mm = 0.0
live_track_m = 0.0
live_rp_raw = ""
base_plane = None
tool_meshes = []
tool_tcp_plane = None
startup_state = {}
status = ""


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------

try:
    if sc is None:
        raise Exception("scriptcontext is not available.")

    load = to_bool(Load, True)
    reload = to_bool(Reload, False)

    urdf_path = safe_path(URDF)
    package_root = safe_path(PackageRoot)

    preferred_port = to_text(PreferredPort, "COM4")
    baud = int(to_float(Baud, 9600))
    test_command = to_text(TestCommand, "HO")

    connect = to_bool(Connect, False)

    # Optional BasePlane input. World XY (URDF world at Rhino origin) if absent,
    # which keeps existing single-robot definitions behaving exactly as before.
    base_plane = to_plane(get_optional_input("BasePlane", None), rg.Plane.WorldXY)

    if not urdf_path:
        raise Exception("No URDF path provided.")

    if not package_root:
        raise Exception("No PackageRoot path provided.")

    cache_id = make_cache_id(urdf_path)

    # ------------------------------------------------------------
    # 1. Scan serial ports
    # ------------------------------------------------------------

    available_ports, available_port_names = scan_ports()
    active_port = choose_port(preferred_port, available_ports, available_port_names)

    # ------------------------------------------------------------
    # 2. Load / cache robot model and meshes
    # ------------------------------------------------------------

    model_key = cache_id + "::model"
    mesh_key = cache_id + "::meshes"
    state_key = cache_id + "::startup_state"
    report_key = cache_id + "::mesh_load_report"

    cached_model = sc.sticky.get(model_key, None)

    if load and (reload or cached_model is None):

        if not os.path.exists(urdf_path):
            raise Exception("URDF file does not exist: {}".format(urdf_path))

        from compas_robots import RobotModel

        model = RobotModel.from_urdf_file(urdf_path)

        # Try to load COMPAS geometry as well, but do not depend on it.
        try:
            from compas_robots.resources import LocalPackageMeshLoader

            package_name = infer_package_name_from_urdf(urdf_path)
            loader = LocalPackageMeshLoader(package_root, package_name)
            model.load_geometry(loader)

        except Exception:
            pass

        mesh_cache, mesh_report = parse_mesh_cache_from_urdf(urdf_path, package_root)

        sc.sticky[model_key] = model
        sc.sticky[mesh_key] = mesh_cache
        sc.sticky[report_key] = mesh_report

    elif cached_model is not None:
        model = cached_model
        mesh_cache = sc.sticky.get(mesh_key, {})
        mesh_report = sc.sticky.get(report_key, [])

    else:
        model = None
        mesh_cache = {}
        mesh_report = []

    if model is not None:
        robot_model = model
        robot_name = model.name

        try:
            joint_names = model.get_configurable_joint_names()
        except Exception:
            joint_names = []

        try:
            link_names = [link.name for link in model.links]
        except Exception:
            link_names = []

    else:
        robot_model = None
        robot_name = None
        joint_names = []
        link_names = []

    # Motion joints used by the AR4 + track.
    # Gripper joint, if present, is intentionally excluded.
    motion_joint_names = [
        "track_joint",
        "joint_1",
        "joint_2",
        "joint_3",
        "joint_4",
        "joint_5",
        "joint_6"
    ]

    mesh_link_names = sorted(list(mesh_cache.keys()))
    mesh_count = 0

    for meshes in mesh_cache.values():
        try:
            mesh_count += len(meshes)
        except Exception:
            pass

    # ------------------------------------------------------------
    # 2b. Load / cache the tool
    # ------------------------------------------------------------

    tool_meshes = to_mesh_list(get_optional_input("ToolMeshes", None))

    tool_tcp_frame, tool_tcp_source = build_tool_tcp_frame(
        get_optional_input("ToolTCPPlane", None),
        get_optional_input("ToolTCPPoint", None)
    )

    if tool_tcp_frame is not None:
        tool_tcp_plane = rg.Plane(
            rg.Point3d(
                tool_tcp_frame["origin_m"][0] * 1000.0,
                tool_tcp_frame["origin_m"][1] * 1000.0,
                tool_tcp_frame["origin_m"][2] * 1000.0
            ),
            rg.Vector3d(*tool_tcp_frame["xaxis"]),
            rg.Vector3d(*tool_tcp_frame["yaxis"])
        )
    else:
        tool_tcp_plane = None

    # Comp4 and Comp6 fall back to these; their own pins still win, which is
    # what lets two robots sharing this cache_id carry different tools.
    sc.sticky["AR4_TOOL_MESHES"] = list(tool_meshes)
    sc.sticky["AR4_TOOL_TCP_FRAME"] = tool_tcp_frame
    sc.sticky["AR4_TOOL_SOURCE"] = tool_tcp_source

    # ------------------------------------------------------------
    # 3. Own the serial port for the whole session
    # ------------------------------------------------------------
    # Leave Connect=True. Comp2 / Comp5 / Comp6 send through sticky
    # AR4_SERIAL_BUS and must not open the COM port themselves.

    bus = get_or_create_bus()
    handshake_reply = []
    handshake_raw = ""

    if connect:
        if not active_port:
            serial_status = "No active COM port found."
            connected = False
        else:
            try:
                opened = bus.connect(active_port, baud, settle_s=2.0)
                connected = bus.is_connected()

                if test_command:
                    # Only handshake when the port was just opened, so a
                    # sustained Connect=True does not spam HO every solve.
                    if opened:
                        status_code, line = bus.send_and_wait(
                            test_command,
                            timeout=3.0,
                            accept_any_line=True
                        )
                        if line:
                            handshake_reply = [line]
                            handshake_raw = line
                        serial_status = (
                            "Port open on {}. Handshake {} -> {}."
                        ).format(active_port, test_command, status_code)
                    else:
                        pose = bus.snapshot_pose()
                        serial_status = (
                            "Holding open connection on {} @ {}."
                        ).format(bus.port, bus.baud)
                        if pose.get("parsed"):
                            serial_status += " Latest RP available."
                else:
                    serial_status = "Port open on {} @ {}.".format(
                        active_port,
                        baud
                    )

            except Exception as e:
                connected = False
                serial_status = "Serial connection error: {}".format(e)
    else:
        if bus.is_connected():
            bus.disconnect()
        connected = False
        serial_status = "Connect=False. Serial bus disconnected."

    pose = bus.snapshot_pose() if bus.is_connected() else parse_live_rp("")
    live_joint_values_deg = list(pose.get("joint_values_deg") or [])
    live_j7_mm = float(pose.get("j7_mm") or 0.0)
    live_track_m = float(pose.get("track_m") or 0.0)
    live_rp_raw = str(pose.get("raw") or "")
    connected = bool(bus.is_connected())
    if connected:
        active_port = bus.port or active_port

    # ------------------------------------------------------------
    # 4. Store startup state
    # ------------------------------------------------------------

    startup_state = {
        "cache_id": cache_id,
        "urdf": urdf_path,
        "package_root": package_root,
        "port": active_port,
        "baud": baud,
        "connected": connected,
        "serial_bus_key": SERIAL_BUS_KEY,
        "test_command": test_command,
        "base_plane": base_plane,
        "track_length_mm": 870.0,
        "track_firmware_inverted": False,
        "track_home_is_motor_side": True,
        "firmware_j7_home_mm": 0.0,
        "logical_track_m_motor_side": 0.0,
        "logical_track_m_far_side": 1.0,
        "firmware_j7_formula": "J7 = TrackM * TrackLengthMM  (0 at motor/limit after CalDir=0)",
        "firmware_j7_note": "If RP shows high J7 at motor, set AR4_TRACK_FIRMWARE_INVERTED=True until Teensy J7 CalDir=0 recalibration.",
        "use_calibration_frame_default": False,
        "encode_firmware_on_send_default": True,
        "home_urdf_deg_default": [-16.0, 0.0, 0.0, 0.0, 22.0, -10.0],
        "joint_signs_default": [-1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
        "target_units_mm": True,
        "motion_joint_names": motion_joint_names,
        "tool_mesh_count": len(tool_meshes),
        "tool_tcp_frame": tool_tcp_frame,
        "tool_tcp_source": tool_tcp_source,
        "mesh_load_report": mesh_report
    }

    sc.sticky[state_key] = startup_state
    # Per-cache base plane. Comp3/Comp4 prefer their own BasePlane pin; this is
    # the single-robot fallback. Wire the pins when loading several robots,
    # because robots sharing a URDF also share this cache_id.
    sc.sticky[cache_id + "::base_plane"] = base_plane
    sc.sticky["AR4_TRACK_LENGTH_MM"] = 870.0
    sc.sticky["AR4_TRACK_FIRMWARE_INVERTED"] = False
    # Program/sim RJ stay in URDF space; Comp5 encodes with this home.
    sc.sticky["AR4_USE_CALIBRATION_FRAME"] = False
    sc.sticky["AR4_ENCODE_FIRMWARE_ON_SEND"] = True
    # firmware = (urdf - home) * sign. J1 sign is negative: URDF joint_1 turns
    # opposite to firmware A (joint_1 carries rpy="3.1416 0 0", inverting its Z).
    # Measured: URDF joint_1 -36 == firmware A +20, both reaching toward +X.
    sc.sticky["AR4_HOME_URDF_DEG"] = [-16.0, 0.0, 0.0, 0.0, 22.0, -10.0]
    sc.sticky["AR4_JOINT_SIGNS"] = [-1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    sc.sticky["AR4_SIM_USE_CALIBRATION_FRAME"] = False

    # ------------------------------------------------------------
    # 5. Status
    # ------------------------------------------------------------

    report_text = ""

    if mesh_report:
        report_text = " Mesh warnings: {}.".format(len(mesh_report))

    if model is not None:
        model_status = "Loaded robot '{}' with {} configurable joint(s), {} link(s), and {} cached mesh object(s).".format(
            robot_name,
            len(joint_names),
            len(link_names),
            mesh_count
        )
    else:
        model_status = "Robot model not loaded. Set Load=True."

    if base_plane is not None:
        base_text = " Base plane origin ({:.1f}, {:.1f}, {:.1f}), X axis ({:.2f}, {:.2f}, {:.2f}).".format(
            base_plane.Origin.X,
            base_plane.Origin.Y,
            base_plane.Origin.Z,
            base_plane.XAxis.X,
            base_plane.XAxis.Y,
            base_plane.XAxis.Z
        )
    else:
        base_text = " Base plane unset."

    if tool_meshes or tool_tcp_frame is not None:
        tool_text = " Tool: {} mesh object(s), TCP {}.".format(
            len(tool_meshes),
            tool_tcp_source if tool_tcp_source else "not set"
        )
    else:
        tool_text = " No tool loaded."

    status = "{} Port: {} (connected={}).{}{} {}{}".format(
        model_status,
        active_port if active_port else "None",
        connected,
        base_text,
        tool_text,
        serial_status,
        report_text
    )

except Exception as e:
    cache_id = None
    robot_model = None
    robot_name = None
    joint_names = []
    motion_joint_names = []
    link_names = []
    mesh_link_names = []
    mesh_count = 0
    available_ports = []
    active_port = ""
    connected = False
    handshake_reply = []
    live_joint_values_deg = []
    live_j7_mm = 0.0
    live_track_m = 0.0
    live_rp_raw = ""
    base_plane = None
    tool_meshes = []
    tool_tcp_plane = None
    startup_state = {}
    status = "ERROR: {}".format(e)