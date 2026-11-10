#! python3

# =============================================================================
# GH_PINS — Rhino does not create pins from pasted code. Copy these names into
# the Python Script component UI (Type Hint / Access in COMPONENT_PINS.txt).
# INPUTS:
#   Captures
#   Solve
#   Apply
#   Clear
#   MaxAxisTiltDeg
# OUTPUTS:
#   tcp_frame
#   tcp_plane_local
#   tcp_offset_mm
#   fit_max_mm
#   fit_rms_mm
#   calibration_point
#   warnings
#   status
# =============================================================================

"""
AR4 + External Track — Component 7: TCP Calibration
Grasshopper Python 3 / Rhino 8

Turns the poses captured by Comp6 into a flange-local TCP frame, then hands
it to Comp3 and Comp4 through sticky AR4_TCP_FRAME.

This is the usual 4-plus-2 routine:

  Stage 1, position. Touch one fixed physical point with four or more
  different arm postures. Every capture satisfies

      p_i + R_i * t = P

  where p_i and R_i are the flange pose, t is the unknown tool tip in flange
  coordinates and P is the unknown fixed point. Four captures give twelve
  equations for six unknowns, so t and P fall out of a least squares solve
  and the residual tells you how good the touch-offs were.

  Stage 2, orientation. Four points pin down where the tip is but say nothing
  about how the tool is turned, and without that the IK can only align the
  bare flange to a target plane. So jog the tip to a point along the tool's
  intended +X, capture it, then along +Z and capture that. Those two tip
  positions give the tool axes.

  Stage 2 requires translation only: the flange must hold the reference
  orientation while you move to the axis points. The component measures how
  far the orientation drifted and refuses to use an axis pose past
  MaxAxisTiltDeg.


CAPTURE LABELS
--------------
Set CaptureLabel in Comp6 before each capture. Matching is case insensitive
and by prefix.

  tip...    one of the four-or-more touch poses        (needs 4)
  axisx...  tip sits along the tool's intended +X      (optional)
  axisz...  tip sits along the tool's intended +Z      (optional)
  ref...    orientation reference for stage 2          (optional)

Without a "ref" capture the first tip pose is the reference. With no axis
captures at all the component solves position only and leaves the tool axes
parallel to the flange.


INPUTS
------
Captures         : capture list from Comp6. Falls back to sticky AR4_JOG_CAPTURES.
Solve            : bool. Recompute the fit.
Apply            : bool edge trigger. Write the result to sticky AR4_TCP_FRAME
                   so Comp3 solves and Comp4 draws against it.
Clear            : bool edge trigger. Remove the stored calibration.
MaxAxisTiltDeg   : default 1.0. Orientation drift allowed on an axis pose.


OUTPUTS
-------
tcp_frame        : the calibration dict handed to Comp3 / Comp4
tcp_plane_local  : the TCP frame as a plane in flange-local Rhino units
tcp_offset_mm    : tool tip offset from the flange, Rhino units
fit_max_mm       : worst touch-off residual
fit_rms_mm       : rms touch-off residual
calibration_point: the fixed point the touch poses agreed on, robot-local
warnings
status
"""

import math

import Rhino.Geometry as rg

try:
    import scriptcontext as sc
except Exception:
    sc = None


CAPTURE_STICKY_KEY = "AR4_JOG_CAPTURES"
TCP_STICKY_KEY = "AR4_TCP_FRAME"

MM_PER_METER = 1000.0


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


def edge_triggered(key, value):
    if sc is None:
        return False

    previous = bool(sc.sticky.get(key, False))
    sc.sticky[key] = bool(value)

    return bool(value) and not previous


# ------------------------------------------------------------
# Small dense linear algebra
# ------------------------------------------------------------

def solve_linear_system(A, b):
    """Gaussian elimination with partial pivoting. Returns None if singular."""
    n = len(A)
    m = [list(A[i]) + [b[i]] for i in range(n)]

    for col in range(n):
        pivot = col

        for row in range(col + 1, n):
            if abs(m[row][col]) > abs(m[pivot][col]):
                pivot = row

        if abs(m[pivot][col]) < 1e-14:
            return None

        if pivot != col:
            m[col], m[pivot] = m[pivot], m[col]

        pv = m[col][col]

        for row in range(n):
            if row == col:
                continue

            factor = m[row][col] / pv

            if factor == 0.0:
                continue

            for k in range(col, n + 1):
                m[row][k] -= factor * m[col][k]

    return [m[i][n] / m[i][i] for i in range(n)]


def normal_equations(A, b):
    """(A^T A, A^T b) for an overdetermined system."""
    rows = len(A)
    cols = len(A[0])

    ata = [[0.0] * cols for _ in range(cols)]
    atb = [0.0] * cols

    for i in range(rows):
        row = A[i]

        for j in range(cols):
            rij = row[j]

            if rij == 0.0:
                continue

            atb[j] += rij * b[i]

            for k in range(cols):
                ata[j][k] += rij * row[k]

    return ata, atb


# ------------------------------------------------------------
# Captures
# ------------------------------------------------------------

def load_captures():
    raw = gh_input("Captures", None)

    items = None

    if raw is not None:
        try:
            items = [unwrap_gh(v) for v in list(raw)]
        except TypeError:
            items = [unwrap_gh(raw)]

    if not items and sc is not None:
        stored = sc.sticky.get(CAPTURE_STICKY_KEY, None)

        if isinstance(stored, list):
            items = list(stored)

    return [c for c in (items or []) if isinstance(c, dict)]


def capture_rotation(capture):
    """Flange rotation as a 3x3 with the axes in the columns."""
    x = capture.get("xaxis", [1.0, 0.0, 0.0])
    y = capture.get("yaxis", [0.0, 1.0, 0.0])
    z = capture.get("zaxis", None)

    if z is None:
        vx = rg.Vector3d(x[0], x[1], x[2])
        vy = rg.Vector3d(y[0], y[1], y[2])
        vz = rg.Vector3d.CrossProduct(vx, vy)
        z = [vz.X, vz.Y, vz.Z]

    return [
        [float(x[0]), float(y[0]), float(z[0])],
        [float(x[1]), float(y[1]), float(z[1])],
        [float(x[2]), float(y[2]), float(z[2])]
    ]


def capture_origin(capture):
    o = capture.get("origin_m", [0.0, 0.0, 0.0])

    return [float(o[0]), float(o[1]), float(o[2])]


def mat3_mul_vec(m, v):
    return [
        m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
        m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
        m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2]
    ]


def mat3_transpose_mul(a, b):
    """a^T * b for two 3x3 matrices."""
    return [
        [sum(a[k][i] * b[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]


def rotation_angle_between(a, b):
    """Angle in radians between two rotation matrices."""
    r = mat3_transpose_mul(a, b)
    trace = r[0][0] + r[1][1] + r[2][2]

    c = (trace - 1.0) * 0.5
    c = max(-1.0, min(1.0, c))

    return math.acos(c)


def label_of(capture):
    return str(capture.get("label", "")).strip().lower()


def pick_by_prefix(captures, prefix):
    return [c for c in captures if label_of(c).startswith(prefix)]


# ------------------------------------------------------------
# Stage 1: TCP position
# ------------------------------------------------------------

def solve_tcp_position(touch_captures):
    """
    Least squares over p_i + R_i * t = P.

    Unknown vector is [tx, ty, tz, Px, Py, Pz]; each capture contributes the
    three rows [R_i | -I] = -p_i. Returns (t, P, max_residual_m, rms_m).
    """
    rows = []
    rhs = []

    for capture in touch_captures:
        r = capture_rotation(capture)
        p = capture_origin(capture)

        for axis in range(3):
            row = [r[axis][0], r[axis][1], r[axis][2], 0.0, 0.0, 0.0]
            row[3 + axis] = -1.0

            rows.append(row)
            rhs.append(-p[axis])

    ata, atb = normal_equations(rows, rhs)
    solution = solve_linear_system(ata, atb)

    if solution is None:
        return None, None, None, None

    t = solution[0:3]
    point = solution[3:6]

    worst = 0.0
    total = 0.0

    for capture in touch_captures:
        r = capture_rotation(capture)
        p = capture_origin(capture)
        tip = mat3_mul_vec(r, t)

        d = math.sqrt(sum(
            (p[a] + tip[a] - point[a]) ** 2
            for a in range(3)
        ))

        worst = max(worst, d)
        total += d * d

    rms = math.sqrt(total / len(touch_captures))

    return t, point, worst, rms


def tip_position(capture, t):
    """Where the tool tip sat for this capture, robot-local meters."""
    r = capture_rotation(capture)
    p = capture_origin(capture)
    offset = mat3_mul_vec(r, t)

    return [p[a] + offset[a] for a in range(3)]


# ------------------------------------------------------------
# Stage 2: TCP orientation
# ------------------------------------------------------------

def solve_tcp_orientation(reference, axis_x_capture, axis_z_capture, t, point):
    """
    Tool axes from two tip positions measured off the calibration point.

    Returns (3x3 flange-local rotation, notes). Falls back to identity, i.e.
    tool axes parallel to the flange, when neither axis pose is supplied.
    """
    notes = []

    x_dir = None
    z_dir = None

    if axis_x_capture is not None:
        q = tip_position(axis_x_capture, t)
        x_dir = rg.Vector3d(q[0] - point[0], q[1] - point[1], q[2] - point[2])

        if x_dir.Length < 1e-5:
            notes.append("axisX pose sits on the calibration point; ignored.")
            x_dir = None

    if axis_z_capture is not None:
        q = tip_position(axis_z_capture, t)
        z_dir = rg.Vector3d(q[0] - point[0], q[1] - point[1], q[2] - point[2])

        if z_dir.Length < 1e-5:
            notes.append("axisZ pose sits on the calibration point; ignored.")
            z_dir = None

    if x_dir is None and z_dir is None:
        notes.append(
            "No usable axis poses. Tool axes left parallel to the flange, so "
            "target plane orientation still resolves against the flange."
        )
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], notes

    # Fill in whichever axis is missing with something perpendicular.
    if x_dir is None:
        z_dir.Unitize()
        seed = rg.Vector3d.XAxis

        if abs(z_dir * seed) > 0.9:
            seed = rg.Vector3d.YAxis

        x_dir = rg.Vector3d.CrossProduct(seed, z_dir)
        notes.append("No axisX pose. Tool X chosen arbitrarily perpendicular to Z.")

    if z_dir is None:
        x_dir.Unitize()
        seed = rg.Vector3d.ZAxis

        if abs(x_dir * seed) > 0.9:
            seed = rg.Vector3d.YAxis

        z_dir = rg.Vector3d.CrossProduct(x_dir, seed)
        notes.append("No axisZ pose. Tool Z chosen arbitrarily perpendicular to X.")

    x_dir.Unitize()
    z_dir.Unitize()

    # X is primary, Z only fixes the roll, so Z is re-squared against X.
    y = rg.Vector3d.CrossProduct(z_dir, x_dir)

    if y.Length < 1e-6:
        notes.append("axisX and axisZ are parallel; cannot build a tool frame.")
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], notes

    y.Unitize()
    z = rg.Vector3d.CrossProduct(x_dir, y)
    z.Unitize()

    skew_deg = math.degrees(abs(math.pi * 0.5 - math.acos(
        max(-1.0, min(1.0, x_dir * z_dir))
    )))

    if skew_deg > 2.0:
        notes.append(
            "axisX and axisZ are {:.2f} deg off perpendicular; Z was squared "
            "against X.".format(skew_deg)
        )

    # Tool axes in robot-local space at the reference orientation.
    tool_world = [
        [x_dir.X, y.X, z.X],
        [x_dir.Y, y.Y, z.Y],
        [x_dir.Z, y.Z, z.Z]
    ]

    # Back into flange coordinates: R_local = R_ref^T * R_toolworld
    return mat3_transpose_mul(capture_rotation(reference), tool_world), notes


# ------------------------------------------------------------
# Outputs / defaults
# ------------------------------------------------------------

tcp_frame = {}
tcp_plane_local = rg.Plane.Unset
tcp_offset_mm = []
fit_max_mm = 0.0
fit_rms_mm = 0.0
calibration_point = rg.Point3d.Unset
warnings = []
status = ""


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------

try:
    if sc is None:
        raise Exception("scriptcontext is not available.")

    if edge_triggered("AR4_TCP_CAL_CLEAR_EDGE", to_bool(gh_input("Clear", False), False)):
        sc.sticky.pop(TCP_STICKY_KEY, None)
        status = (
            "Stored calibration cleared. Comp3 and Comp4 fall back to their "
            "TCPPlane / TCPPoint / ToolZ pins."
        )

    elif not to_bool(gh_input("Solve", False), False):
        stored = sc.sticky.get(TCP_STICKY_KEY, None)
        captures = load_captures()

        if isinstance(stored, dict):
            tcp_frame = dict(stored)
            o = stored.get("origin_m", [0.0, 0.0, 0.0])
            tcp_offset_mm = [v * MM_PER_METER for v in o]
            fit_max_mm = to_float(stored.get("fit_max_mm"), 0.0)
            fit_rms_mm = to_float(stored.get("fit_rms_mm"), 0.0)

        status = (
            "Solve=False. {} capture(s) available. Stored calibration: {}."
        ).format(
            len(captures),
            stored.get("source", "?") if isinstance(stored, dict) else "none"
        )

    else:
        captures = load_captures()

        touch = pick_by_prefix(captures, "tip")

        # Be forgiving: if nothing is labelled, treat every non-axis capture
        # as a touch pose rather than failing on a naming slip.
        if not touch:
            touch = [
                c for c in captures
                if not label_of(c).startswith("axis") and not label_of(c).startswith("ref")
            ]
            if touch:
                warnings.append(
                    "No captures labelled 'tip'; used {} unlabelled capture(s) "
                    "as touch poses.".format(len(touch))
                )

        if len(touch) < 4:
            raise Exception(
                "Need at least 4 touch poses, found {}. In Comp6 set "
                "CaptureLabel to tip1..tip4 and touch the same physical point "
                "with four clearly different arm postures.".format(len(touch))
            )

        t, point, worst_m, rms_m = solve_tcp_position(touch)

        if t is None:
            raise Exception(
                "Touch poses are degenerate — the fit is singular. Use four "
                "postures with genuinely different wrist orientations, not "
                "four nearby poses."
            )

        fit_max_mm = worst_m * MM_PER_METER
        fit_rms_mm = rms_m * MM_PER_METER

        max_tilt_deg = to_float(gh_input("MaxAxisTiltDeg", 1.0), 1.0)

        ref_list = pick_by_prefix(captures, "ref")
        reference = ref_list[0] if ref_list else touch[0]

        if not ref_list:
            warnings.append(
                "No 'ref' capture; used touch pose '{}' as the orientation "
                "reference.".format(reference.get("label", "?"))
            )

        ref_rotation = capture_rotation(reference)

        def usable_axis_pose(prefix):
            """Reject an axis pose whose flange turned away from the reference."""
            found = pick_by_prefix(captures, prefix)

            if not found:
                return None

            capture = found[0]
            tilt = math.degrees(
                rotation_angle_between(ref_rotation, capture_rotation(capture))
            )

            if tilt > max_tilt_deg:
                warnings.append(
                    "'{}' is {:.2f} deg off the reference orientation (limit "
                    "{:.2f}); ignored. Translate only when moving to the axis "
                    "points.".format(capture.get("label", prefix), tilt, max_tilt_deg)
                )
                return None

            return capture

        axis_x = usable_axis_pose("axisx")
        axis_z = usable_axis_pose("axisz")

        rotation, notes = solve_tcp_orientation(reference, axis_x, axis_z, t, point)
        warnings.extend(notes)

        tcp_frame = {
            "origin_m": [float(v) for v in t],
            "xaxis": [rotation[0][0], rotation[1][0], rotation[2][0]],
            "yaxis": [rotation[0][1], rotation[1][1], rotation[2][1]],
            "source": "Comp7 calibration ({} touch, X={}, Z={})".format(
                len(touch),
                "yes" if axis_x is not None else "no",
                "yes" if axis_z is not None else "no"
            ),
            "fit_max_mm": fit_max_mm,
            "fit_rms_mm": fit_rms_mm,
            "touch_pose_count": len(touch)
        }

        tcp_offset_mm = [v * MM_PER_METER for v in t]

        tcp_plane_local = rg.Plane(
            rg.Point3d(
                tcp_offset_mm[0],
                tcp_offset_mm[1],
                tcp_offset_mm[2]
            ),
            rg.Vector3d(rotation[0][0], rotation[1][0], rotation[2][0]),
            rg.Vector3d(rotation[0][1], rotation[1][1], rotation[2][1])
        )

        calibration_point = rg.Point3d(
            point[0] * MM_PER_METER,
            point[1] * MM_PER_METER,
            point[2] * MM_PER_METER
        )

        if edge_triggered("AR4_TCP_CAL_APPLY_EDGE", to_bool(gh_input("Apply", False), False)):
            sc.sticky[TCP_STICKY_KEY] = dict(tcp_frame)
            apply_note = "APPLIED to sticky; Comp3 and Comp4 will use it"
        else:
            apply_note = "not applied — toggle Apply False then True to store"

        status = (
            "TCP from {} touch pose(s). Offset [{:.3f}, {:.3f}, {:.3f}] mm. "
            "Fit max {:.3f} mm, rms {:.3f} mm. Orientation: X={} Z={}. {}."
        ).format(
            len(touch),
            tcp_offset_mm[0],
            tcp_offset_mm[1],
            tcp_offset_mm[2],
            fit_max_mm,
            fit_rms_mm,
            "measured" if axis_x is not None else "assumed",
            "measured" if axis_z is not None else "assumed",
            apply_note
        )

        if fit_max_mm > 2.0:
            warnings.append(
                "Worst residual is {:.2f} mm. Either the touch-offs missed the "
                "point or the joint signs / home offsets are off.".format(fit_max_mm)
            )

except Exception as e:
    tcp_frame = {}
    tcp_plane_local = rg.Plane.Unset
    tcp_offset_mm = []
    fit_max_mm = 0.0
    fit_rms_mm = 0.0
    calibration_point = rg.Point3d.Unset
    status = "Comp7 TCP calibration ERROR: {}".format(e)
