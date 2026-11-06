#! python3
# r: pyserial

import time
import math

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


def to_int_flag(value):
    return 1 if to_bool(value, False) else 0


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


# ------------------------------------------------------------
# Helpers: command construction
# ------------------------------------------------------------

def normalize_mode(value):
    mode = to_text(value, "RP").strip().upper()

    if mode in ["TEST", "TEST_LIMITS", "LIMIT", "LIMITS"]:
        return "TL"

    if mode in ["CAL", "CALIBRATE", "CALIBRATION", "HOME", "HOMING"]:
        return "LL"

    if mode in ["READ", "POSITION", "READ_POSITION"]:
        return "RP"

    if mode not in ["TL", "LL", "RP"]:
        return "RP"

    return mode


def build_ll_command(flags):
    """
    Firmware LL format:

    LL
    A-I = selected joint flags J1-J9
    J-R = calibration offsets J1-J9

    This simplified component uses zero offsets.
    """

    j1, j2, j3, j4, j5, j6, j7, j8, j9 = flags

    return (
        "LL"
        "A{j1}"
        "B{j2}"
        "C{j3}"
        "D{j4}"
        "E{j5}"
        "F{j6}"
        "G{j7}"
        "H{j8}"
        "I{j9}"
        "J0.000"
        "K0.000"
        "L0.000"
        "M0.000"
        "N0.000"
        "O0.000"
        "P0.000"
        "Q0.000"
        "R0.000"
    ).format(
        j1=j1,
        j2=j2,
        j3=j3,
        j4=j4,
        j5=j5,
        j6=j6,
        j7=j7,
        j8=j8,
        j9=j9
    )


def selected_joint_names(flags):
    names = []

    for i, flag in enumerate(flags):
        if flag == 1:
            names.append("J{}".format(i + 1))

    return names


# ------------------------------------------------------------
# Helpers: RP parsing
# ------------------------------------------------------------

def clean_rp_reply(value):
    if value is None:
        return ""

    s = str(value).strip()

    for line in s.splitlines():
        line = line.strip()

        # RP position strings begin with A and include external axis P.
        if line.startswith("A") and "B" in line and "C" in line and "P" in line:
            return line

    return ""


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


def parse_rp(rp):
    result = {
        "joint_values_deg": [],
        "joint_values_rad": [],
        "cartesian_pose": [],
        "external_axes_mm": [],
        "j7_mm": 0.0,
        "track_m": 0.0,
        "speed_violation": "",
        "debug": "",
        "flag": "",
        "parsed": False
    }

    if not rp:
        return result

    try:
        # A-F = robot joint angles in degrees.
        A = to_float(between(rp, "A", "B"))
        B = to_float(between(rp, "B", "C"))
        C = to_float(between(rp, "C", "D"))
        D = to_float(between(rp, "D", "E"))
        E = to_float(between(rp, "E", "F"))
        F = to_float(between(rp, "F", "G"))

        joint_values_deg = [A, B, C, D, E, F]
        joint_values_rad = [math.radians(v) for v in joint_values_deg]

        # G-L = firmware Cartesian pose.
        X = to_float(between(rp, "G", "H"))
        Y = to_float(between(rp, "H", "I"))
        Z = to_float(between(rp, "I", "J"))

        Rz = to_float(between(rp, "J", "K"))
        Ry = to_float(between(rp, "K", "L"))
        Rx = to_float(between(rp, "L", "M"))

        cartesian_pose = [X, Y, Z, Rz, Ry, Rx]

        # M-O = diagnostics.
        speed_violation = between(rp, "M", "N")
        debug = between(rp, "N", "O")
        flag = between(rp, "O", "P")

        # P-R = external axes.
        P = to_float(between(rp, "P", "Q"))
        Q = to_float(between(rp, "Q", "R"))
        R = to_float(between(rp, "R", None))

        external_axes_mm = [P, Q, R]

        # J7 track mapping (shared convention after Teensy J7 CalDir=0):
        #
        # Firmware:
        #   J7 = 0     -> motor / home-switch side
        #   J7 = max   -> far end
        #
        # URDF TrackM:
        #   TrackM = J7 / TrackLength   (TrackFirmwareInverted=False)
        # If RP still shows high J7 at the motor, sticky AR4_TRACK_FIRMWARE_INVERTED=True
        # uses TrackM = 1 - J7/TrackLength.
        track_length = 870.0
        track_inverted = False
        if sc is not None:
            track_length = to_float(sc.sticky.get("AR4_TRACK_LENGTH_MM", 870.0), 870.0)
            track_inverted = to_bool(sc.sticky.get("AR4_TRACK_FIRMWARE_INVERTED", False), False)
        if track_length <= 0.0:
            track_length = 870.0

        j7_mm_value = P
        track_m_value = clamp(j7_mm_value / track_length, 0.0, 1.0)
        if track_inverted:
            track_m_value = 1.0 - track_m_value

        result["joint_values_deg"] = joint_values_deg
        result["joint_values_rad"] = joint_values_rad
        result["cartesian_pose"] = cartesian_pose
        result["external_axes_mm"] = external_axes_mm
        result["j7_mm"] = j7_mm_value
        result["track_m"] = track_m_value
        result["track_length_mm"] = track_length
        result["track_firmware_inverted"] = track_inverted
        result["speed_violation"] = speed_violation
        result["debug"] = debug
        result["flag"] = flag
        result["parsed"] = True

        return result

    except Exception:
        return result


# ------------------------------------------------------------
# Helpers: TL parsing
# ------------------------------------------------------------

def clean_tl_reply(value):
    if value is None:
        return ""

    s = str(value).strip()

    for line in s.splitlines():
        line = line.strip()

        # Original firmware returns "J1 = 0   J2 = 0..."
        # Updated firmware may return "J1=0 J2=0 ..."
        if "J1" in line and "J2" in line:
            return line

    return ""


def parse_tl(tl):
    result = {
        "raw": tl,
        "switches": {},
        "parsed": False
    }

    if not tl:
        return result

    normalized = (
        tl.replace("=", " ")
          .replace(",", " ")
          .replace(":", " ")
          .replace("\t", " ")
    )

    parts = normalized.split()

    switches = {}

    for i, part in enumerate(parts):
        p = part.strip().upper()

        if p.startswith("J") and len(p) >= 2:
            try:
                joint_number = int(p[1:])
            except Exception:
                continue

            if i + 1 < len(parts):
                value = parts[i + 1].strip()

                if value in ["0", "1"]:
                    switches["J{}".format(joint_number)] = int(value)

    if switches:
        result["switches"] = switches
        result["parsed"] = True

    return result


# ------------------------------------------------------------
# Helpers: serial
# ------------------------------------------------------------

def send_serial_command(port, baud, command, timeout, ignore_timeout=False):
    import serial

    ser = serial.Serial(
        port=port,
        baudrate=baud,
        timeout=0.1,
        write_timeout=2.0
    )

    # Teensy serial settle.
    time.sleep(2.0)

    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except Exception:
        pass

    wire_command = command.strip() + "\n"

    ser.write(wire_command.encode("utf-8"))
    ser.flush()

    start = time.time()
    buffer = ""

    try:
        while True:
            if (not ignore_timeout) and (time.time() - start >= timeout):
                break

            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                text = data.decode("utf-8", errors="ignore")
                buffer += text

                # TL and RP should return one line.
                if command.startswith("TL") or command.startswith("RP"):
                    if "\n" in buffer:
                        break

                # LL can take longer and should eventually return an RP-style line.
                if command.startswith("LL"):
                    if clean_rp_reply(buffer):
                        break

            time.sleep(0.05)
    finally:
        try:
            ser.close()
        except Exception:
            pass

    lines = [line.strip() for line in buffer.splitlines() if line.strip()]

    return wire_command.strip(), lines, buffer


# ------------------------------------------------------------
# Outputs / defaults
# ------------------------------------------------------------

sent_command = None
reply_lines = []
raw_reply = ""

joint_values_deg = []
joint_values_rad = []
cartesian_pose = []
external_axes_mm = []
j7_mm = 0.0
track_m = 0.0

calibration_state = {}
status = ""


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------

try:
    if sc is None:
        raise Exception("scriptcontext is not available.")

    port = to_text(Port, "").strip()
    baud = int(to_float(Baud, 9600))
    mode = normalize_mode(Mode)
    run = to_bool(Run, False)
    timeout = to_float(Timeout, 120.0)

    flags = [
        to_int_flag(J1),
        to_int_flag(J2),
        to_int_flag(J3),
        to_int_flag(J4),
        to_int_flag(J5),
        to_int_flag(J6),
        to_int_flag(J7),
        to_int_flag(J8),
        to_int_flag(J9)
    ]

    selected = selected_joint_names(flags)

    if not port:
        status = "No Port provided."

    elif baud <= 0:
        status = "Invalid Baud value."

    else:
        # Build command based on mode.
        if mode == "TL":
            command = "TL"

        elif mode == "RP":
            command = "RP"

        elif mode == "LL":
            command = build_ll_command(flags)

        else:
            command = "RP"

        # Edge-trigger Run so Grasshopper does not repeatedly calibrate.
        edge_key = "AR4_CALIBRATION_COMPONENT_LAST_RUN_STATE"
        previous_run = sc.sticky.get(edge_key, False)

        sc.sticky[edge_key] = run

        if not run:
            sent_command = command

            if mode == "LL":
                if selected:
                    wait_note = ""
                    if flags[6] == 1:
                        wait_note = " J7 selected: will wait until LL finishes (Timeout ignored)."
                    status = "Ready. Mode=LL. Selected joints: {}. Toggle Run True to calibrate.{}".format(
                        ", ".join(selected),
                        wait_note
                    )
                else:
                    status = "Ready. Mode=LL, but no joints are selected."

            else:
                status = "Ready. Mode={}. Toggle Run True to send {}.".format(mode, command)

        elif previous_run:
            sent_command = command
            status = "Run is still True. Not sending again. Toggle Run False, then True to resend."

        else:
            if mode == "LL" and not selected:
                sent_command = command
                status = "Mode=LL but no joints selected. Nothing sent."

            else:
                # J7 LL can exceed Timeout; wait for RP reply instead.
                ignore_timeout = bool(mode == "LL" and flags[6] == 1)
                if ignore_timeout:
                    status = "LL running for {} — waiting for firmware (Timeout ignored)...".format(
                        ", ".join(selected)
                    )

                sent_command, reply_lines, raw_reply = send_serial_command(
                    port,
                    baud,
                    command,
                    timeout,
                    ignore_timeout=ignore_timeout
                )

                rp_line = clean_rp_reply(raw_reply)
                tl_line = clean_tl_reply(raw_reply)

                rp_result = parse_rp(rp_line)
                tl_result = parse_tl(tl_line)

                joint_values_deg = rp_result["joint_values_deg"]
                joint_values_rad = rp_result["joint_values_rad"]
                cartesian_pose = rp_result["cartesian_pose"]
                external_axes_mm = rp_result["external_axes_mm"]
                j7_mm = rp_result["j7_mm"]
                track_m = rp_result["track_m"]

                calibration_state = {
                    "mode": mode,
                    "selected_joints": selected,
                    "sent_command": sent_command,
                    "reply_lines": reply_lines,
                    "rp_line": rp_line,
                    "rp_parsed": rp_result["parsed"],
                    "tl_line": tl_line,
                    "tl_parsed": tl_result["parsed"],
                    "limit_switches": tl_result["switches"],
                    # Firmware RP joints == Annin DH / URDF joints (no home offset).
                    "joint_values_deg": joint_values_deg,
                    "joint_values_rad": joint_values_rad,
                    "firmware_joint_values_deg": joint_values_deg,
                    "urdf_joint_values_deg": joint_values_deg,
                    "urdf_joint_values_rad": joint_values_rad,
                    "cartesian_pose": cartesian_pose,
                    "external_axes_mm": external_axes_mm,
                    "j7_mm": j7_mm,
                    "track_m": track_m,
                    "track_firmware_inverted": bool(rp_result.get("track_firmware_inverted", False)),
                    "track_length_mm": to_float(rp_result.get("track_length_mm"), 870.0),
                    "port": port,
                    "baud": baud
                }

                # Publish shared track convention for Program / Simulation.
                sc.sticky["AR4_TRACK_LENGTH_MM"] = calibration_state["track_length_mm"]
                sc.sticky["AR4_TRACK_FIRMWARE_INVERTED"] = calibration_state["track_firmware_inverted"]
                sc.sticky["AR4_LAST_CALIBRATION_STATE"] = calibration_state

                if mode == "TL":
                    if tl_result["parsed"]:
                        status = "TL complete. Limit switches: {}".format(tl_result["switches"])
                    elif reply_lines:
                        status = "TL sent. Reply received but could not parse switch states: {}".format(reply_lines)
                    else:
                        status = "TL sent, but no reply received."

                elif mode == "RP":
                    if rp_result["parsed"]:
                        status = "RP complete. J7 {:.3f} mm → TrackM {:.4f}. Joints parsed.".format(
                            j7_mm,
                            track_m
                        )
                    elif reply_lines:
                        status = "RP sent. Reply received but could not parse RP line: {}".format(reply_lines)
                    else:
                        status = "RP sent, but no reply received."

                elif mode == "LL":
                    if rp_result["parsed"]:
                        status = "LL calibration complete for {}. J7 {:.3f} mm → TrackM {:.4f}.".format(
                            ", ".join(selected),
                            j7_mm,
                            track_m
                        )
                    elif reply_lines:
                        status = "LL sent for {}. Reply received but no RP line parsed: {}".format(
                            ", ".join(selected),
                            reply_lines
                        )
                    else:
                        if flags[6] == 1:
                            status = "LL sent for {}, but no RP reply yet (wait was open-ended for J7).".format(
                                ", ".join(selected)
                            )
                        else:
                            status = "LL sent for {}, but no reply received before timeout.".format(
                                ", ".join(selected)
                            )

except Exception as e:
    sent_command = None
    reply_lines = []
    raw_reply = ""

    joint_values_deg = []
    joint_values_rad = []
    cartesian_pose = []
    external_axes_mm = []
    j7_mm = 0.0
    track_m = 0.0

    calibration_state = {}
    status = "ERROR: {}".format(e)