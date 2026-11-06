#! python3
# r: pyserial

"""
AR4 + External Track — Component 5: Run / Serial Sender

Accepts program/sim RJ commands (URDF joint degrees in A..F).
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
"""

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
        "Sm",
        "Ac",
        "Dc",
        "Rm",
        "W",
        "Lm"
    ]

    for token in required_tokens:
        if token not in s:
            return False, "Missing token {}.".format(token)

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

def read_reply_for_command(ser, timeout, wait_for_rp=True):
    """
    Reads serial response after one command.

    For RJ, the firmware should eventually return a position line.
    This function also stops early if it sees ER/EL-style errors.
    """
    buffer = ""
    start = time.time()

    while time.time() - start < timeout:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            text = data.decode("utf-8", errors="ignore")
            buffer += text

            lines = [line.strip() for line in buffer.splitlines() if line.strip()]

            for line in lines:
                # Firmware error strings.
                if line == "ER":
                    return lines, buffer, "ERROR_ER"

                if line.startswith("EL"):
                    return lines, buffer, "ERROR_EL"

                # RP-style position line.
                if line.startswith("A") and "B" in line and "C" in line and "P" in line:
                    return lines, buffer, "RP"

            # If we are not specifically waiting for an RP line, return after first line.
            if not wait_for_rp and "\n" in buffer:
                return lines, buffer, "LINE"

        time.sleep(0.02)

    lines = [line.strip() for line in buffer.splitlines() if line.strip()]

    if buffer:
        return lines, buffer, "TIMEOUT_WITH_DATA"

    return lines, buffer, "TIMEOUT_EMPTY"


def send_command_sequence(port, baud, commands, delay, wait_for_reply, timeout_per_command):
    import serial

    sent = []
    replies = []
    raw = []
    parsed = []

    failed = -1
    failure_reason = ""

    ser = serial.Serial(
        port=port,
        baudrate=baud,
        timeout=0.1,
        write_timeout=2.0
    )

    # Give Teensy serial connection time to settle.
    time.sleep(2.0)

    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except Exception:
        pass

    for i, command in enumerate(commands):
        ok, message = validate_rj_command(command)

        if not ok:
            failed = i
            failure_reason = "Invalid command at index {}: {}".format(i, message)
            break

        wire_command = command.strip() + "\n"

        try:
            ser.write(wire_command.encode("utf-8"))
            ser.flush()

        except Exception as e:
            failed = i
            failure_reason = "Serial write failed at index {}: {}".format(i, e)
            break

        sent.append(command.strip())

        if wait_for_reply:
            lines, buffer, reply_status = read_reply_for_command(
                ser,
                timeout_per_command,
                wait_for_rp=True
            )

            replies.append(lines)
            raw.append(buffer)

            rp_line = clean_rp_reply(buffer)
            state = parse_rp_line(rp_line)

            state["command_index"] = i
            state["reply_status"] = reply_status
            parsed.append(state)

            if reply_status in ["ERROR_ER", "ERROR_EL"]:
                failed = i
                failure_reason = "Firmware returned {} at command index {}.".format(
                    reply_status,
                    i
                )
                break

            if reply_status in ["TIMEOUT_EMPTY", "TIMEOUT_WITH_DATA"]:
                failed = i
                failure_reason = "Timed out waiting for reply at command index {}.".format(i)
                break

        else:
            replies.append([])
            raw.append("")
            parsed.append({
                "parsed": False,
                "command_index": i,
                "reply_status": "NOT_WAITING"
            })

        if delay > 0.0:
            time.sleep(delay)

    try:
        ser.close()
    except Exception:
        pass

    return sent, replies, raw, parsed, failed, failure_reason


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
status = ""


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------

try:
    if sc is None:
        raise Exception("scriptcontext is not available.")

    port = to_text(Port, "").strip()
    baud = int(to_float(Baud, 9600))

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

    delay = to_float(Delay, 0.25)
    wait_for_reply = to_bool(WaitForReply, True)
    timeout_per_command = to_float(TimeoutPerCommand, 30.0)

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

    if stop:
        sc.sticky["AR4_RUN_LAST_STATE"] = {
            "sent_commands": [],
            "program_commands": [],
            "reply_lines": [],
            "raw_replies": [],
            "parsed_states": [],
            "completed": False,
            "failed_index": -1,
            "status": "Stopped before run."
        }

        status = "Stop=True. Nothing sent."

    elif not port:
        status = "No Port provided."

    elif baud <= 0:
        status = "Invalid Baud."

    elif not program_commands:
        status = "No RJ commands provided. Connect Program rj_commands to Commands."

    else:
        # Validate encoded commands that will actually go out on serial.
        validation_errors = []

        for i, cmd in enumerate(send_commands):
            ok, msg = validate_rj_command(cmd)

            if not ok:
                validation_errors.append("Index {}: {}".format(i, msg))

        if validation_errors:
            status = "Command validation failed. {}".format("; ".join(validation_errors[:3]))

        else:
            # Edge-trigger Run.
            edge_key = "AR4_RUN_COMPONENT_LAST_RUN_STATE"
            previous_run = sc.sticky.get(edge_key, False)

            sc.sticky[edge_key] = run

            last_state = sc.sticky.get("AR4_RUN_LAST_STATE", None)

            if not run:
                if isinstance(last_state, dict):
                    sent_commands = last_state.get("sent_commands", [])
                    reply_lines = last_state.get("reply_lines", [])
                    raw_replies = last_state.get("raw_replies", [])
                    parsed_states = last_state.get("parsed_states", [])
                    completed = last_state.get("completed", False)
                    failed_index = last_state.get("failed_index", -1)

                    if raw_replies:
                        last_reply = raw_replies[-1]

                    if parsed_states:
                        last_rp_state = parsed_states[-1]
                        run_index = last_rp_state.get("command_index", -1)

                    status = (
                        "Run=False. Holding last run state. {} command(s) previously sent. {}."
                    ).format(len(sent_commands), encode_note)
                else:
                    status = "Ready. {} command(s) loaded ({}). Toggle Run True to send.".format(
                        len(program_commands),
                        encode_note
                    )

            elif previous_run:
                if isinstance(last_state, dict):
                    sent_commands = last_state.get("sent_commands", [])
                    reply_lines = last_state.get("reply_lines", [])
                    raw_replies = last_state.get("raw_replies", [])
                    parsed_states = last_state.get("parsed_states", [])
                    completed = last_state.get("completed", False)
                    failed_index = last_state.get("failed_index", -1)

                    if raw_replies:
                        last_reply = raw_replies[-1]

                    if parsed_states:
                        last_rp_state = parsed_states[-1]
                        run_index = last_rp_state.get("command_index", -1)

                    status = "Run is still True. Not resending. Toggle Run False, then True to run again."
                else:
                    status = "Run is True, but no previous run state exists. Toggle Run False, then True."

            else:
                sent_commands, reply_lines, raw_replies, parsed_states, failed_index, failure_reason = send_command_sequence(
                    port=port,
                    baud=baud,
                    commands=send_commands,
                    delay=delay,
                    wait_for_reply=wait_for_reply,
                    timeout_per_command=timeout_per_command
                )

                if raw_replies:
                    last_reply = raw_replies[-1]

                if parsed_states:
                    last_rp_state = parsed_states[-1]
                    run_index = last_rp_state.get("command_index", -1)

                completed = failed_index < 0 and len(sent_commands) == len(send_commands)

                run_state = {
                    "sent_commands": sent_commands,
                    "program_commands": program_commands,
                    "encoded_commands": send_commands,
                    "encode_firmware_on_send": encode_on_send,
                    "home_urdf_deg": list(home_urdf_deg),
                    "joint_signs": list(joint_signs),
                    "encode_maps": encode_maps,
                    "reply_lines": reply_lines,
                    "raw_replies": raw_replies,
                    "parsed_states": parsed_states,
                    "last_reply": last_reply,
                    "last_rp_state": last_rp_state,
                    "run_index": run_index,
                    "completed": completed,
                    "failed_index": failed_index,
                    "port": port,
                    "baud": baud
                }

                sc.sticky["AR4_RUN_LAST_STATE"] = run_state

                if completed:
                    status = "Run complete. Sent {} of {} command(s) ({}).".format(
                        len(sent_commands),
                        len(send_commands),
                        encode_note
                    )
                else:
                    status = "Run stopped/failure. Sent {} of {} command(s). Failed index: {}. {} ({})".format(
                        len(sent_commands),
                        len(send_commands),
                        failed_index,
                        failure_reason,
                        encode_note
                    )

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
    status = "ERROR: {}".format(e)