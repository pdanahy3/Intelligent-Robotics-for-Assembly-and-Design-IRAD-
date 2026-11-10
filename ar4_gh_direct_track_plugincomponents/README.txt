AR4 Grasshopper Python component replacements
=============================================

Full docs: ../README.md. Pin names/types: COMPONENT_PINS.txt.
Each .py starts with a GH_PINS copy-paste block.

SERIAL
  Comp1 Connect=True owns the COM port (AR4_SERIAL_BUS).
  Comp2 / Comp5 / Comp6 have no Port/Baud pins.

TOOL / TCP
  Set ToolMeshes + ToolTCP* only on Comp1.
  Comp7 Apply writes calibrated AR4_TCP_FRAME.
  Downstream components read sticky — remove old Tool/TCP pins from 03/04/06.


01 Startup
  Inputs:  Load, Reload, URDF, PackageRoot, PreferredPort, Baud, TestCommand,
           Connect, BasePlane, ToolMeshes, ToolTCPPlane, ToolTCPPoint
  Outputs: cache_id … connected, live_joint_*, tool_*, startup_state, status
  Leave Connect=True for the session.

02 Calibration
  Inputs:  Mode, J1..J9, Run, Timeout
  Modes:   RP, TL, LL, Z7

03 IK Planner
  Inputs:  cache_id (from Comp1), Targets, Speed, Accel, Decel, Ramp, Wrist,
           UseTrack, Solve, UseCalibratedTCP, tolerances, track/home options
  No TCPPlane/TCPPoint/ToolZ pins. cache_id is required (falls back to sticky).
  Speed is Sp percent 1-100, default 25.

04 Simulation
  Inputs:  Commands, Time, draw flags, HideGripper, BasePlane, track options
  No ToolMeshes / TCP pins — uses Comp1/Comp3/Comp7 sticky.

05 Run (streaming)
  Inputs:  Commands, Run, Stop, Delay, WaitForReply, TimeoutPerCommand,
           MaxInFlight, EncodeFirmwareOnSend, HomeJointsDeg, JointSigns, DrawRobot
  Outputs: sent_*, completed, failed_index, live_joint_values_deg, live_track_m,
           live_j7_mm, flange_plane, tcp_plane, robot_geometry, tool_geometry, status
  Run streams one ack per solve and re-expires so GH preview updates live.
  Wire robot_geometry / tool_geometry to a Custom Preview.

06 Jog
  Inputs:  Mode (joint|cartesian), J1..J6, TrackM, X,Y,Z, Rx,Ry,Rz, Send,
           Capture*, draw flags, speed tokens, BasePlane, track/home options
  Outputs: geometry, flange/tcp planes, joints_urdf_deg, xyz_mm, rpy_deg,
           rj_command, captures, status
  Cartesian: absolute robot-base X,Y,Z mm + Rx,Ry,Rz deg (same as Comp3).
  Holds current TCP on mode entry until you move a pin. Analytic chain IK.
  No Tool/TCP pins.

07 TCP Calibration
  Inputs:  Captures, Solve, Apply, Clear, MaxAxisTiltDeg
  Labels:  tip*, axisx*, axisz*, ref*


Workflow
  1. Comp1 Load + Connect=True (set tool here)
  2. Comp2 calibrate as needed
  3. Comp6 jog (joint or cartesian); Capture for TCP
  4. Comp7 Solve + Apply
  5. Comp3 Solve targets
  6. Comp4 simulate
  7. Comp5 Run — watch live robot_geometry
  8. Comp1 Connect=False when done
