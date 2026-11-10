AR4 Grasshopper Python component replacements
=============================================

Replace each Grasshopper Python 3 component script with the matching .py file.
Full documentation is in ../README.md; this file is the quick reference.

Components communicate through scriptcontext.sticky, so component 1 must run
first to seed the shared defaults.


01_startup_connection.py
  Wired:    Load, Reload, URDF, PackageRoot, PreferredPort, Baud, TestCommand,
            Connect
  Optional: BasePlane, ToolMeshes, ToolTCPPlane, ToolTCPPoint
  Outputs:  cache_id, robot_model, robot_name, joint_names, motion_joint_names,
            link_names, mesh_link_names, mesh_count, available_ports,
            active_port, handshake_reply, base_plane, tool_meshes,
            tool_tcp_plane, startup_state, status
  ToolMeshes is a Mesh parameter, List access. Breps are meshed on the way in.

02_calibration.py
  Wired:    Port, Baud, Mode, J1..J9, Run, Timeout
  Outputs:  sent_command, reply_lines, raw_reply, joint_values_deg,
            joint_values_rad, cartesian_pose, external_axes_mm, j7_mm, track_m,
            calibration_state, status
  Modes:    RP, TL, LL, Z7

03_program_ik_planner.py
  Wired:    Targets, Speed, Accel, Decel, Ramp, Wrist, UseTrack, Solve
  Optional: TCPPlane, TCPPoint, UseCalibratedTCP, PosTolMM, RotTolDeg,
            RotationWeight, MaxSeeds, MaxSolveSeconds, BasePlane, TargetUnitsMM,
            SplitTrackMotion, TrackSpeedPercent, TrackLengthMM,
            TrackRailLengthMM, TrackMoveEpsilonMM, TrackFirmwareInverted,
            HomeJointsDeg, JointSigns, UseCalibrationFrame, ToolZ (legacy)
  Outputs:  solved, program_joint_values_rad, program_joint_values_deg,
            program_track_m, program_j7_mm, rj_commands, ik_errors,
            tcp_pos_error_mm, tcp_rot_error_deg, target_planes_robot,
            tcp_planes_solved, program_state, status
  Speed is percent of max step rate (Sp), 1-100, default 25.

04_simulation_visualization.py
  No serial. No Port/Baud inputs.
  Optional: Commands, Time, DrawRobot, DrawTrack, DrawPath, DrawTargets,
            DrawSkeleton, HideGripper, ToolMeshes, TCPPlane, TCPPoint,
            BasePlane, TrackLengthMM, TrackRailLengthMM,
            TrackFirmwareInverted, HomeJointsDeg, JointSigns, ToolZ
  Outputs:  active_index, active_command, active_joint_values_deg,
            active_joint_values_rad, active_track_m, active_j7_mm,
            robot_geometry, tool_geometry, track_geometry, tcp_planes, tcp_path,
            target_geometry, skeleton_lines, simulation_state, status
  The URDF gripper hides automatically when any tool mesh is present.

05_run_serial_sender.py
  Wired:    Port, Baud, Commands, Run, Stop, Delay, WaitForReply,
            TimeoutPerCommand
  Optional: MaxInFlight, EncodeFirmwareOnSend, HomeJointsDeg, JointSigns
  Outputs:  sent_commands, reply_lines, raw_replies, parsed_states, last_reply,
            last_rp_state, run_index, completed, failed_index, status
  Run is edge-triggered: toggle False then True to send again.
  MaxInFlight caps unacknowledged commands at 2, one below the controller's
  three-deep queue. TimeoutPerCommand must exceed the longest single move.

06_jog_control.py
  Optional: J1..J6, TrackM, Send, Capture, CaptureLabel, ClearCaptures, Port,
            Baud, Speed, SpeedType, Accel, Decel, Ramp, Wrist, SendTimeout,
            DrawRobot, DrawTrack, DrawSkeleton, HideGripper, ToolMeshes,
            TCPPlane, TCPPoint, BasePlane, TrackLengthMM, TrackRailLengthMM,
            TrackFirmwareInverted, HomeJointsDeg, JointSigns, ToolZ,
            EncodeFirmwareOnSend
  Outputs:  robot_geometry, tool_geometry, track_geometry, skeleton_lines,
            flange_plane, tcp_plane, rj_command, captures, capture_count,
            capture_labels, jog_state, status
  Send is edge-triggered, so unrelated slider moves do not re-fire it.

07_tcp_calibration.py
  Optional: Captures (No Type Hint, List access), Solve, Apply, Clear,
            MaxAxisTiltDeg
  Outputs:  tcp_frame, tcp_plane_local, tcp_offset_mm, fit_max_mm, fit_rms_mm,
            calibration_point, warnings, status
  Capture labels: tip* (4 or more, one physical point from different
  orientations), axisx* and axisz* (translation-only moves), ref* (reference
  orientation). Writes sticky AR4_TCP_FRAME.


Track convention (the simulation is the source of truth)
--------------------------------------------------------
  TrackM = 0.0 -> motor / limit-switch side -> firmware J7 = 0
  TrackM = 1.0 -> far end                   -> firmware J7 = TrackLengthMM
  J7 = TrackM * TrackLengthMM
  Usable travel 870 mm on a 1000 mm rail.
  Set Teensy J7axisLimPos and the HMI length to 1000 for a 1 m track,
  not 3450 or 6000. TrackFirmwareInverted is only a bridge if the firmware is
  still opposite the simulation.


Joint frame mapping
-------------------
  firmware = (urdf - HomeJointsDeg) * JointSigns
  HomeJointsDeg = [-16, 0, 0, 0, 22, -10]
  JointSigns    = [-1, 1, 1, 1, 1, 1]
  Flipping JointSigns[0] requires flipping HomeJointsDeg[0] with it.


Workflow
--------
  1. Startup (01): Load=True, Reload=True once.
  2. Calibration (02): RP or LL as needed.
  3. Jog (06): drive to a known pose; Capture labelled poses for TCP work.
  4. TCP calibration (07): Solve, then Apply to publish the frame.
  5. Program (03): set Targets with Solve=False, then Solve=True once.
  6. Simulation (04): connect the Program rj_commands output.
  7. Run (05): connect rj_commands, then toggle Run False -> True.
