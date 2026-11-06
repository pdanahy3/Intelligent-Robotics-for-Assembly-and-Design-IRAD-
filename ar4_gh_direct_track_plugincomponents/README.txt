AR4 Grasshopper Python component replacements

Replace each GhPython component script with the matching .py file:

01_startup_connection.py
  Inputs:  Load, Reload, URDF, PackageRoot, PreferredPort, Baud, TestCommand, Connect
  Outputs: cache_id, robot_model, robot_name, joint_names, motion_joint_names, link_names, mesh_link_names, mesh_count, available_ports, active_port, handshake_reply, startup_state, status

02_calibration.py
  Inputs:  Port, Baud, Mode, J1, J2, J3, J4, J5, J6, J7, J8, J9, Run, Timeout
  Outputs: sent_command, reply_lines, raw_reply, joint_values_deg, joint_values_rad, cartesian_pose, external_axes_mm, j7_mm, track_m, calibration_state, status
  Modes: RP, TL, LL, Z7

03_program_ik_planner.py
  Inputs:  cache_id, Targets, ToolZ, Speed, Accel, Decel, Ramp, Wrist, UseTrack, Solve
  Outputs: solved, program_joint_values_rad, program_joint_values_deg, program_track_m, program_j7_mm, rj_commands, ik_errors, target_planes_robot, tcp_planes_solved, program_state, status
  Fixes: TrackM convention 0=motor side / 1=far end; split J7-only commands at 100%; 5-minute timeout.

04_simulation_visualization.py
  Inputs:  Commands, Time, DrawRobot, DrawTrack, DrawPath, DrawTargets, DrawSkeleton
  Outputs: active_index, active_command, active_joint_values_deg, active_joint_values_rad, active_track_m, active_j7_mm, robot_geometry, track_geometry, tcp_planes, tcp_path, target_geometry, skeleton_lines, simulation_state, status

05_run_serial_sender.py
  Inputs:  Port, Baud, Commands, Run, Stop, Delay, WaitForReply, TimeoutPerCommand
  Outputs: sent_commands, reply_lines, raw_replies, parsed_states, last_reply, last_rp_state, run_index, completed, failed_index, status

06_teach_pose_calibration.py
  Inputs:  Port, Baud, Action, Run, Timeout, TrackLengthMM
  Outputs: sent_command, reply_lines, raw_reply, current_joints_deg, current_j7,
           pose_a_joints_deg, pose_a_j7, pose_b_joints_deg, pose_b_j7,
           joint_offsets_deg, j7_offset, j7_span, j7_sign, j7_scale,
           recommended_home_deg, recommended_track_length_mm, track_firmware_inverted,
           calibration_report, teach_state, status
  Actions: READ, TEACH_A, TEACH_B, COMPUTE, CLEAR
  Pose A expected: all joints 0, J7=0
  Pose B expected: arm joints 0, J7=TrackLengthMM (1000)
  Writes sticky: AR4_HOME_URDF_DEG, AR4_TRACK_LENGTH_MM, AR4_TRACK_FIRMWARE_INVERTED

Track convention in all rewritten components (simulation is source of truth):
  TrackM = 0.0 -> motor / limit-switch side -> firmware J7 = 0
  TrackM = 1.0 -> far end                  -> firmware J7 = TrackLengthMM (1000 for 1 m)
  J7 = TrackM * TrackLengthMM
  Firmware MotDir/CalDir/length should be set so RP matches the simulation.
  TrackFirmwareInverted is only a temporary bridge if firmware is still opposite the sim.
  Set Teensy J7axisLimPos / HMI CE length to 1000 for a 1 m track (not 3450/6000).

Suggested workflow:
  1. Startup: Load=True, Reload=True once.
  2. Calibration: use your normal LL or RP modes after fixing calibration direction in firmware.
  3. Teach poses (06): TEACH_A at visual zero / J7 home, TEACH_B at visual zero / J7 far.
  4. Use calibration_report offsets for J1 zero error and J7 scale/sign.
  5. Program: Solve=False, set targets; then Solve=True once.
  6. Simulation: Connect Program rj_commands.
  6. Run: Connect Program rj_commands, then toggle Run False->True.
