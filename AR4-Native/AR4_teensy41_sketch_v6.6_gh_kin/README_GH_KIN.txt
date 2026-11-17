AR4 Teensy 6.6-gh-kin (Grasshopper / URDF Cartesian fork)
=========================================================

Revert stock anytime
  Flash: ../AR4_teensy41_sketch_v6.5_annin_stock_backup/AR4_teensy41_sketch_v6.5.ino
  Or set GH_KIN_ALIGN 0 in this sketch and reflash.

What this fork changes
  - FIRMWARE_VERSION = "6.6-gh-kin"
  - SolveFowardKinematics: Annin FK, then Rx(pi) so RP G..L are GH base_link
  - SolveInverseKinematics: ML XYZ/uvw treated as GH base_link, Rx(pi) to Annin,
    then stock closed-form IK (JangleOut stays firmware degrees)
  - DHparams J3 d = 7 mm (URDF joint_3 z offset)

Why Rx(pi)
  URDF joint_1 origin rpy=(pi,0,0) makes the J1 axis -Z in base_link; Annin J1
  is +Z in its base. Comp3 JointSigns[0]=-1 matches the motor sense; Cartesian
  ML still needs this base remap.

Flash / test
  1. Close Comp1 Connect (release COM)
  2. Flash AR4_teensy41_sketch_v6.6_gh_kin.ino (Teensy 4.1)
  3. Comp1 Connect=True
  4. Comp2 RP at a known RJ pose — G,H,I should be closer to GH flange XYZ
  5. Comp3 MotionMode=spline, short open-air path, Comp5 Run
  6. If wrong: flash stock backup; set MotionMode=rj

Comp3
  Spline emits SL + bridged ML (arm-base flange + uvw) + SS for this firmware.