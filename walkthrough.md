# AMR Firmware Walkthrough

## esp32s3xe_v2 Smooth Tracking Default

`esp32s3xe_v2` is the active real-vehicle firmware for this milestone. Its onboard navigator now boots into the smoother tracking preset:

- Tracking controller: `fl_preview`
- Trajectory profile: `adaptive_scurve_corner`
- Runtime rollback presets: `smooth_default`, `legacy`, `safe`, `fl_regularized`

### Config Smoke Commands

Send these WebSocket JSON config messages from the app/dev console or a small bench client:

```json
{"type":"config","preset":"smooth_default"}
{"type":"config","preset":"legacy"}
{"type":"config","preset":"fl_regularized"}
```

Expected ack: `type:"config_ack"`, `scope:"preset"`, plus the active `tracking` and `trajectory` names.

### Low-Speed Floor Test

1. Put the robot in onboard mode and confirm telemetry reports `track.controller:"fl_preview"` and `traj.mode:"adaptive_scurve_corner"`.
2. Run straight 1 m at low speed and record final distance/yaw drift. Confirm `drive.hold:true` and small `drive.headingErr`; if `drive.headingCorr` saturates, lower speed and inspect `vL_r` versus `vR_r`.
3. Run an L-turn path and watch for overshoot, wheel saturation, or oscillation.
4. Run a gentle S-curve and compare lateral error against the benchmark trend.
5. If the robot shakes, lower `previewHeadingGain`; if it cuts corners too fast, lower `maxCornerSpeed`.

---

# AMR Firmware v2.1 Walkthrough

## What Changed

`esp32s3xe_v2_1` is now a separate firmware rebuild scaffold. It does not edit or replace `esp32s3xe_v2`.

The first buildable loop is intentionally fake-HAL based:

1. Serial command enters `ProtocolBridge`.
2. Parsed `RobotCommand` is published to `CommandBus`.
3. `TaskRegistry` dispatches commands.
4. `SimplePlanner` creates a minimal onboard path.
5. `TrajectoryProfile` samples the path.
6. `TrajectoryTracker` generates target velocity.
7. `SafetyGate` clamps/brakes output.
8. Fake motor/encoder simulate movement.
9. Telemetry prints `pose_fast`, `nav_debug`, and `system_health`.

## Manual Smoke Commands

Open a serial monitor after flashing v2.1, then try:

```text
goto 1.0,0.5
vel 0.1,0.0
stop
onboard
pc_browser
```

## Next Safe Migration Step

`esp32s3xe_v2.1codex` now has a real-HAL runtime draft. The next safe migration is occupancy-grid planning; do not floor-test autonomous obstacle navigation until that path is ported and benchmarked.

## v2.1 Codex Runtime

- Build folder: `esp32s3xe_v2.1codex`
- AP: `AMR_V21` / `amr2021nav`
- WebSocket port: `81`
- Runtime tasks: `v21_control`, `v21_network`, `v21_telem`
- Current autonomous flow: direct `goto` planner, trajectory sampler, tracker, safety gate, real motor HAL, A1M8 scan HAL
- Idle onboard boot should show `fault:"none"`; `command_timeout` is only expected after manual `vel v,w` control goes stale.
