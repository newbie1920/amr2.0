# AMR Firmware Task Tracker

## Current Goal - esp32s3xe_v2 Smooth Tracking Default

Promote `esp32s3xe_v2` to use the smoother onboard tracking stack by default:
`fl_preview` controller plus `adaptive_scurve_corner` trajectory profile.

## Active Scope - esp32s3xe_v2

- Keep `esp32s3xe_v2` as the firmware used for the real vehicle in this milestone.
- Keep runtime rollback paths through config: `preset:"legacy"`, `preset:"safe"`, or explicit `tracking`/`trajectory` config.
- Validate with PlatformIO build, trajectory benchmark, app tests/build, then low-speed floor tests.

## Done - esp32s3xe_v2 Smooth Tracking

- Switched boot defaults to `fl_preview` + `adaptive_scurve_corner`.
- Kept legacy/backstepping and `fl_regularized` selectable by config.
- Added config presets for `smooth_default`, `legacy`/`safe`, and `fl_regularized`.
- Updated benchmark naming so `firmware_current` means the new smooth default and `legacy_baseline` preserves the old comparison.
- Added independent wheel-PID target generation with straight heading-hold shared by onboard nav and manual velocity commands.

## Next - esp32s3xe_v2 Smooth Tracking

- Run bench validation and update `docs/03_Research/trajectory_tracking_benchmark.md`.
- Floor-test low speed: straight 1 m, L-turn, S-curve; watch telemetry `drive.hold`, `drive.headingErr`, `drive.headingCorr`, `vL_r`, and `vR_r`.
- If the robot shakes or cuts corners too hard, reduce `previewHeadingGain` or `maxCornerSpeed` and rerun validation.

---

# AMR Firmware v2.1 Task Tracker

## Current Goal

Build a clean navigation-first firmware line in `esp32s3xe_v2_1` while preserving `esp32s3xe_v2` as the rollback baseline.

## Active Scope

- Rebuild firmware architecture with explicit layers and contracts.
- Default to onboard navigation as source-of-truth.
- Prevent old recurring issues: broad `extern` globals, god files, protocol mixed with control logic, unbenchmarked navigation changes.

## Done

- Created independent v2.1 PlatformIO project.
- Added HAL contracts and fake HAL bench rig.
- Added state store, command/fault queues, domain command/snapshot types.
- Added onboard `goto → plan → trajectory → tracker → safety → motor` scaffold.
- Added thin protocol bridge with ack and telemetry streams.
- Added `esp32s3xe_v2.1codex` full-runtime draft with real HAL, FreeRTOS tasks, WebSocket ingress, and serial fallback.

## Next

- Port A1M8 scan decoding and occupancy grid into v2.1 HAL/perception.
- Port v2 trajectory/tracking controller into isolated v2.1 modules.
- Add app compatibility bridge for current browser command naming.
- Expand benchmarks for obstacle replanning and controller regressions.
