# Skill: AMR2 Robot Upgrade

Use this skill when the user asks to improve the ESP32-S3 vehicle, trajectory
tracking, mapping, navigation, telemetry, or robot-control behavior.

## Steps

1. Read `task.md` first if it exists.
2. Search relevant firmware/app files with `rg` and GitNexus.
3. Identify whether the change belongs to firmware, app, docs, or benchmark
   tooling.
4. Keep baseline behavior safe by default.
5. Add config flags or telemetry fields for experiments when needed.
6. Validate with the narrowest useful set:
   - `npm.cmd run benchmark:trajectory` for controller comparisons.
   - `npm.cmd run test` for JS regressions.
   - `npm.cmd run build` for frontend compile.
   - `python -m platformio run` from `esp32s3xe_v2` when available.

## High-Signal Files

- `task.md`
- `esp32s3xe_v2/src/navigation/navigator.h`
- `esp32s3xe_v2/src/navigation/trajectory/`
- `esp32s3xe_v2/src/network/network_comm.cpp`
- `src/components/RVizPanel/`
- `src/stores/navStore.js`
- `scripts/benchmark_trajectory_tracking.js`
- `tests/trajectoryBenchmark.test.js`
