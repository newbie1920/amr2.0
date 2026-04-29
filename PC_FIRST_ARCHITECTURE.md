# AMR 2.0 PC-First Architecture

## Goal

Move AMR 2.0 toward a cleaner split:

- `ESP32-S3`: real-time edge controller
- `PC App`: SLAM, localization, planning, mission logic

This matches your actual deployment model better than pretending the ESP32 is a small ROS computer.

## Role Split

### ESP32 keeps

- motor PID
- encoder counting
- IMU sampling / fusion
- LiDAR acquisition
- battery / power telemetry
- obstacle emergency stop
- manual velocity execution
- optional fallback onboard path tracking in `hybrid`

### PC app owns

- occupancy mapping
- scan matching / localization
- frontier exploration
- global planning
- local planning
- task orchestration
- map save/load/import/export

## Architecture Profiles

The firmware now supports:

### `hybrid`

- occupancy grid stream: `ON`
- onboard navigator: `ON`
- useful for compatibility with older task flows

### `pc_slam`

- occupancy grid stream: `OFF`
- onboard navigator: `OFF`
- raw lidar + odom telemetry still stream to the app
- intended for browser-side SLAM/localization/planning

## Current App Behavior

When the app starts:

- mapping -> firmware switches to `pc_slam`
- localization -> firmware switches to `pc_slam`
- stop mapping/localization -> firmware returns to `hybrid`

This reduces duplicate mapping work and makes the app the clear SLAM owner.

## Why this helps

Before:

- ESP32 built a grid
- app also built a grid
- ESP32 nav and app nav both existed
- state ownership was blurry

Now:

- during SLAM work, only the PC-side mapper is authoritative
- firmware still enforces low-level safety
- telemetry makes the active authority visible in UI

## Next recommended steps

1. Route task navigation through app-side path following instead of `navigate()` on ESP32.
2. Replace single global exploration state with per-robot state.
3. Add PC-side navigation state machine so task completion no longer depends on firmware `nav`.
4. Export maps in ROS-compatible formats (`pgm/yaml`) in addition to app JSON.
5. Add localization confidence and drift metrics to UI.
