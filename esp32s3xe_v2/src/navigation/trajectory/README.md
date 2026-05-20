# Firmware Trajectory Layer

`trajectory_profile.h` is the first lightweight firmware trajectory generator for AMR 2.0.

It intentionally starts smaller than the full KinoDynamics paper:

- fixed-size path buffer for ESP32 safety
- configurable `vMax`, `aMax`, `jMax`, and `enabled`
- per-segment acceleration-limited reference with jerk-limited acceleration ramp
- `profileType` and `segmentProgress` telemetry for debugging and thesis plots
- reference output for the feedback-style `Navigator` tracker

Later work can replace the simple profile with the full Adaptive Topology S-curve and fixed-time synchronization logic.
