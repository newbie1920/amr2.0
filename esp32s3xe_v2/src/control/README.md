# Firmware Control Layer

Future home for wheel control code.

Current direction:

- Move wheel PID ownership here after impact analysis.
- Keep motor driver code in `src/drivers`.
- Add adaptive PID / gain scheduling here when telemetry logging is stable.
- Control output remains motor PWM commands executed by firmware safety logic.

Do not move existing headers into this folder without updating includes and running a firmware build.
