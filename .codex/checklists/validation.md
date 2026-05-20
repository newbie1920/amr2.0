# Validation Checklist

Pick the narrowest meaningful check for the actual change.

## Frontend/App

- [ ] `npm.cmd run test`
- [ ] `npm.cmd run build`
- [ ] `npm.cmd run check`
- [ ] Browser screenshot or visual check for UI changes when practical.

## Firmware

- [ ] `python -m platformio run` from `esp32s3xe_v2` when PlatformIO is
      available.
- [ ] Benchmark or sim check before promoting risky controller behavior.

## Trajectory/Robot Control

- [ ] `npm.cmd run benchmark:trajectory`
- [ ] Confirm baseline remains default unless user requested replacement.
- [ ] Confirm experimental behavior is opt-in through config or telemetry.

## Docs/Config Only

- [ ] Parse JSON/YAML/TOML when relevant.
- [ ] `git diff --check`
