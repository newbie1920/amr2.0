# AMR2.0 Codex Memory

## Stable Facts

- AMR2.0 is firmware-first: ESP32-S3 handles mapping, localization, path
  planning, trajectory tracking, PID, and safety for the real robot.
- The React/Tauri app is a control and monitoring station, plus a simulator and
  debug surface.
- `task.md` is often the current scope-of-truth for firmware/controller work.
- Research notes under `docs/03_Research/` are usually meant to drive real
  vehicle upgrades, not just summaries.
- Demo-friendly defaults should remain safe; experimental controller paths
  should be opt-in.

## Useful Validation Commands

```powershell
npm.cmd run test
npm.cmd run build
npm.cmd run check
npm.cmd run benchmark:trajectory
git diff --check
```

Firmware, when PlatformIO is available:

```powershell
cd esp32s3xe_v2
python -m platformio run
```
