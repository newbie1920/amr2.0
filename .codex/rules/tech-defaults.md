# Tech Defaults

## 1. Project Structure
You are working in `C:\code2\AMR2.0`, a mixed robotics project:
- `esp32s3xe_v2/`: ESP32-S3 vehicle-brain firmware in C++/Arduino/PlatformIO.
- `src/`: React 19 + Vite + Tauri monitoring/control app.
- `docs/`: TDTU thesis, architecture notes, research notes.
- `scripts/` & `tests/`: automation, benchmarks, regression tests.

## 2. Frontend
- React 19 + Vite + Tauri.
- Zustand for app state.
- Three.js / React Three Fiber for 3D views.
- Use ES modules and existing project patterns.
- Prefer `npm.cmd` and `npx.cmd` commands on Windows.
- **Operating Rule:** For UI state that must rerender, use React/Zustand hook selectors rather than `.getState()`.

## 3. Firmware
- ESP32-S3 N16R8.
- C++ / Arduino / PlatformIO.
- FreeRTOS tasks.
- Keep allocation-sensitive and realtime paths conservative.
- **Operating Rule:** For firmware/controller changes, keep the safe baseline as default.
- **Operating Rule:** Treat firmware telemetry as source of truth for real-robot behavior.

## 4. Data, Reports, and Thesis
- Python 3.10+ for data scripts when Node is not a better fit.
- Keep report generation reproducible from scripts.
- **Operating Rule:** For thesis/report artifacts, follow TDTU MauDATN_2021 style (Python 3.10+, C++ Arduino, JS ES6+).
- **Operating Rule:** Preserve user work. The worktree is often dirty; never revert unrelated changes.
