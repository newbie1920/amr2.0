# AMR 2.0 - Project Context

## 1. PROJECT OVERVIEW
* **Goal of the system:** Develop a high-performance, ROS2-inspired Autonomous Mobile Robot (AMR) system where the complex navigation stack (SLAM, Pathfinding, DWA) is offloaded entirely to a web browser Dashboard. This allows cheap ESP32 hardware to function as a professional-grade AMR without needing an onboard Raspberry Pi.
* **Core features:** 
  - Correlative Scan Matching (SLAM) & Log-Odds Occupancy Grid Mapping.
  - A* Pathfinding with Gradient Descent Path Smoothing and RDP simplification.
  - Dynamic Window Approach (DWA) Local Planner.
  - Frontier-based Autonomous Exploration (State Machine).
  - Web Worker Multi-threading for heavy algorithms.
  - Priority-based Velocity Multiplexer.
* **Tech stack:** React.js, Zustand, Vite, Comlink (Web Workers), Three.js (UI rendering), C++/PlatformIO (ESP32 Firmware), WebSockets.

## 2. ARCHITECTURE
* **High-level design:** The architecture follows a Master-Slave pattern. The **Web-App (Master)** processes Lidar data, builds the map, computes paths, and calculates real-time `cmd_vel` (v, w). The **ESP32 (Slave)** receives `cmd_vel` via WebSockets, translates it to wheel speeds using onboard PID control, and streams back Lidar & Encoder telemetry at 10Hz.
* **Key modules and responsibilities:**
  - `robotStore.js`: Central Zustand store. Manages WebSocket connections, telemetry updates, and orchestrates the navigation pipeline.
  - `navWorker.js`: The background thread (Web Worker) executing CPU-intensive tasks.
  - `exploration.js`: State machine managing the robot's autonomous mode (Init Spin → Find Frontier → Navigate → Arrived Scan).
  - `lidarMapper.js` / `scanMatcher.js`: Map generation and localization.
  - `lidarPathfinder.js` / `dwaPlanner.js`: Global pathfinding and local obstacle avoidance.
* **Data flow (important):**
  1. ESP32 sends telemetry JSON `{"lidar": [...], "x":..., "y":..., "heading":...}`.
  2. `robotStore.js` receives telemetry. Raw Odometry is logged.
  3. Async Web Worker runs `matchScan(lidar, grid)` to compute a `mapToOdom` TF correction.
  4. The global Map Pose is calculated: `mapPose = odomPose + mapToOdom TF`.
  5. The Occupancy Grid is updated using `mapPose` and Lidar rays.
  6. Concurrently, `exploration.js` runs a 10Hz loop. It asks the Worker to `findPath` to a frontier, then asks the Worker to calculate `computeVelocity` (DWA) to follow the path.
  7. Velocity command passes through `VelocityMux` and is sent to ESP32 as `{"type":"cmd_vel", "linear": v, "angular": w}`.

## 3. CURRENT STATE
* **What has been implemented:** SLAM mapping, A* Pathfinding + Smoothing, DWA Local Planner, Velocity Multiplexer, UI Dashboard, ESP32 WebSocket server, and Web Worker offloading (Phase 5).
* **What is partially done:** Multi-robot visualization (UI supports it, but algorithms are heavily tied to the first robot), Map persistence (saving to localStorage works, but reloading to resume SLAM / Pure Localization is untested).
* **What is NOT started:** Dynamic, moving obstacle tracking in the Costmap; Multi-floor or fleet traffic control.

## 4. ACTIVE TASK
* **What exactly we were just working on before this export:** We successfully completed **"Phase 5: Moving heavy algorithms to Web Workers"**. We installed `comlink`, moved `ScanMatcher`, `Pathfinder`, and `DWAPlanner` into `navWorker.js`, and updated `robotStore.js` and `exploration.js` to use asynchronous promises with locking mechanisms (`isMatching` and `workerBusy`).
* **Why this task matters:** JavaScript runs in a single Main Thread. Complex operations like calculating DWA trajectories over an 80x80 costmap at 10Hz, or doing brute-force Correlative Scan Matching, was freezing the React UI and causing WebSocket packet drops. Offloading to a Worker ensures the UI remains silky smooth.

## 5. REASONING & DECISIONS
* **Key decisions made and why:**
  - *Browser-based Nav Stack:* Decided against running ROS2 natively to keep hardware costs low. The browser V8 engine is surprisingly fast for robotics math if engineered correctly.
  - *Comlink over raw `postMessage`:* Used Google's `comlink` to maintain a clean OOP architecture without dealing with complex event listeners and message routing.
  - *DWA instead of Pure Pursuit:* Pure pursuit blindly follows waypoints. DWA simulates physics and respects the inflation costmap, ensuring the robot doesn't clip corners or hit new obstacles.
* **Trade-offs considered:**
  - *Data Serialization:* Moving the `OccupancyGrid` to the Worker requires passing the `logOdds` and `costmap` Float32Arrays. This adds a slight copy overhead (serialize/deserialize) instead of shared memory, but `SharedArrayBuffer` was avoided because it requires strict cross-origin isolation headers which complicate local development.
* **Assumptions:** Assuming the ESP32 PID controller is perfectly tuned and capable of accurately honoring the `cmd_vel` target speeds instantly.

## 6. KNOWN ISSUES / BUGS
* **Current errors:** No blocking errors, but if the network drops, the Worker promises might hang or state desynchronization can occur.
* **Edge cases:** If the robot spins too fast, Lidar scan distortion (skew) might break the Scan Matcher because we do not compensate for motion during the 360-degree Lidar spin.
* **Technical risks:** Worker serialization overhead could become a bottleneck if the map expands to an enormous size (e.g., 1000x1000 cells).

## 7. NEXT ACTION PLAN (STEP-BY-STEP)
1. **Verify AMCL/Pure Localization:** Test loading a saved static map and running the robot purely in Localization Mode (Updating `mapToOdom` but freezing `logOdds` updates).
2. **Optimize Map Rendering:** Convert the OccupancyGrid React rendering from `canvas` 2D context to a WebGL/Three.js texture for better performance on large maps.
3. **Advanced DWA Tuning:** Expose DWA parameters (`v_max`, `w_max`, `sim_time`, `path_dist_bias`) to the UI so users can tune the robot's aggressiveness in real-time.

## 8. IMPORTANT FILES
* `src/stores/robotStore.js`: The heart of the application. Manages connections, TF frames (`mapToOdom`), and triggers SLAM mapping.
* `src/core/navWorker.js`: The Web Worker exposing SLAM, Pathfinding, and DWA to the Main Thread via Comlink.
* `src/core/exploration.js`: The brain of the robot. A state machine that autonomously drives the robot to unexplored frontiers.
* `src/core/lidarMapper.js` & `src/core/scanMatcher.js`: Log-odds probability math and Lidar alignment algorithms.
* `c:\Users\ADMIN\Documents\PlatformIO\Projects\esp32s3xe\src\main.cpp`: ESP32 firmware handling WiFi, WebSockets, Encoder interrupts, PID motor control, and LD19 Lidar parsing.

## 9. SETUP INSTRUCTIONS
* **Web Application:** 
  1. Open terminal at `c:\code2\AMR2.0`
  2. Run `npm install` (make sure `comlink` is installed).
  3. Run `npm run dev`. Access at `http://localhost:5173`.
* **ESP32 Firmware:**
  1. Open `esp32s3xe` folder in VSCode + PlatformIO.
  2. Build and Upload to the ESP32-S3 via USB.
  3. Update the Robot's IP in the Web UI to connect.

## 10. PROMPT CONTINUATION BLOCK
```markdown
You are picking up the AMR 2.0 Web-based Navigation Project.
Please read the provided `PROJECT_CONTEXT.md` to understand the architecture.

Our current state: We have just completed "Phase 5", offloading heavy SLAM and DWA algorithms to a Web Worker using `comlink`. The React UI is the Master node, and the ESP32 is the hardware slave.

Your next task:
1. Review `src/stores/robotStore.js` and `src/core/navWorker.js`.
2. Propose a plan to implement "Pure Localization Mode" (AMCL equivalent) where the robot uses a pre-saved map to localize itself without updating the map's obstacles.
3. Wait for my confirmation before editing code.
```
