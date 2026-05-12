# AMR 2.0 v2 — Firmware Architecture

> ESP32-S3 N16R8 | FreeRTOS | Layered Modular Design  
> Ported from monolithic v1 (963 LOC main.cpp) → modular v2 (~2500 LOC across 20+ files)

---

## 1. Overview

```
┌─────────────────────────────────────────────────────┐
│                    main.cpp (~130 LOC)               │
│  setup() → Layer 0 → Layer 1 → Layer 2 → Layer 4   │
│  loop()  → Network + Slow updates + Heartbeat       │
└──────┬──────────────────────────────────────────────┘
       │ creates
       ▼
┌──────────────────────────────────────────────────────┐
│                  FreeRTOS Tasks                       │
│  ┌─────────────┐  ┌───────────┐  ┌──────────────┐   │
│  │ controlTask │  │ lidarTask │  │ pathfinder/  │   │
│  │  Core 1     │  │  Core 0   │  │ exploration  │   │
│  │  50Hz       │  │  ~500Hz   │  │  Core 0      │   │
│  │  Priority 5 │  │  Prio 3   │  │  Prio 1-2    │   │
│  └─────────────┘  └───────────┘  └──────────────┘   │
└──────────────────────────────────────────────────────┘
```

## 2. Layer Architecture

| Layer | Components | Responsibility |
|-------|-----------|----------------|
| **Layer 0: Core** | `robot_state`, `tasks`, `config`, `log` | Central state, task creation, constants |
| **Layer 1: Drivers** | `motor_driver`, `encoder_driver`, `imu_mpu6050`, `lidar_a1m8`, `oled_display`, `ina3221_power`, `battery_adc` | Hardware abstraction |
| **Layer 2: Perception** | `odometry`, `occupancy_grid`, `icp_matcher` | Sensor fusion, mapping, SLAM |
| **Layer 3: Navigation** | `navigator`, `dwa_planner`, `pathfinder`, `frontier_explorer`, `pid_controller` | Path planning, obstacle avoidance |
| **Layer 4: Network** | `network_comm` (WiFi + WebSocket + MQTT + OTA) | Communication, telemetry, remote control |

## 3. Data Flow

```
                 ┌──────────────────────────┐
                 │    RobotState (Central)   │
                 │  • odom: x, y, theta     │
                 │  • map:  x, y, theta     │
                 │  • tf:   dx, dy, dTheta  │
                 │  • motor: ticks, vel, pwm│
                 │  • lidar: distances[360] │
                 │  • power: batt%, INA3221 │
                 │  • nav:   flags          │
                 │  • slam:  diagnostics    │
                 └──────┬───────────────────┘
                        │
          ┌─────────────┼──────────────┐
          │             │              │
    ┌─────▼──────┐ ┌───▼────┐  ┌──────▼──────┐
    │ controlTask│ │lidarTask│  │  loopTask   │
    │ Core 1     │ │ Core 0  │  │  Core 1     │
    │ 50Hz       │ │ ~500Hz  │  │  ~100Hz     │
    ├────────────┤ ├─────────┤  ├─────────────┤
    │ Encoder    │ │ LiDAR   │  │ WebSocket   │
    │ IMU Read   │ │ OccGrid │  │ MQTT        │
    │ Odometry   │ │ ICP     │  │ OTA         │
    │ PID Compute│ │ SLAM TF │  │ Telemetry   │
    │ Motor PWM  │ │         │  │ Battery     │
    │ Navigator  │ │         │  │ INA3221     │
    │ DWA        │ │         │  │ OLED        │
    └────────────┘ └─────────┘  └─────────────┘
```

## 4. Thread Safety Model

### RobotState Ownership

| Field | Writer | Reader | Protection |
|-------|--------|--------|-----------|
| `odom.*` | controlTask (Core 1) | lidarTask, loopTask | `portENTER_CRITICAL(&stateMux)` for cross-core reads |
| `map.*` | controlTask via `applyTf()` | All | `stateMux` critical section |
| `tf.*` | lidarTask via `updateTf()` | controlTask | `stateMux` critical section |
| `motor.leftTicks/rightTicks` | ISR (any core) | controlTask | `noInterrupts()/interrupts()` |
| `motor.targetLeftVel/Right` | loopTask (WebSocket cmd) | controlTask | Atomic on ESP32 (single-word float) |
| `lidar.distances[]` | lidarTask (Core 0) | loopTask (telemetry) | Eventually consistent (acceptable) |
| `power.*` | loopTask | loopTask (telemetry) | Same task (no contention) |

### Mutex Strategy
- **`stateMux`** (portMUX spinlock): For `odom`, `map`, `tf` cross-core access
- **`i2cMutex`** (FreeRTOS mutex): Protects shared I2C bus (MPU6050, OLED, INA3221)

## 5. SLAM Pipeline

```
LiDAR Point → gridMapper.add_point() → Buffer
                                          │
              every 200ms (>180 points)    │
                                          ▼
                    gridMapper.update_grid() ─→ Occupancy Grid (128x128)
                                          │
                              ICP Scan Matching
                                          │
                              ┌───────────▼───────────┐
                              │  Correction (dx,dy,dθ) │
                              │  Clamped ±5cm, ±0.08rad│
                              │  Weight = 0.4          │
                              └───────────┬───────────┘
                                          │
                              updateTf() ─→ state.tf.*
                                          │
                              applyTf()  ─→ state.map.* = odom ⊕ TF
```

## 6. Navigation Stack

```
WebSocket "goto" cmd
        │
        ▼
  GoToRequest → pathfinderQueue
        │
        ▼
  pathfinderTask (A* on occupancy grid)
        │
        ▼
  navigator.loadPath(waypoints)
        │
        ▼ (50Hz in controlTask)
  navigator.update() → Backstepping controller
        │                    │
        │              Recovery Behaviors:
        │              SPIN → BACKUP → WAIT
        │
        ▼ (10Hz in controlTask)
  dwaPlanner.computeVelocity() → obstacle avoidance
        │
        ▼
  (v, w) → wheel velocities → PID → Motor PWM
```

## 7. Network Protocol

### WebSocket Binary Frames (Port 81)
| Prefix | Direction | Content |
|--------|-----------|---------|
| `0x01` | ESP→Web | Occupancy grid (RLE compressed) |
| `0x02` | ESP→Web | Telemetry (MsgPack) |
| `0x03` | Web→ESP | Static map upload (chunked) |
| JSON | Web→ESP | Commands (cmd: "goto", "navigate", "brake", etc.) |

### MQTT Auto-Discovery (broker.hivemq.com:1883)
- **Topic:** `amr2/discovery/{ROBOT_ID}` (retained)
- **Payload:** `{id, ip, port, status, battery, firmware}`
- **Heartbeat:** Every 30s
- **LWT:** `{id, status: "offline"}` (auto-published on disconnect)

## 8. Pin Map

```
ESP32-S3 N16R8
├── GPIO 2  → Battery ADC (ADC1_CH1)
├── GPIO 4  → Encoder Left A
├── GPIO 5  → Encoder Left B
├── GPIO 6  → Encoder Right A
├── GPIO 7  → Encoder Right B
├── GPIO 8  → Motor Left EN (PWM)
├── GPIO 9  → Motor Left IN1
├── GPIO 10 → Motor Left IN2
├── GPIO 11 → Motor Right EN (PWM)
├── GPIO 12 → Motor Right IN3
├── GPIO 13 → Motor Right IN4
├── GPIO 15 → LiDAR Motor PWM
├── GPIO 18 → LiDAR RX (←Lidar TX)
├── GPIO 3  → LiDAR TX (→Lidar RX)
├── GPIO 39 → I2C SDA (MPU6050 + OLED + INA3221)
├── GPIO 40 → I2C SCL
└── GPIO 48 → NeoPixel RGB LED
```

## 9. Memory Budget

| Resource | Used | Available | Usage |
|----------|------|-----------|-------|
| RAM | 246 KB | 320 KB | 75.1% |
| Flash | 1.0 MB | 6.5 MB | 15.3% |
| PSRAM | ~0.5 MB | 8 MB | ~6% |
| FreeRTOS Tasks | 4 + loopTask | — | 5 total |

### Stack Allocation
| Task | Stack | Core | Priority |
|------|-------|------|----------|
| controlTask | 12 KB | 1 | 5 |
| lidarTask | 8 KB | 0 | 3 |
| pathfinderTask | 16 KB | 0 | 2 |
| explorationTask | 8 KB | 0 | 1 |

## 10. Known Limitations

1. **Battery ADC:** GPIO2 may need hardware validation; using `analogReadMilliVolts()` for calibrated readings.
2. **ICP Matching:** Only runs when robot is moving — prevents drift correction when stationary.
3. **Grid size:** Fixed 128×128 × 5cm = 6.4m × 6.4m coverage area.
4. **Single IMU axis:** Only gyro-Z used; no accelerometer fusion (adequate for flat surfaces).
5. **WiFi blocking:** Initial WiFi scan blocks loopTask for up to 15s (WDT temporarily removed during this period).
