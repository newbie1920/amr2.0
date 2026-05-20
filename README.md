# AMR 2.0 - Vehicle-Brain Autonomous Mobile Robot

AMR 2.0 là dự án xe tự hành dùng ESP32-S3 làm bộ não vận hành chính. Firmware trên xe xử lý cảm biến, mapping/localization, path planning, trajectory tracking, PID và an toàn realtime. App React/Tauri là trạm điều khiển và giám sát: gửi mission/goal/config/manual command, hiển thị map/path/telemetry và hỗ trợ debug/demo.

Pipeline chuẩn:

```text
App gửi goal/config -> ESP32 xử lý localization/mapping/planning/tracking/PID -> xe chạy -> app xem telemetry/map/path/status
```

## Tính năng chính

1. **Robot firmware-first**
   - ESP32-S3 đọc Lidar, Encoder, IMU, pin/dòng.
   - Firmware giữ `RobotState`, odometry, occupancy grid, ICP/localization, navigation và PID.
   - Xe tự thực thi an toàn, app không phải bộ não điều hướng của robot thật.

2. **Điều hướng tự hành**
   - Firmware có các module pathfinder, navigator, DWA/frontier và PID bánh xe.
   - Hướng nghiên cứu tiếp theo: kinodynamic S-curve trajectory, feedback-linearized tracking và adaptive PID.

3. **App điều khiển/giám sát**
   - RViz/warehouse map để xem pose, Lidar, map, path, costmap và trạng thái.
   - Gửi `goal`, `navigate`, `cmd_vel`, `brake`, config/tuning và mission.
   - Browser-side planner/simulator chỉ dùng cho demo, debug và regression, không phải source of truth cho robot thật.

4. **WMS / Task UI**
   - Quản lý nhiệm vụ kho, trạng thái robot và dữ liệu hàng hóa.
   - Có thể kết nối Supabase cho dữ liệu realtime.

## Công nghệ

### Firmware xe

- ESP32-S3 N16R8
- C++ / Arduino / PlatformIO
- FreeRTOS tasks
- WebSocket, MQTT discovery, OTA
- Lidar A1M8, encoder, IMU MPU6050, INA3221/OLED

### App giám sát

- React 19, Vite, Tauri
- Zustand
- Three.js / React Three Fiber
- WebSocket telemetry and command UI

## Cấu trúc dự án

```text
AMR2.0/
├─ esp32s3xe_v2/                    # Não chính của xe
│  ├─ src/drivers/                  # Motor, encoder, lidar, IMU, battery, OLED
│  ├─ src/perception/               # Odometry, occupancy grid, ICP/SLAM
│  ├─ src/navigation/               # A*/Theta*, DWA, frontier, navigator
│  ├─ src/navigation/trajectory/    # Hướng S-curve / kinodynamic trajectory
│  ├─ src/control/                  # Hướng wheel PID / adaptive PID
│  └─ src/network/                  # WebSocket telemetry and commands
├─ src/                             # App điều khiển/giám sát
│  ├─ components/
│  ├─ stores/
│  ├─ core/protocol/
│  ├─ core/visualization/
│  └─ core/sim/
├─ docs/
│  ├─ 01_TDTU_Thesis/
│  ├─ 02_Architecture/
│  ├─ 03_Research/
│  ├─ 03_Development_Logs/
│  └─ knowledge/
├─ scripts/
└─ tests/
```

## Local Development

```bash
npm install
npm.cmd run dev
```

Firmware:

```bash
cd esp32s3xe_v2
pio run
```

## Tài liệu quan trọng

- `docs/02_Architecture/VEHICLE_BRAIN_ARCHITECTURE.md`
- `esp32s3xe_v2/ARCHITECTURE.md`
- `docs/03_Research/APPLICATION_TO_AMR2.md`
- `docs/01_TDTU_Thesis/PROJECT_REPORT.md`
