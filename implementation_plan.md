# AMR 2.0 vs TurtleBot3 Nav2 — Gap Analysis & Roadmap

## 🎯 Mục tiêu: "Click đến đâu, chạy đến đó" + SLAM + Obstacle Avoidance

Bạn muốn đạt được khả năng như hình: Click 1 điểm trên bản đồ → Robot tự tìm đường đi → Tránh vật cản real-time → Hiển thị costmap/path trên map.

---

## So sánh tổng thể: AMR 2.0 vs TurtleBot3 Nav2

| Module | TurtleBot3 (ROS2 Nav2) | AMR 2.0 (Hiện tại) | Gap |
|--------|----------------------|-------------------|-----|
| **SLAM** | Cartographer / SLAM Toolbox | `lidarMapper.js` + `scanMatcher.js` (ICP) | ⚠️ Cơ bản — chỉ log-odds, chưa loop closure |
| **Global Planner** | NavFn (A*) trên global costmap | `lidarPathfinder.js` (A* trên raw grid) | ⚠️ Có A* nhưng chưa chạy trên **inflated costmap** |
| **Local Planner** | DWB (Dynamic Window) | `dwaPlanner.js` + `navWorker.js` | ✅ Đã có DWA với tuning presets |
| **Costmap** | 4 layers: static + obstacle + voxel + inflation | `lidarMapper.js` chỉ có 1 layer thô | ❌ **Thiếu Inflation Layer** — critical! |
| **Recovery** | Spin + BackUp + Wait | Chỉ có E-STOP (dừng khi thấy vật cản) | ❌ **Thiếu hoàn toàn** |
| **Localization** | AMCL (particle filter) | Encoder + Gyro fusion | ⚠️ OK cho khu vực nhỏ, drift theo thời gian |
| **Click-to-Navigate** | RViz2 → `Nav2 Goal` tool | Code có sẵn nhưng là **TODO** (line 215) | ❌ **Chưa implement** |
| **Visualization** | RViz2 (costmap, path, laser, TF) | `RVizPanel.jsx` (grid, laser, robot, path) | ⚠️ Có nhưng thiếu costmap gradient |

---

## Phân tích chi tiết từng Gap

### ❌ Gap 1: Click-to-Navigate (CRITICAL — dễ fix nhất)

Code đã có sẵn framework nhưng chỉ là `TODO`:
```javascript
// RVizPanel.jsx:213-215
if (activeTool === 'goal') {
  console.log(`[RVizTDTU] 🎯 Nav Goal: ...`);
  // TODO: Dispatch navigation goal to store  ← ĐÂY!
}
```

**Cần làm:**
1. Khi user click với tool `goal` → tính tọa độ world (x,y) ← đã có
2. Chạy A* pathfinder từ robot position → goal ← đã có `lidarPathfinder.js`
3. Gửi path (danh sách waypoints) xuống ESP32 qua WebSocket ← đã có `sendNavigate()`
4. ESP32 navigator chạy waypoint-following ← đã có `navigator.h`

**Effort: ~2 giờ** — Chỉ cần nối dây giữa các module đã có.

---

### ❌ Gap 2: Inflation Layer (CRITICAL — ảnh hưởng safety)

Trong hình TurtleBot3, bạn thấy các **gradient màu hồng-tím** xung quanh tường — đó là **Inflation Layer**. Nó tạo ra "vùng cấm mềm" xung quanh vật cản, giúp robot không đi quá sát tường.

**AMR 2.0 hiện tại:**
- `lidarMapper.js` chỉ có occupancy grid thô (0 = free, 100 = occupied)
- A* pathfinder chạy trên grid thô → path có thể đi sát tường → robot va chạm!

**TurtleBot3 Nav2:**
```yaml
inflation_layer:
  inflation_radius: 0.7   # 70cm buffer zone
  cost_scaling_factor: 8.0 # gradient decay
```

**Cần làm:**
1. Thêm `inflateGrid()` function trong `lidarMapper.js`
2. Thuật toán: BFS từ mỗi cell occupied → spread cost ra `inflation_radius` cells
3. Cost function: `cost = 254 * e^(-cost_scaling_factor * distance)`
4. A* pathfinder sử dụng inflated cost thay vì binary occupied/free

---

### ❌ Gap 3: Recovery Behaviors (IMPORTANT)

**TurtleBot3 Nav2:**
```yaml
recovery_plugins: ["spin", "backup", "wait"]
# Robot bị kẹt → xoay 360° → lùi 30cm → chờ 5s → thử lại
```

**AMR 2.0 hiện tại:**
- Chỉ có E-STOP: Phát hiện vật cản → dừng hoàn toàn → chờ vật cản biến mất
- Không có khả năng tự thoát kẹt

**Cần làm:**
1. Thêm state machine trong `navigator.h`: `NAV_RECOVERY_SPIN`, `NAV_RECOVERY_BACKUP`
2. Trigger khi robot stuck > 10 giây (progress checker)
3. Sequence: Spin 180° → BackUp 30cm → Replan path → Retry

---

### ⚠️ Gap 4: Global Planner chưa dùng Costmap

**Hiện tại:** `lidarPathfinder.js` chạy A* nhưng trên **raw occupancy grid**
**Cần:** A* chạy trên **inflated costmap** → đường đi tự nhiên cách xa tường

---

### ⚠️ Gap 5: AMCL Localization (NICE-TO-HAVE cho giai đoạn sau)

**TurtleBot3 Nav2:** Dùng AMCL (Adaptive Monte Carlo Localization) với 500-2000 particles
**AMR 2.0:** Encoder + Gyro complementary filter → drift tích lũy theo thời gian

Cho phạm vi luận văn, encoder + gyro fusion **đủ dùng** nếu khu vực hoạt động < 20m². AMCL là nice-to-have.

---

## Repo `noshluk2/ROS2-Autonomous-Driving-and-Navigation-SLAM-with-TurtleBot3` — Có gì hữu ích?

| File | Giá trị cho AMR 2.0 |
|------|---------------------|
| `tb3_nav_params.yaml` | ✅ **Rất hữu ích** — Copy các tham số DWB, inflation, recovery vào DWATuningPanel |
| `maze_solver.py` | ✅ **Tham khảo** — Logic giải maze dùng Nav2 Commander API, áp dụng cho TaskManager |
| `tb3_cartographer.lua` | ⚠️ Ít liên quan — config cho Google Cartographer (ROS2 only) |
| `hotel_map.pgm` | ⚠️ Sample map — dùng test pathfinder |

**Kết luận:** Repo này chủ yếu hữu ích ở **config parameters** (DWB tuning, costmap layers, recovery sequences). Logic code thì dùng hoàn toàn API của ROS2 nên không copy trực tiếp được — nhưng concept rất giá trị.

---

## 🚀 Lộ trình đề xuất (3 Phases)

### Phase A: Click-to-Navigate + Costmap Inflation (2-3 giờ)
> **Kết quả:** User click trên map → Robot tự chạy tới + tránh tường

#### [MODIFY] [RVizPanel.jsx](file:///c:/code2/AMR2.0/src/components/RVizPanel/RVizPanel.jsx)
- Implement `handleCanvasClick` khi tool = `goal`
- Gọi pathfinder → gửi navigate command xuống ESP32
- Vẽ goal marker (mũi tên xanh) trên canvas

#### [MODIFY] [lidarMapper.js](file:///c:/code2/AMR2.0/src/core/lidarMapper.js)
- Thêm `inflateGrid(grid, inflationRadius, costScaling)` 
- BFS inflation từ occupied cells
- Export inflated grid cho pathfinder sử dụng

#### [MODIFY] [lidarPathfinder.js](file:///c:/code2/AMR2.0/src/core/lidarPathfinder.js)
- Sử dụng inflated costmap thay vì raw grid
- Tính cost = grid cost + distance heuristic

---

### Phase B: Recovery Behaviors + Progress Checker (2-3 giờ)
> **Kết quả:** Robot tự thoát kẹt khi gặp ngõ cụt

#### [MODIFY] [navigator.h](file:///c:/code2/AMR2.0/esp32s3xe/include/navigator.h)
- Thêm states: `NAV_RECOVERY_SPIN`, `NAV_RECOVERY_BACKUP`, `NAV_RECOVERY_WAIT`
- Progress checker: Nếu robot di chuyển < 5cm trong 10 giây → trigger recovery
- Recovery sequence: Spin → Backup → Request replan

#### [MODIFY] [main.cpp](file:///c:/code2/AMR2.0/esp32s3xe/src/main.cpp)
- Thêm telemetry field `recovery_state` cho frontend hiển thị

---

### Phase C: Costmap Visualization trên RViz (1-2 giờ)
> **Kết quả:** Gradient hồng-tím-xanh như hình TurtleBot3

#### [MODIFY] [rvizLayers.js](file:///c:/code2/AMR2.0/src/components/RVizPanel/rvizLayers.js)
- Cập nhật `drawCostmap()` với color gradient giống Nav2 RViz:
  - Lethal (254) → Đỏ
  - Inscribed (253) → Hồng
  - Inflation (1-252) → Tím → Xanh nhạt
  - Free (0) → Trong suốt

---

## Verification Plan

### Automated Tests
- Build firmware: `pio run` ← kiểm tra navigator.h compile
- Build frontend: `npm run dev` ← kiểm tra React compile

### Manual Verification
1. Mở Dashboard → Click tool "Nav Goal" → Click 1 điểm trên map
2. Xác nhận robot nhận path và bắt đầu di chuyển
3. Đặt vật cản trước robot → Xác nhận robot dừng + thử recovery
4. Kiểm tra costmap gradient hiển thị đúng trên RViz panel

---

## Open Questions

> [!IMPORTANT]
> **Bạn muốn bắt đầu từ Phase nào?**
> - **Phase A** (Click-to-Navigate) cho kết quả visible nhanh nhất
> - **Phase B** (Recovery) cho robot thông minh hơn
> - Hay làm cả 3 phase liên tục?

> [!NOTE]
> **Về Simulation:** Hình TurtleBot3 dùng Gazebo (3D simulator). AMR 2.0 của bạn đã có `SimControlPanel` cho simulation 2D. Nếu cần demo cho luận văn, 2D simulator hiện tại + robot thật là đủ. Không cần Gazebo.
