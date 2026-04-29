# LIDAR-Based Occupancy Grid Mapping - Implementation Guide

## 📋 Overview

A complete LIDAR-based real-time mapping system has been implemented for AMR 2.0. This system uses the RPLIDAR A1M8 sensor to create a 2D occupancy grid of the environment in real-time.

**Status**: ✅ Fully Implemented and Integrated

---

## 🏗️ Architecture

### System Components

```
ESP32-S3 Firmware                Web UI (React)
├─ LIDAR Scanner               ├─ robotProtocol.js
│  └─ RPLidar Library          │  └─ Binary message handler
├─ lidar_mapper.h              ├─ lidarMapper.js
│  └─ OccupancyGridMapper      │  └─ OccupancyGrid parser
└─ WebSocket Transmitter       ├─ robotStore.js
   └─ Grid + Scan data         │  └─ Grid state management
                               └─ LidarGridVisualizer
                                  └─ 3D visualization
```

---

## 📝 Implementation Details

### 1. ESP32 Firmware Components

**File**: `esp32s3xe/include/lidar_mapper.h`

#### OccupancyGridMapper Class
- **Grid Size**: 40×40 cells = 1600 bytes
- **Resolution**: 0.25m per cell = 10m × 10m coverage
- **Occupancy Model**: Log-odds (int8_t range: -100 to +100)
  - Free: -10 per ray
  - Occupied: +50 per point
  - Unknown: 0 (default)
- **Algorithm**: Bresenham raycast for free-space marking

**Key Methods**:
```cpp
void update_pose(float x, float y, float heading);
void add_point(float angle_deg, float distance_m);
void update_grid();
void serialize_grid(uint8_t* buffer, int& len);
```

**Integration in main.cpp** (~line 470-490):
```cpp
// Global instance
OccupancyGridMapper gridMapper;
unsigned long lastGridUpdateTime = 0;
static const unsigned long GRID_UPDATE_INTERVAL = 200; // 5 Hz

// In lidarTask()
gridMapper.add_point(angle, distance / 1000.0f);
if (millis() - lastGridUpdateTime > GRID_UPDATE_INTERVAL && gridMapper.point_count > 180) {
  gridMapper.update_pose(robotX, robotY, robotTheta);
  gridMapper.update_grid();
  lastGridUpdateTime = millis();
}

// In main loop()
if (millis() - lastGridSendTime > 500) {
  lastGridSendTime = millis();
  send_occupancy_grid();
}
```

---

### 2. WebSocket Protocol

#### Binary Grid Message Format
```
Byte 0:       Message Type (0x01 = Occupancy Grid)
Byte 1:       Grid Width (40)
Byte 2:       Grid Height (40)
Bytes 3-6:    Cell Resolution (float, 0.25)
Bytes 7-10:   Robot X (float, world coords)
Bytes 11-14:  Robot Y (float, world coords)
Bytes 15-18:  Robot Heading (float, radians)
Bytes 19-1618: Grid Data (1600 bytes, occupancy 0-255)
────────────────────────────────────────────────
Total: 1619 bytes per transmission (≈3.2 KB/s per robot)
```

**Transmission Rate**: Every 500ms (2 Hz)

---

### 3. JavaScript/React Components

#### lidarMapper.js
Provides occupancy grid parsing and utilities:

```javascript
// Parse binary message from ESP32
const grid = OccupancyGrid.fromBinary(binaryBuffer);
console.log(grid.width, grid.height);  // 40, 40
console.log(grid.robotX, grid.robotY); // World position

// Get cell occupancy (0-255)
const occupancy = grid.getCell(gx, gy);
// 0-100: free (green)
// 100-150: unknown (gray)  
// 150-255: occupied (red)

// Get heatmap for visualization
const heatmap = grid.getHeatmap(); // Float32Array [0-1]

// Get obstacles for path planning
const obstacles = grid.getObstacles(); // {x, y, occupancy}[]
```

#### robotProtocol.js Updates
- Added binary message handler: `_handleBinaryMessage()`
- Added callback: `connection.onLidarGrid = (grid) => {...}`
- Automatically parses grid and triggers store update

#### robotStore.js Updates
```javascript
connection.onLidarGrid = (grid) => {
  set((state) => ({
    occupancyGrid: {
      ...state.occupancyGrid,
      [robotId]: grid,
    },
  }));
};
```

#### WarehouseMap.jsx Components
Three visualization modes:

```jsx
// 1. Grid Texture (efficient, shows occupancy heatmap)
<OccupancyGridVisualizer grid={grid} opacity={0.6} />

// 2. Cell Overlay (debug mode, shows individual cells)
<GridCellsOverlay grid={grid} cellOpacity={0.15} />

// 3. Obstacle Contours (edge detection for planning)
<ObstacleContours grid={grid} />
```

---

## 🚀 Testing & Usage

### On ESP32
1. Flash firmware to ESP32-S3
2. LIDAR task automatically starts reading
3. Grid updates and transmits in background (no manual action needed)

### In Web UI
1. Open WarehouseMap component
2. Click "🔴 Map Lidar (Point Cloud)" button
3. Switch view mode to see grid from different angles
4. Real-time updates as robot moves

### Validation Checks

**Check LIDAR data flowing**:
- Serial console should show points being read
- `lidarDists[]` array should populate (0-360 degrees)

**Check grid updates**:
- `gridMapper.point_count` should reach 180+ before update
- Every 200ms grid should be updated with robot pose

**Check WebSocket transmission**:
- Network tab should show binary messages
- 1619-byte packets every ~500ms

---

## 📊 Performance Metrics

| Metric | Value |
|--------|-------|
| Grid Resolution | 0.25m/cell |
| Grid Coverage | 10m × 10m |
| Update Frequency | 5 Hz (200ms) |
| Transmission Frequency | 2 Hz (500ms) |
| Memory per robot | ~2 KB (grid + buffers) |
| Bandwidth | ~3.2 KB/s (~25 Kbps) |
| Latency | <1 second real-time |

---

## 🔧 Configuration Options

**In `lidar_mapper.h`**:
```cpp
#define GRID_SIZE 40                    // Cells (40×40)
#define GRID_RESOLUTION 0.25f           // meters/cell
#define LOGODDS_OCC 50                  // Occupancy increment
#define LOGODDS_FREE -10                // Free-space decrement
#define LIDAR_MAX_RANGE 6.0f            // meters
```

**In main.cpp**:
```cpp
#define GRID_UPDATE_INTERVAL 200        // milliseconds (5 Hz)
// In loop: static const 500ms for transmission rate
```

---

## 🎨 Visualization Features

### Color Scheme
- **Green** (0-100): Free space for movement
- **Gray** (100-150): Unknown/unexplored
- **Red** (150-255): Obstacles/occupied

### Rendering Modes
1. **Heatmap** (efficient GPU texture)
   - Real-time, smooth visualization
   - Best for continuous monitoring

2. **Cell Grid** (debug overlay)
   - Shows individual cell boundaries
   - Helps understand grid resolution

3. **Contour Lines** (obstacle edges)
   - Highlights obstacle boundaries
   - Useful for collision detection

---

## 🔌 Hardware Requirements

**RPLIDAR A1M8**:
- UART: 115200 baud
- RX Pin: GPIO 47 (DATA from LIDAR)
- TX Pin: GPIO 46 (CONTROL to LIDAR)
- PWM Pin: GPIO 15 (Motor speed control)
- Rotation Rate: 5.5 Hz (330 RPM)

---

## 📈 Future Enhancements

### Short Term
- [ ] Occupancy grid persistence (save/load maps)
- [ ] Obstacle inflation for safety margins
- [ ] Grid-based A* pathfinding integration

### Medium Term
- [ ] Multi-robot collaborative SLAM
- [ ] Dynamic obstacle tracking
- [ ] Loop closure detection

### Long Term
- [ ] Full 3D voxel mapping
- [ ] Sensor fusion (camera + LIDAR)
- [ ] Real-time bag-of-words localization

---

## 🐛 Troubleshooting

### LIDAR not scanning
**Symptom**: No data in `lidarDists[]` array
- Check PWM signal on pin 15
- Verify UART connection (pins 46, 47)
- Look for LIDAR error in serial output

### Grid not updating
**Symptom**: `gridMapper.point_count` stays low
- May need to wait for 180+ points per cycle
- Check if robot is rotating (scanner needs motion)

### No WebSocket transmission
**Symptom**: Network doesn't show grid messages
- Verify WebSocket connection status
- Check for binary message handler registration
- Look for JavaScript console errors

### Visualization not showing
**Symptom**: "Map Lidar" button doesn't display grid
- Verify `occupancyGrid` state is populated
- Check if grid object is being created
- Look for React rendering errors

---

## 📚 Code References

| File | Purpose |
|------|---------|
| `esp32s3xe/include/lidar_mapper.h` | C++ grid mapper |
| `esp32s3xe/src/main.cpp` | ESP32 firmware integration |
| `src/core/lidarMapper.js` | JavaScript grid parser |
| `src/core/robotProtocol.js` | WebSocket handler |
| `src/stores/robotStore.js` | State management |
| `src/components/LidarGridVisualizer/` | 3D visualization |

---

## 🎓 Algorithm Details

### Occupancy Grid Mapping
Uses probabilistic log-odds representation:

```
log_odds = log(p / (1-p))

For each ray:
- Mark cells along ray as FREE (-10)
- Mark end cell as OCCUPIED (+50)
- Clamp to [-100, 100]

Probability = 1 / (1 + exp(-log_odds))
```

### Bresenham Raycast
Efficiently marks cells from robot to obstacle using integer-only arithmetic:

```
Marks all cells intersected by line from (x0,y0) to (x1,y1)
Time complexity: O(max(width, height))
```

---

## ✅ Implementation Checklist

- [x] C++ occupancy grid mapper (`lidar_mapper.h`)
- [x] ESP32 integration (LIDAR task + telemetry)
- [x] WebSocket binary message protocol
- [x] JavaScript grid parser (`lidarMapper.js`)
- [x] React store integration (`robotStore.js`)
- [x] 3D visualization components
- [x] WarehouseMap integration
- [x] Real-time updates (no manual refresh needed)
- [x] Performance optimization (5Hz update, 2Hz transmission)
- [x] Documentation

---

**Created**: April 2026 | **Status**: Production Ready
