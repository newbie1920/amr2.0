/**
 * AMR 2.0 — Warehouse Data Model
 * Mô hình dữ liệu kho xưởng 10x10m
 */

// ============================================================
//   WAREHOUSE CONFIGURATION
// ============================================================

/** Kích thước kho xưởng (mét) */
export const WAREHOUSE_WIDTH = 10;
export const WAREHOUSE_HEIGHT = 10;

/** Grid resolution cho A* pathfinding (mét/cell) */
export const GRID_CELL_SIZE = 0.25;
export const GRID_COLS = Math.floor(WAREHOUSE_WIDTH / GRID_CELL_SIZE);   // 40
export const GRID_ROWS = Math.floor(WAREHOUSE_HEIGHT / GRID_CELL_SIZE);  // 40

/** Khoảng cách an toàn robot đối với obstacle (mét) */
export const ROBOT_RADIUS = 0.22; // Bán kính ngoại tiếp của hình vuông 30x30cm (~21.2cm) + margin nhẹ
export const SAFETY_MARGIN = 0.1;

/** Kích thước khung xe hình chữ nhật (mét) — dùng cho footprint collision check */
export const ROBOT_HALF_WIDTH = 0.15;   // Nửa chiều rộng = 30cm / 2
export const ROBOT_HALF_LENGTH = 0.15;  // Nửa chiều dài  = 30cm / 2

// ============================================================
//   VỊ TRÍ CÁC ĐỐI TƯỢNG TRONG KHO
// ============================================================

/**
 * Cổng nhập / xuất hàng
 * Vị trí ở cạnh dưới (y=0) của kho
 */
export const GATES = {
  import_1: { id: 'import_1', name: 'Cổng Nhập 1', x: 1.5, y: 0.5, heading: 90, type: 'import' },
  import_2: { id: 'import_2', name: 'Cổng Nhập 2', x: 3.0, y: 0.5, heading: 90, type: 'import' },
  import_3: { id: 'import_3', name: 'Cổng Nhập 3', x: 4.5, y: 0.5, heading: 90, type: 'import' },
  
  export_1: { id: 'export_1', name: 'Cổng Xuất 1', x: 6.0, y: 0.5, heading: 90, type: 'export' },
  export_2: { id: 'export_2', name: 'Cổng Xuất 2', x: 7.5, y: 0.5, heading: 90, type: 'export' },
  export_3: { id: 'export_3', name: 'Cổng Xuất 3', x: 9.0, y: 0.5, heading: 90, type: 'export' },

  error_1:  { id: 'error_1',  name: 'Cổng Hàng Lỗi', x: 0.5, y: 5.0, heading: 90, type: 'error' },
};

/**
 * Trụ sạc
 * Ở phía trên kho, robot tự động đi sạc khi pin < 20%
 */
export const CHARGING_STATIONS = [
  { id: 'charger_1', name: 'Trụ Sạc 1', x: 2.0, y: 9.0, heading: 270 },
  { id: 'charger_2', name: 'Trụ Sạc 2', x: 4.0, y: 9.0, heading: 270 },
  { id: 'charger_3', name: 'Trụ Sạc 3', x: 6.0, y: 9.0, heading: 270 },
  { id: 'charger_4', name: 'Trụ Sạc 4', x: 8.0, y: 9.0, heading: 270 },
];

/**
 * Kệ hàng — 2 kệ, mỗi kệ 2 tầng, mỗi tầng 3 ô
 * approach: vị trí robot dừng lại để gắp/đặt hàng
 * heading: góc robot cần xoay để đối mặt với kệ
 */
export const SHELVES = [
  // HÀNG DƯỚI (Gần Cổng)
  {
    id: 'shelf_1',
    name: 'Kệ 1',
    bounds: { x1: 1.5, y1: 3.0, x2: 3.5, y2: 4.5 },
    levels: [
      {
        level: 1, name: 'Tầng 1',
        slots: [
          { id: 's1_l1_1', name: 'Kệ 1: Tầng 1 - Ô 1', approach: { x: 2.0, y: 2.5 }, heading: 90, item: null },
          { id: 's1_l1_2', name: 'Kệ 1: Tầng 1 - Ô 2', approach: { x: 2.5, y: 2.5 }, heading: 90, item: null },
          { id: 's1_l1_3', name: 'Kệ 1: Tầng 1 - Ô 3', approach: { x: 3.0, y: 2.5 }, heading: 90, item: null },
        ],
      },
      {
        level: 2, name: 'Tầng 2',
        slots: [
          { id: 's1_l2_1', name: 'Kệ 1: Tầng 2 - Ô 1', approach: { x: 2.0, y: 2.5 }, heading: 90, item: null },
          { id: 's1_l2_2', name: 'Kệ 1: Tầng 2 - Ô 2', approach: { x: 2.5, y: 2.5 }, heading: 90, item: null },
          { id: 's1_l2_3', name: 'Kệ 1: Tầng 2 - Ô 3', approach: { x: 3.0, y: 2.5 }, heading: 90, item: null },
        ],
      },
    ],
  },
  {
    id: 'shelf_2',
    name: 'Kệ 2',
    bounds: { x1: 6.5, y1: 3.0, x2: 8.5, y2: 4.5 },
    levels: [
      {
        level: 1, name: 'Tầng 1',
        slots: [
          { id: 's2_l1_1', name: 'Kệ 2: Tầng 1 - Ô 1', approach: { x: 7.0, y: 2.5 }, heading: 90, item: null },
          { id: 's2_l1_2', name: 'Kệ 2: Tầng 1 - Ô 2', approach: { x: 7.5, y: 2.5 }, heading: 90, item: null },
          { id: 's2_l1_3', name: 'Kệ 2: Tầng 1 - Ô 3', approach: { x: 8.0, y: 2.5 }, heading: 90, item: null },
        ],
      },
      {
        level: 2, name: 'Tầng 2',
        slots: [
          { id: 's2_l2_1', name: 'Kệ 2: Tầng 2 - Ô 1', approach: { x: 7.0, y: 2.5 }, heading: 90, item: null },
          { id: 's2_l2_2', name: 'Kệ 2: Tầng 2 - Ô 2', approach: { x: 7.5, y: 2.5 }, heading: 90, item: null },
          { id: 's2_l2_3', name: 'Kệ 2: Tầng 2 - Ô 3', approach: { x: 8.0, y: 2.5 }, heading: 90, item: null },
        ],
      },
    ],
  },
  
  // HÀNG TRÊN (Gần Trụ Sạc)
  {
    id: 'shelf_3',
    name: 'Kệ 3',
    bounds: { x1: 1.5, y1: 6.0, x2: 3.5, y2: 7.5 },
    levels: [
      {
        level: 1, name: 'Tầng 1',
        slots: [
          { id: 's3_l1_1', name: 'Kệ 3: Tầng 1 - Ô 1', approach: { x: 2.0, y: 5.5 }, heading: 90, item: null },
          { id: 's3_l1_2', name: 'Kệ 3: Tầng 1 - Ô 2', approach: { x: 2.5, y: 5.5 }, heading: 90, item: null },
          { id: 's3_l1_3', name: 'Kệ 3: Tầng 1 - Ô 3', approach: { x: 3.0, y: 5.5 }, heading: 90, item: null },
        ],
      },
      {
        level: 2, name: 'Tầng 2',
        slots: [
          { id: 's3_l2_1', name: 'Kệ 3: Tầng 2 - Ô 1', approach: { x: 2.0, y: 5.5 }, heading: 90, item: null },
          { id: 's3_l2_2', name: 'Kệ 3: Tầng 2 - Ô 2', approach: { x: 2.5, y: 5.5 }, heading: 90, item: null },
          { id: 's3_l2_3', name: 'Kệ 3: Tầng 2 - Ô 3', approach: { x: 3.0, y: 5.5 }, heading: 90, item: null },
        ],
      },
    ],
  },
  {
    id: 'shelf_4',
    name: 'Kệ 4',
    bounds: { x1: 6.5, y1: 6.0, x2: 8.5, y2: 7.5 },
    levels: [
      {
        level: 1, name: 'Tầng 1',
        slots: [
          { id: 's4_l1_1', name: 'Kệ 4: Tầng 1 - Ô 1', approach: { x: 7.0, y: 5.5 }, heading: 90, item: null },
          { id: 's4_l1_2', name: 'Kệ 4: Tầng 1 - Ô 2', approach: { x: 7.5, y: 5.5 }, heading: 90, item: null },
          { id: 's4_l1_3', name: 'Kệ 4: Tầng 1 - Ô 3', approach: { x: 8.0, y: 5.5 }, heading: 90, item: null },
        ],
      },
      {
        level: 2, name: 'Tầng 2',
        slots: [
          { id: 's4_l2_1', name: 'Kệ 4: Tầng 2 - Ô 1', approach: { x: 7.0, y: 5.5 }, heading: 90, item: null },
          { id: 's4_l2_2', name: 'Kệ 4: Tầng 2 - Ô 2', approach: { x: 7.5, y: 5.5 }, heading: 90, item: null },
          { id: 's4_l2_3', name: 'Kệ 4: Tầng 2 - Ô 3', approach: { x: 8.0, y: 5.5 }, heading: 90, item: null },
        ],
      },
    ],
  },
];

// ============================================================
//   OBSTACLE GRID GENERATION
// ============================================================

/**
 * Tạo occupancy grid cho A* pathfinding
 * 0 = free, 1 = obstacle
 * @returns {number[][]} grid[row][col]
 */
export function createOccupancyGrid() {
  const grid = [];
  for (let r = 0; r < GRID_ROWS; r++) {
    grid[r] = new Array(GRID_COLS).fill(0);
  }

  // Ánh vùng chặn (kệ hàng) lên grid
  const inflate = ROBOT_RADIUS + SAFETY_MARGIN; // Phình ra cho robot
  for (const shelf of SHELVES) {
    const { x1, y1, x2, y2 } = shelf.bounds;
    const c1 = Math.max(0, Math.floor((x1 - inflate) / GRID_CELL_SIZE));
    const c2 = Math.min(GRID_COLS - 1, Math.ceil((x2 + inflate) / GRID_CELL_SIZE));
    const r1 = Math.max(0, Math.floor((y1 - inflate) / GRID_CELL_SIZE));
    const r2 = Math.min(GRID_ROWS - 1, Math.ceil((y2 + inflate) / GRID_CELL_SIZE));
    for (let r = r1; r <= r2; r++) {
      for (let c = c1; c <= c2; c++) {
        grid[r][c] = 1;
      }
    }
  }

  // Tường biên (4 cạnh), trừ cổng nhập/xuất
  for (let c = 0; c < GRID_COLS; c++) {
    // Tường trên
    grid[GRID_ROWS - 1][c] = 1;
    // Tường dưới (trừ cổng)
    const xMeter = c * GRID_CELL_SIZE;
    const isImportGate = xMeter >= 1.0 && xMeter <= 3.0;
    const isExportGate = xMeter >= 7.0 && xMeter <= 9.0;
    if (!isImportGate && !isExportGate) {
      grid[0][c] = 1;
    }
  }
  for (let r = 0; r < GRID_ROWS; r++) {
    grid[r][0] = 1;                // Tường trái
    grid[r][GRID_COLS - 1] = 1;    // Tường phải
  }

  return grid;
}

// ============================================================
//   HELPER FUNCTIONS
// ============================================================

/**
 * Chuyển đổi tọa độ thực (mét) → grid cell
 */
export function meterToGrid(x, y) {
  return {
    col: Math.floor(x / GRID_CELL_SIZE),
    row: Math.floor(y / GRID_CELL_SIZE),
  };
}

/**
 * Chuyển đổi grid cell → tọa độ thực (trung tâm cell)
 */
export function gridToMeter(col, row) {
  return {
    x: (col + 0.5) * GRID_CELL_SIZE,
    y: (row + 0.5) * GRID_CELL_SIZE,
  };
}

/**
 * Tính khoảng cách Euclidean giữa 2 điểm
 */
export function distance(x1, y1, x2, y2) {
  return Math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2);
}

/**
 * Chuẩn hóa góc về [-PI, PI]
 */
export function normalizeAngle(angle) {
  while (angle > Math.PI) angle -= 2 * Math.PI;
  while (angle < -Math.PI) angle += 2 * Math.PI;
  return angle;
}

/**
 * Chuyển độ → radian
 */
export function degToRad(deg) {
  return (deg * Math.PI) / 180;
}

/**
 * Chuyển radian → độ
 */
export function radToDeg(rad) {
  return (rad * 180) / Math.PI;
}

/**
 * Tìm slot kệ theo ID
 */
export function findSlotById(slotId) {
  for (const shelf of SHELVES) {
    for (const level of shelf.levels) {
      for (const slot of level.slots) {
        if (slot.id === slotId) {
          return { shelf, level, slot };
        }
      }
    }
  }
  return null;
}

/**
 * Tìm trụ sạc gần nhất với robot
 */
export function findNearestCharger(robotX, robotY) {
  let nearest = null;
  let minDist = Infinity;
  for (const charger of CHARGING_STATIONS) {
    const d = distance(robotX, robotY, charger.x, charger.y);
    if (d < minDist) {
      minDist = d;
      nearest = charger;
    }
  }
  return nearest;
}
