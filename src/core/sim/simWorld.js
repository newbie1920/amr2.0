/**
 * GazeboTDTU — Simulated World Model
 * 
 * Quản lý "thế giới ảo": tường, kệ hàng, vật cản tĩnh/động.
 * Chuyển đổi tất cả geometry thành LINE SEGMENTS cho raycasting nhanh.
 * 
 * Hệ toạ độ: giống warehouse.js (mét, gốc ở góc dưới trái)
 *   +X = phải, +Y = lên
 */

import {
  WAREHOUSE_WIDTH, WAREHOUSE_HEIGHT,
  SHELVES, GATES, CHARGING_STATIONS,
  ROBOT_RADIUS,
} from '../warehouse.js';

// ============================================================
//   LINE SEGMENT — Đơn vị cơ bản cho raycasting
// ============================================================

/**
 * @typedef {Object} Segment
 * @property {number} x1
 * @property {number} y1
 * @property {number} x2
 * @property {number} y2
 * @property {string} [tag] - 'wall' | 'shelf' | 'obstacle' | 'charger'
 */

// ============================================================
//   SIM WORLD CLASS
// ============================================================

export class SimWorld {
  constructor() {
    /** @type {Segment[]} - Tất cả line segments (static + dynamic) */
    this.segments = [];

    /** @type {Segment[]} - Chỉ static segments (tường + kệ) */
    this.staticSegments = [];

    /** @type {Map<string, {bounds: {x1,y1,x2,y2}, segments: Segment[]}>} - Dynamic obstacles */
    this.dynamicObstacles = new Map();

    /** @type {{x: number, y: number, theta: number}} - Vị trí spawn mặc định */
    this.defaultSpawn = { x: 5.0, y: 1.5, theta: Math.PI / 2 };

    this._buildStaticWorld();
  }

  // ──────────────────────────────────────────────────────────
  //   BUILD STATIC WORLD (tường + kệ)
  // ──────────────────────────────────────────────────────────

  _buildStaticWorld() {
    this.staticSegments = [];

    // 1. Tường biên kho xưởng (4 cạnh)
    const W = WAREHOUSE_WIDTH;
    const H = WAREHOUSE_HEIGHT;

    // Tường dưới (y=0) — NHƯNG CÓ GAPS cho cổng
    const gateGaps = this._computeGateGaps();
    let prevX = 0;
    for (const gap of gateGaps) {
      if (gap.x1 > prevX) {
        this.staticSegments.push({ x1: prevX, y1: 0, x2: gap.x1, y2: 0, tag: 'wall' });
      }
      prevX = gap.x2;
    }
    if (prevX < W) {
      this.staticSegments.push({ x1: prevX, y1: 0, x2: W, y2: 0, tag: 'wall' });
    }

    // Tường trên (y=H)
    this.staticSegments.push({ x1: 0, y1: H, x2: W, y2: H, tag: 'wall' });
    // Tường trái (x=0)
    this.staticSegments.push({ x1: 0, y1: 0, x2: 0, y2: H, tag: 'wall' });
    // Tường phải (x=W)
    this.staticSegments.push({ x1: W, y1: 0, x2: W, y2: H, tag: 'wall' });

    // 2. Kệ hàng → 4 cạnh mỗi kệ
    for (const shelf of SHELVES) {
      const { x1, y1, x2, y2 } = shelf.bounds;
      this.staticSegments.push({ x1, y1, x2: x2, y2: y1, tag: 'shelf' }); // bottom
      this.staticSegments.push({ x1: x2, y1, x2: x2, y2, tag: 'shelf' }); // right
      this.staticSegments.push({ x1: x2, y1: y2, x2: x1, y2, tag: 'shelf' }); // top
      this.staticSegments.push({ x1, y1: y2, x2: x1, y2: y1, tag: 'shelf' }); // left
    }

    // 3. Trụ sạc → hình chữ U mỏng
    for (const charger of CHARGING_STATIONS) {
      const cx = charger.x, cy = charger.y;
      const hw = 0.5, hd = 0.6; // half-width, depth
      // Back wall
      this.staticSegments.push({ x1: cx - hw, y1: cy + hd, x2: cx + hw, y2: cy + hd, tag: 'charger' });
      // Left wall
      this.staticSegments.push({ x1: cx - hw, y1: cy, x2: cx - hw, y2: cy + hd, tag: 'charger' });
      // Right wall
      this.staticSegments.push({ x1: cx + hw, y1: cy, x2: cx + hw, y2: cy + hd, tag: 'charger' });
    }

    // Rebuild all segments
    this._rebuildSegments();
  }

  /**
   * Tính gaps trên tường dưới cho cổng nhập/xuất
   */
  _computeGateGaps() {
    const gaps = [];
    for (const gate of Object.values(GATES)) {
      if (gate.y <= 1.0) { // Cổng ở gần tường dưới
        gaps.push({ x1: gate.x - 0.7, x2: gate.x + 0.7 });
      }
    }
    gaps.sort((a, b) => a.x1 - b.x1);
    return gaps;
  }

  _rebuildSegments() {
    this.segments = [...this.staticSegments];
    for (const obs of this.dynamicObstacles.values()) {
      this.segments.push(...obs.segments);
    }
  }

  // ──────────────────────────────────────────────────────────
  //   DYNAMIC OBSTACLES API
  // ──────────────────────────────────────────────────────────

  /**
   * Thêm vật cản hình hộp
   * @param {string} id - ID duy nhất
   * @param {number} cx - Tâm X (mét)
   * @param {number} cy - Tâm Y (mét)
   * @param {number} w - Chiều rộng (mét)
   * @param {number} h - Chiều cao (mét)
   */
  addBoxObstacle(id, cx, cy, w, h) {
    const x1 = cx - w / 2, y1 = cy - h / 2;
    const x2 = cx + w / 2, y2 = cy + h / 2;
    const segments = [
      { x1, y1, x2: x2, y2: y1, tag: 'obstacle' },
      { x1: x2, y1, x2: x2, y2, tag: 'obstacle' },
      { x1: x2, y1: y2, x2: x1, y2, tag: 'obstacle' },
      { x1, y1: y2, x2: x1, y2: y1, tag: 'obstacle' },
    ];
    this.dynamicObstacles.set(id, { bounds: { x1, y1, x2, y2 }, segments });
    this._rebuildSegments();
  }

  /**
   * Thêm vật cản hình tròn (xấp xỉ bằng đa giác 8 cạnh)
   */
  addCircleObstacle(id, cx, cy, radius, sides = 8) {
    const segments = [];
    for (let i = 0; i < sides; i++) {
      const a1 = (2 * Math.PI * i) / sides;
      const a2 = (2 * Math.PI * (i + 1)) / sides;
      segments.push({
        x1: cx + radius * Math.cos(a1),
        y1: cy + radius * Math.sin(a1),
        x2: cx + radius * Math.cos(a2),
        y2: cy + radius * Math.sin(a2),
        tag: 'obstacle',
      });
    }
    const bounds = {
      x1: cx - radius, y1: cy - radius,
      x2: cx + radius, y2: cy + radius,
    };
    this.dynamicObstacles.set(id, { bounds, segments });
    this._rebuildSegments();
  }

  /**
   * Xoá vật cản
   */
  removeObstacle(id) {
    this.dynamicObstacles.delete(id);
    this._rebuildSegments();
  }

  /**
   * Di chuyển vật cản (dùng cho dynamic obstacles sau này)
   */
  moveObstacle(id, newCX, newCY) {
    const obs = this.dynamicObstacles.get(id);
    if (!obs) return;

    const oldCX = (obs.bounds.x1 + obs.bounds.x2) / 2;
    const oldCY = (obs.bounds.y1 + obs.bounds.y2) / 2;
    const dx = newCX - oldCX;
    const dy = newCY - oldCY;

    obs.bounds.x1 += dx; obs.bounds.y1 += dy;
    obs.bounds.x2 += dx; obs.bounds.y2 += dy;
    for (const seg of obs.segments) {
      seg.x1 += dx; seg.y1 += dy;
      seg.x2 += dx; seg.y2 += dy;
    }
    this._rebuildSegments();
  }

  // ──────────────────────────────────────────────────────────
  //   COLLISION CHECK
  // ──────────────────────────────────────────────────────────

  /**
   * Kiểm tra robot (hình tròn) có va chạm với bất kỳ segment nào không
   * @param {number} x - Tâm robot
   * @param {number} y - Tâm robot
   * @param {number} radius - Bán kính robot
   * @returns {boolean}
   */
  checkCollision(x, y, radius = ROBOT_RADIUS) {
    for (const seg of this.segments) {
      const dist = this._pointToSegmentDist(x, y, seg.x1, seg.y1, seg.x2, seg.y2);
      if (dist < radius) return true;
    }
    // Kiểm tra biên kho
    if (x - radius < 0 || x + radius > WAREHOUSE_WIDTH ||
        y - radius < 0 || y + radius > WAREHOUSE_HEIGHT) {
      return true;
    }
    return false;
  }

  /**
   * Khoảng cách từ điểm đến đoạn thẳng
   */
  _pointToSegmentDist(px, py, x1, y1, x2, y2) {
    const dx = x2 - x1, dy = y2 - y1;
    const lenSq = dx * dx + dy * dy;
    if (lenSq === 0) return Math.hypot(px - x1, py - y1);

    let t = ((px - x1) * dx + (py - y1) * dy) / lenSq;
    t = Math.max(0, Math.min(1, t));

    const nearX = x1 + t * dx;
    const nearY = y1 + t * dy;
    return Math.hypot(px - nearX, py - nearY);
  }

  // ──────────────────────────────────────────────────────────
  //   SERIALIZATION (cho Web Worker)
  // ──────────────────────────────────────────────────────────

  serialize() {
    return {
      segments: this.segments,
      defaultSpawn: { ...this.defaultSpawn },
      dynamicObstacleIds: [...this.dynamicObstacles.keys()],
    };
  }

  /**
   * Tạo lại từ serialized data (trong Worker)
   */
  static fromSerialized(data) {
    const world = new SimWorld();
    // Thêm segments từ data nếu có custom obstacles
    return world;
  }
}

export default SimWorld;
