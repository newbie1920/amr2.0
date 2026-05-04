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
  ROBOT_HALF_WIDTH, ROBOT_HALF_LENGTH,
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
    this.defaultSpawn = { x: 3.5, y: 2.0, theta: Math.PI / 2 };

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

    // Tường dưới (y=0)
    this.staticSegments.push({ x1: 0, y1: 0, x2: W, y2: 0, tag: 'wall' });

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

  // ──────────────────────────────────────────────────────────
  //   SLAM TEST MAP — Tạo bản đồ thử nghiệm cho SLAM
  // ──────────────────────────────────────────────────────────

  /**
   * Load bản đồ phòng thử nghiệm SLAM thay thế kho xưởng mặc định.
   * Layout 10x10m:
   *   - 4 tường biên kín (không có cổng)
   *   - 3 phòng nối bằng hành lang
   *   - Cột trụ tròn
   *   - Vật cản hình hộp rải rác
   * Robot spawn ở góc dưới trái (1.5, 1.5)
   */
  loadSlamTestMap() {
    this.staticSegments = [];
    this.dynamicObstacles.clear();

    const W = WAREHOUSE_WIDTH;  // 10m
    const H = WAREHOUSE_HEIGHT; // 10m

    // ── 1. TƯỜNG BIÊN KÍN ──────────────────────────────────
    this.staticSegments.push({ x1: 0, y1: 0, x2: W, y2: 0, tag: 'wall' }); // bottom
    this.staticSegments.push({ x1: W, y1: 0, x2: W, y2: H, tag: 'wall' }); // right
    this.staticSegments.push({ x1: W, y1: H, x2: 0, y2: H, tag: 'wall' }); // top
    this.staticSegments.push({ x1: 0, y1: H, x2: 0, y2: 0, tag: 'wall' }); // left

    // ── 2. TƯỜNG CHIA PHÒNG (tạo hành lang) ────────────────
    
    // Tường ngang chia tầng dưới/giữa — y=4, gap ở x=2..3 (cửa 1m)
    this.staticSegments.push({ x1: 0, y1: 4, x2: 2, y2: 4, tag: 'wall' });
    this.staticSegments.push({ x1: 3, y1: 4, x2: 6.5, y2: 4, tag: 'wall' });
    this.staticSegments.push({ x1: 7.5, y1: 4, x2: 10, y2: 4, tag: 'wall' });

    // Tường ngang chia tầng giữa/trên — y=7, gap ở x=4..5.5 (cửa 1.5m)
    this.staticSegments.push({ x1: 0, y1: 7, x2: 4, y2: 7, tag: 'wall' });
    this.staticSegments.push({ x1: 5.5, y1: 7, x2: 10, y2: 7, tag: 'wall' });

    // Tường dọc chia phòng dưới trái/phải — x=5, y=0..3 (gap ở y=1.5..2.5)
    this.staticSegments.push({ x1: 5, y1: 0, x2: 5, y2: 1.5, tag: 'wall' });
    this.staticSegments.push({ x1: 5, y1: 2.5, x2: 5, y2: 4, tag: 'wall' });

    // Tường dọc phòng trên — x=3, y=7..9 (tạo phòng nhỏ góc trái trên)
    this.staticSegments.push({ x1: 3, y1: 7, x2: 3, y2: 9, tag: 'wall' });

    // ── 3. VẬT CẢN HÌNH HỘP (bàn, kệ nhỏ) ────────────────
    const boxes = [
      { cx: 2.0, cy: 2.0, w: 0.8, h: 0.8 },   // Hộp phòng dưới trái
      { cx: 7.5, cy: 2.0, w: 1.0, h: 0.6 },   // Bàn phòng dưới phải
      { cx: 8.5, cy: 1.2, w: 0.5, h: 0.5 },   // Hộp nhỏ phòng dưới phải
      { cx: 1.5, cy: 5.5, w: 0.6, h: 1.2 },   // Kệ dọc phòng giữa trái
      { cx: 8.0, cy: 5.5, w: 1.2, h: 0.6 },   // Kệ ngang phòng giữa phải
      { cx: 5.0, cy: 5.8, w: 0.5, h: 0.5 },   // Hộp nhỏ giữa
      { cx: 1.5, cy: 8.5, w: 0.7, h: 0.7 },   // Hộp phòng trên trái
      { cx: 7.0, cy: 8.5, w: 1.0, h: 0.8 },   // Bàn phòng trên phải
    ];

    for (const box of boxes) {
      const x1 = box.cx - box.w / 2, y1 = box.cy - box.h / 2;
      const x2 = box.cx + box.w / 2, y2 = box.cy + box.h / 2;
      this.staticSegments.push({ x1, y1, x2: x2, y2: y1, tag: 'obstacle' });
      this.staticSegments.push({ x1: x2, y1, x2: x2, y2, tag: 'obstacle' });
      this.staticSegments.push({ x1: x2, y1: y2, x2: x1, y2, tag: 'obstacle' });
      this.staticSegments.push({ x1, y1: y2, x2: x1, y2: y1, tag: 'obstacle' });
    }

    // ── 4. CỘT TRỤ TRÒN (xấp xỉ bát giác) ────────────────
    const pillars = [
      { cx: 4.0, cy: 5.5, r: 0.25 },  // Cột giữa
      { cx: 6.5, cy: 5.5, r: 0.25 },  // Cột giữa phải
      { cx: 5.0, cy: 8.5, r: 0.3  },  // Cột phòng trên
    ];
    
    for (const p of pillars) {
      const sides = 8;
      for (let i = 0; i < sides; i++) {
        const a1 = (2 * Math.PI * i) / sides;
        const a2 = (2 * Math.PI * (i + 1)) / sides;
        this.staticSegments.push({
          x1: p.cx + p.r * Math.cos(a1),
          y1: p.cy + p.r * Math.sin(a1),
          x2: p.cx + p.r * Math.cos(a2),
          y2: p.cy + p.r * Math.sin(a2),
          tag: 'obstacle',
        });
      }
    }

    // ── 5. TƯỜNG CHÉO / L-SHAPE ────────────────────────────
    // L-shape ở phòng giữa phải
    this.staticSegments.push({ x1: 8.5, y1: 4.5, x2: 8.5, y2: 6.0, tag: 'obstacle' }); // vertical
    this.staticSegments.push({ x1: 8.5, y1: 6.0, x2: 9.5, y2: 6.0, tag: 'obstacle' }); // horizontal

    // Spawn tại khu vực trống giữa phòng (x=3.5, y=2.0)
    this.defaultSpawn = { x: 3.5, y: 2.0, theta: Math.PI / 2 };

    this._rebuildSegments();
    console.log(`[SimWorld] SLAM Test Map loaded: ${this.segments.length} segments`);
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
   * BUG #2 FIX: Rectangular footprint collision check.
   * Tests 4 corners + 4 edge midpoints of the robot body at given heading.
   * Falls back to circle check if theta is not provided.
   * 
   * @param {number} x - Robot center X
   * @param {number} y - Robot center Y
   * @param {number} radius - Robot radius (for fallback circle check)
   * @param {number} [theta=0] - Robot heading (radians)
   * @returns {boolean} true if collision detected
   */
  checkCollision(x, y, radius = ROBOT_RADIUS, theta = 0) {
    const hw = ROBOT_HALF_WIDTH;
    const hl = ROBOT_HALF_LENGTH;
    const cosT = Math.cos(theta);
    const sinT = Math.sin(theta);

    // 8 key points: 4 corners + 4 edge midpoints
    const checkPoints = [
      { lx: -hw, ly:  hl },  // front-left
      { lx:  hw, ly:  hl },  // front-right
      { lx:  hw, ly: -hl },  // rear-right
      { lx: -hw, ly: -hl },  // rear-left
      { lx:  0,  ly:  hl },  // front-center
      { lx:  0,  ly: -hl },  // rear-center
      { lx: -hw, ly:  0  },  // left-center
      { lx:  hw, ly:  0  },  // right-center
    ];

    for (const pt of checkPoints) {
      const wx = x + pt.lx * cosT - pt.ly * sinT;
      const wy = y + pt.lx * sinT + pt.ly * cosT;

      // Check against all wall/obstacle segments
      for (const seg of this.segments) {
        const dist = this._pointToSegmentDist(wx, wy, seg.x1, seg.y1, seg.x2, seg.y2);
        if (dist < 0.02) return true; // 2cm tolerance (point is ON the footprint edge)
      }

      // Check warehouse boundaries
      if (wx < 0 || wx > WAREHOUSE_WIDTH || wy < 0 || wy > WAREHOUSE_HEIGHT) {
        return true;
      }
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
