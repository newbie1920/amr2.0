/**
 * GazeboTDTU — Simulated Lidar (Raycasting Engine)
 * 
 * Mô phỏng cảm biến LD19 Lidar:
 *   - 360° scan, N points per revolution
 *   - Output: [{a: angle_deg, d: distance_mm}, ...]
 *   - Gaussian noise model
 * 
 * Thuật toán: Ray-Segment Intersection
 *   Cho mỗi tia (ray) từ vị trí robot, tìm giao điểm gần nhất
 *   với tất cả line segments trong SimWorld.
 */

// ============================================================
//   CONFIG — Tham số LD19 Lidar thật
// ============================================================

const DEFAULT_CONFIG = {
  numRays: 360,          // Số tia mỗi vòng quét (LD19 ~ 450 điểm)
  maxRange: 8000,        // Tầm xa tối đa (mm) — LD19: 12000mm
  minRange: 20,          // Tầm gần tối thiểu (mm)
  noiseStdDev: 5,        // Gaussian noise σ (mm) — LD19 ~ 5-10mm
  startAngle: 0,         // Góc bắt đầu (deg)
  angularRes: 1,         // Độ phân giải góc (deg) = 360/numRays
  failRate: 0.02,        // Tỉ lệ miss (tia không nhận) — ~2%
};

// ============================================================
//   SIM LIDAR CLASS
// ============================================================

export class SimLidar {
  /**
   * @param {Object} [config] - Override DEFAULT_CONFIG
   */
  constructor(config = {}) {
    this.config = { ...DEFAULT_CONFIG, ...config };
    this.config.angularRes = 360 / this.config.numRays;
  }

  /**
   * Thực hiện 1 vòng quét Lidar
   * 
   * @param {number} robotX - Vị trí robot X (mét)
   * @param {number} robotY - Vị trí robot Y (mét)
   * @param {number} robotTheta - Heading robot (rad, 0 = +X, CCW positive)
   * @param {import('./simWorld.js').Segment[]} segments - Các đoạn thẳng trong thế giới
   * @returns {{a: number, d: number}[]} - Mảng điểm Lidar giống ESP32 format
   */
  scan(robotX, robotY, robotTheta, segments) {
    const { numRays, maxRange, minRange, noiseStdDev, failRate } = this.config;
    const points = [];

    for (let i = 0; i < numRays; i++) {
      // Góc tia trong hệ toạ độ robot (deg)
      // LD19: 0° = phía trước robot, quay CW
      const localAngleDeg = i * this.config.angularRes;

      // Góc tia trong hệ toạ độ thế giới (rad)
      // Robot heading + local angle
      // Chuyển local angle sang rad, trừ vì LD19 quay CW
      const worldAngleRad = robotTheta + (localAngleDeg * Math.PI) / 180;

      // Direction vector
      const dx = Math.cos(worldAngleRad);
      const dy = Math.sin(worldAngleRad);

      // Tìm giao điểm gần nhất
      let minDist = maxRange; // mm

      for (const seg of segments) {
        const dist = this._raySegmentIntersect(
          robotX, robotY, dx, dy,
          seg.x1, seg.y1, seg.x2, seg.y2
        );
        if (dist !== null) {
          const distMM = dist * 1000; // mét → mm
          if (distMM >= minRange && distMM < minDist) {
            minDist = distMM;
          }
        }
      }

      // Simulate failures
      if (Math.random() < failRate) continue;

      // Noise
      if (minDist < maxRange) {
        minDist += this._gaussianNoise(noiseStdDev);
        minDist = Math.max(minRange, minDist);
      }

      points.push({
        a: Math.round(localAngleDeg),        // angle (degrees)
        d: Math.round(minDist),               // distance (mm)
      });
    }

    return points;
  }

  // ──────────────────────────────────────────────────────────
  //   RAY-SEGMENT INTERSECTION
  // ──────────────────────────────────────────────────────────

  /**
   * Tìm giao điểm giữa tia (ray) và đoạn thẳng (segment)
   * 
   * Ray:     P + t * D  (t >= 0)
   * Segment: A + s * (B - A)  (0 <= s <= 1)
   * 
   * @returns {number|null} Khoảng cách t (mét) hoặc null nếu không giao
   */
  _raySegmentIntersect(px, py, dx, dy, x1, y1, x2, y2) {
    const ex = x2 - x1;
    const ey = y2 - y1;

    const denom = dx * ey - dy * ex;
    if (Math.abs(denom) < 1e-10) return null; // Song song

    const tx = ((x1 - px) * ey - (y1 - py) * ex) / denom;
    const sx = ((x1 - px) * dy - (y1 - py) * dx) / denom;

    // t >= 0 (tia bắn về phía trước) và 0 <= s <= 1 (giao điểm trên segment)
    if (tx >= 0 && sx >= 0 && sx <= 1) {
      return tx;
    }

    return null;
  }

  // ──────────────────────────────────────────────────────────
  //   NOISE MODEL
  // ──────────────────────────────────────────────────────────

  /**
   * Box-Muller transform cho Gaussian noise
   */
  _gaussianNoise(stdDev) {
    const u1 = Math.random();
    const u2 = Math.random();
    return stdDev * Math.sqrt(-2 * Math.log(u1 || 1e-10)) * Math.cos(2 * Math.PI * u2);
  }
}

export default SimLidar;
