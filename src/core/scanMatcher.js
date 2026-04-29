/**
 * AMR 2.0 — Correlative Scan Matching (CSM) Module
 * 
 * Tham khảo: SLAM Toolbox (Karto SDK) + Olson (2009) Real-Time Correlative Scan Matching
 * 
 * Mục đích:
 *   Robot sử dụng encoder + IMU để tính vị trí (odometry), nhưng odometry bị DRIFT.
 *   Scan Matching so sánh scan hiện tại với map đã có → hiệu chỉnh vị trí chính xác hơn.
 *   
 * Pipeline (giống SLAM Toolbox):
 *   1. Nhận scan mới + odometry pose từ firmware
 *   2. Tạo "likelihood field" từ OccupancyGrid hiện tại
 *   3. Thử tất cả pose (x ± Δx, y ± Δy, θ ± Δθ) trong search window
 *   4. Chấm điểm mỗi pose = tổng likelihood tại vị trí các scan point
 *   5. Pose có điểm cao nhất = vị trí đã hiệu chỉnh
 *   6. Ghi đè robotX, robotY, robotTheta bằng pose đã hiệu chỉnh
 * 
 * Tối ưu cho JS/Browser:
 *   - Multi-resolution: coarse search trước (5cm), fine search sau (1cm)  
 *   - Giới hạn search window nhỏ (±0.3m, ±10°)
 *   - Skip scan matching nếu robot di chuyển < threshold
 */

// ============================================================
//   SCAN MATCHING CONFIG
// ============================================================

const CSM_CONFIG = {
  // Search window (tham khảo slam_toolbox: correlation_search_space_dimension: 0.5)
  searchWindowXY: 0.3,        // ±0.3m search range
  searchWindowTheta: 0.175,   // ±10° (0.175 rad) search range

  // Resolution (tham khảo slam_toolbox: correlation_search_space_resolution: 0.01)
  coarseStepXY: 0.05,         // 5cm step for coarse search
  coarseStepTheta: 0.035,     // ~2° step for coarse search
  fineStepXY: 0.01,           // 1cm step for fine search
  fineStepTheta: 0.005,       // ~0.3° step for fine search
  fineWindowXY: 0.08,         // ±8cm fine search window around coarse result
  fineWindowTheta: 0.05,      // ±3° fine search window

  // Likelihood field (tham khảo slam_toolbox: correlation_search_space_smear_deviation: 0.1)
  likelihoodSigma: 0.1,       // Gaussian smear σ for likelihood field (meters)
  likelihoodMaxDist: 0.5,     // Max distance for likelihood contribution (meters)

  // Thresholds
  minTravelDist: 0.15,        // Minimum travel distance before scan matching (tham khảo: minimum_travel_distance: 0.5, nhưng giảm cho robot nhỏ)
  minTravelHeading: 0.2,      // Minimum heading change before scan matching (rad ≈ 11°)
  minScanPoints: 30,          // Minimum valid points to perform matching
  minMatchScore: 0.3,         // Score threshold to accept match (0-1)
  maxCorrectionDist: 0.25,    // Maximum correction per match (safety)
  maxCorrectionAngle: 0.12,   // Maximum angle correction (≈7°)
};

// ============================================================
//   SCAN MATCHER CLASS
// ============================================================

export class ScanMatcher {
  constructor(config = {}) {
    this.config = { ...CSM_CONFIG, ...config };

    // Last matched pose
    this.lastMatchedX = 0;
    this.lastMatchedY = 0;
    this.lastMatchedTheta = 0;

    // Previous scan for reference
    this.prevScanPoints = null;

    // Likelihood field cache
    this._likelihoodField = null;
    this._likelihoodWidth = 0;
    this._likelihoodHeight = 0;

    // Statistics
    this.matchCount = 0;
    this.lastScore = 0;
    this.lastCorrection = { dx: 0, dy: 0, dTheta: 0 };
  }

  /**
   * Perform scan matching: tìm pose tốt nhất cho scan hiện tại trên grid
   * 
   * @param {OccupancyGrid} grid - Occupancy grid hiện tại
   * @param {number} odomX - Odometry X (meters)
   * @param {number} odomY - Odometry Y (meters)
   * @param {number} odomTheta - Odometry heading (radians)
   * @param {Array} scanPoints - Array of {a: angle_deg, d: distance_mm}
   * @returns {{ x, y, theta, score, corrected }} - Pose đã hiệu chỉnh
   */
  matchScan(grid, odomX, odomY, odomTheta, scanPoints) {
    const cfg = this.config;

    // Kiểm tra minimum travel distance
    const travelDist = Math.hypot(odomX - this.lastMatchedX, odomY - this.lastMatchedY);
    const travelHeading = Math.abs(_normalizeAngle(odomTheta - this.lastMatchedTheta));
    
    if (travelDist < cfg.minTravelDist && travelHeading < cfg.minTravelHeading) {
      return { x: odomX, y: odomY, theta: odomTheta, score: 0, corrected: false };
    }

    // Convert scan points to local Cartesian (robot frame)
    const localPoints = _scanToLocal(scanPoints, cfg.minScanPoints);
    if (!localPoints || localPoints.length < cfg.minScanPoints) {
      return { x: odomX, y: odomY, theta: odomTheta, score: 0, corrected: false };
    }

    // Cần ít nhất vài scan trước đó để có grid data
    if (grid.scanCount < 3) {
      this.lastMatchedX = odomX;
      this.lastMatchedY = odomY;
      this.lastMatchedTheta = odomTheta;
      return { x: odomX, y: odomY, theta: odomTheta, score: 0, corrected: false };
    }

    // Build likelihood field từ grid hiện tại
    this._buildLikelihoodField(grid);

    // === PHASE 1: Coarse Search ===
    const coarseResult = this._searchBestPose(
      grid, localPoints, odomX, odomY, odomTheta,
      cfg.searchWindowXY, cfg.searchWindowXY, cfg.searchWindowTheta,
      cfg.coarseStepXY, cfg.coarseStepTheta
    );

    // === PHASE 2: Fine Search (around coarse result) ===
    const fineResult = this._searchBestPose(
      grid, localPoints, coarseResult.x, coarseResult.y, coarseResult.theta,
      cfg.fineWindowXY, cfg.fineWindowXY, cfg.fineWindowTheta,
      cfg.fineStepXY, cfg.fineStepTheta
    );

    // Safety: giới hạn correction
    let corrX = fineResult.x - odomX;
    let corrY = fineResult.y - odomY;
    let corrTheta = _normalizeAngle(fineResult.theta - odomTheta);

    const corrDist = Math.hypot(corrX, corrY);
    if (corrDist > cfg.maxCorrectionDist) {
      const scale = cfg.maxCorrectionDist / corrDist;
      corrX *= scale;
      corrY *= scale;
    }
    if (Math.abs(corrTheta) > cfg.maxCorrectionAngle) {
      corrTheta = Math.sign(corrTheta) * cfg.maxCorrectionAngle;
    }

    const finalX = odomX + corrX;
    const finalY = odomY + corrY;
    const finalTheta = _normalizeAngle(odomTheta + corrTheta);
    const score = fineResult.score;

    // Accept match?
    const accepted = score > cfg.minMatchScore;

    if (accepted) {
      this.lastMatchedX = finalX;
      this.lastMatchedY = finalY;
      this.lastMatchedTheta = finalTheta;
      this.lastScore = score;
      this.lastCorrection = { dx: corrX, dy: corrY, dTheta: corrTheta };
      this.matchCount++;
    }

    return {
      x: accepted ? finalX : odomX,
      y: accepted ? finalY : odomY,
      theta: accepted ? finalTheta : odomTheta,
      score,
      corrected: accepted,
      correction: { dx: corrX, dy: corrY, dTheta: corrTheta },
    };
  }

  // ============================================================
  //   CORE: Brute-force search in (x, y, θ) space
  // ============================================================

  _searchBestPose(grid, localPoints, centerX, centerY, centerTheta, rangeXY, _rangeXY2, rangeTheta, stepXY, stepTheta) {
    let bestScore = -Infinity;
    let bestX = centerX;
    let bestY = centerY;
    let bestTheta = centerTheta;

    const res = grid.resolution;
    const w = grid.width, h = grid.height;

    for (let dTheta = -rangeTheta; dTheta <= rangeTheta; dTheta += stepTheta) {
      const theta = centerTheta + dTheta;
      const cosT = Math.cos(theta);
      const sinT = Math.sin(theta);

      for (let dx = -rangeXY; dx <= rangeXY; dx += stepXY) {
        for (let dy = -rangeXY; dy <= rangeXY; dy += stepXY) {
          const px = centerX + dx;
          const py = centerY + dy;

          let score = 0;
          let validCount = 0;

          for (let i = 0; i < localPoints.length; i++) {
            const lp = localPoints[i];
            // Transform local point → world
            const wx = px + lp.x * cosT - lp.y * sinT;
            const wy = py + lp.x * sinT + lp.y * cosT;

            // World → grid
            const gx = Math.floor((wx - grid.originX) / res);
            const gy = Math.floor((wy - grid.originY) / res);

            if (gx >= 0 && gx < w && gy >= 0 && gy < h) {
              score += this._likelihoodField[gy * w + gx];
              validCount++;
            }
          }

          // Normalize
          if (validCount > 0) {
            score /= validCount;
          }

          if (score > bestScore) {
            bestScore = score;
            bestX = px;
            bestY = py;
            bestTheta = theta;
          }
        }
      }
    }

    return { x: bestX, y: bestY, theta: bestTheta, score: bestScore };
  }

  // ============================================================
  //   LIKELIHOOD FIELD — Gaussian distance transform
  //   Tham khảo: slam_toolbox correlation_search_space_smear_deviation
  // ============================================================

  _buildLikelihoodField(grid) {
    const w = grid.width, h = grid.height;

    // Cache: Chỉ rebuild nếu grid size thay đổi hoặc scan mới
    if (this._likelihoodWidth === w && this._likelihoodHeight === h && this._likelihoodField) {
      // Chỉ update periodically thay vì mỗi scan
      if (this.matchCount % 3 !== 0) return;
    }

    this._likelihoodField = new Float32Array(w * h);
    this._likelihoodWidth = w;
    this._likelihoodHeight = h;

    const sigma = this.config.likelihoodSigma / grid.resolution; // Convert to cells
    const maxDistCells = Math.ceil(this.config.likelihoodMaxDist / grid.resolution);
    const sigma2 = 2 * sigma * sigma;

    // Bước 1: Tìm tất cả occupied cells
    const occupiedCells = [];
    for (let i = 0; i < grid.logOdds.length; i++) {
      if (grid.logOdds[i] > 0.5) {
        occupiedCells.push({ gx: i % w, gy: Math.floor(i / w) });
      }
    }

    // Bước 2: BFS distance transform từ occupied cells
    // (đơn giản hơn exact distance transform nhưng đủ tốt)
    const distMap = new Float32Array(w * h);
    distMap.fill(Infinity);
    const queue = [];

    for (const oc of occupiedCells) {
      const idx = oc.gy * w + oc.gx;
      distMap[idx] = 0;
      this._likelihoodField[idx] = 1.0; // Maximum likelihood at obstacle
      queue.push(idx);
    }

    // BFS
    let qi = 0;
    while (qi < queue.length) {
      const idx = queue[qi++];
      const gx = idx % w;
      const gy = Math.floor(idx / w);
      const currentDist = distMap[idx];

      if (currentDist >= maxDistCells) continue;

      for (let dy = -1; dy <= 1; dy++) {
        for (let dx = -1; dx <= 1; dx++) {
          if (dx === 0 && dy === 0) continue;
          const nx = gx + dx, ny = gy + dy;
          if (nx < 0 || nx >= w || ny < 0 || ny >= h) continue;
          const ni = ny * w + nx;
          const newDist = currentDist + (dx !== 0 && dy !== 0 ? 1.414 : 1.0);
          if (newDist < distMap[ni]) {
            distMap[ni] = newDist;
            // Gaussian falloff
            this._likelihoodField[ni] = Math.exp(-(newDist * newDist) / sigma2);
            queue.push(ni);
          }
        }
      }
    }
  }

  // ============================================================
  //   STATS
  // ============================================================

  getStats() {
    return {
      matchCount: this.matchCount,
      lastScore: this.lastScore,
      lastCorrection: this.lastCorrection,
    };
  }

  reset() {
    this.lastMatchedX = 0;
    this.lastMatchedY = 0;
    this.lastMatchedTheta = 0;
    this.prevScanPoints = null;
    this._likelihoodField = null;
    this.matchCount = 0;
    this.lastScore = 0;
    this.lastCorrection = { dx: 0, dy: 0, dTheta: 0 };
  }
}

// ============================================================
//   HELPERS
// ============================================================

function _scanToLocal(scanPoints, minPoints) {
  if (!scanPoints || scanPoints.length < minPoints) return null;

  const local = [];
  for (const pt of scanPoints) {
    const distM = pt.d / 1000.0;
    if (distM < 0.05 || distM > 3.0) continue;

    const rad = (pt.a * Math.PI) / 180.0;
    local.push({
      x: Math.cos(rad) * distM,
      y: Math.sin(rad) * distM,
    });
  }
  return local;
}

function _normalizeAngle(a) {
  while (a > Math.PI) a -= 2 * Math.PI;
  while (a < -Math.PI) a += 2 * Math.PI;
  return a;
}

export default ScanMatcher;
