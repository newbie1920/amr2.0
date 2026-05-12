/**
 * RVizTDTU — Canvas 2D Layer Rendering Functions
 * 
 * Mỗi hàm vẽ 1 layer lên canvas context.
 * Thứ tự vẽ: Grid → Map → Costmap → WorldSegments → Path → LaserScan → Robot → TF → Frontiers
 * 
 * Tất cả hàm nhận viewport = { worldToScreen, scale }
 * và vẽ trực tiếp lên ctx (CanvasRenderingContext2D).
 */

// ============================================================
//   COLOR SCHEME (RViz-inspired)
// ============================================================

const COLORS = {
  grid: { major: 'rgba(71, 85, 105, 0.3)', minor: 'rgba(51, 65, 85, 0.15)' },
  free: 'rgba(45, 55, 72, 0.6)',
  occupied: '#ef4444',
  unknown: 'rgba(30, 35, 45, 0.1)',
  laserHit: '#ef4444',
  laserRay: 'rgba(34, 197, 94, 0.08)',
  robot: '#22c55e',
  robotFill: 'rgba(34, 197, 94, 0.15)',
  path: '#3b82f6',
  pathDashed: [8, 4],
  frontier: '#06b6d4',
  wallSegment: 'rgba(255, 255, 255, 0.7)',
  tfX: '#ef4444',
  tfY: '#22c55e',
  costmapLow: 'rgba(59, 130, 246, 0.2)',
  costmapHigh: 'rgba(239, 68, 68, 0.5)',
  measure: '#f59e0b',
};

// ============================================================
//   1. GRID BACKGROUND
// ============================================================

export function drawGrid(ctx, w, h, viewport) {
  const { worldToScreen, scale } = viewport;

  // Determine grid spacing based on zoom
  const majorSpacing = scale > 40 ? 1.0 : scale > 15 ? 2.0 : 5.0; // meters
  const minorSpacing = majorSpacing / 5;

  // Find visible world bounds
  const topLeft = viewport.screenToWorld(0, 0);
  const bottomRight = viewport.screenToWorld(w, h);
  const minWX = Math.floor(topLeft.x / majorSpacing) * majorSpacing - majorSpacing;
  const maxWX = Math.ceil(bottomRight.x / majorSpacing) * majorSpacing + majorSpacing;
  const minWY = Math.floor(bottomRight.y / majorSpacing) * majorSpacing - majorSpacing;
  const maxWY = Math.ceil(topLeft.y / majorSpacing) * majorSpacing + majorSpacing;

  // Minor grid lines
  if (scale > 25) {
    ctx.strokeStyle = COLORS.grid.minor;
    ctx.lineWidth = 0.5;
    ctx.beginPath();
    for (let x = minWX; x <= maxWX; x += minorSpacing) {
      const s = worldToScreen(x, 0);
      ctx.moveTo(s.x, 0);
      ctx.lineTo(s.x, h);
    }
    for (let y = minWY; y <= maxWY; y += minorSpacing) {
      const s = worldToScreen(0, y);
      ctx.moveTo(0, s.y);
      ctx.lineTo(w, s.y);
    }
    ctx.stroke();
  }

  // Major grid lines
  ctx.strokeStyle = COLORS.grid.major;
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let x = minWX; x <= maxWX; x += majorSpacing) {
    const s = worldToScreen(x, 0);
    ctx.moveTo(s.x, 0);
    ctx.lineTo(s.x, h);
  }
  for (let y = minWY; y <= maxWY; y += majorSpacing) {
    const s = worldToScreen(0, y);
    ctx.moveTo(0, s.y);
    ctx.lineTo(w, s.y);
  }
  ctx.stroke();

  // Origin axes
  const origin = worldToScreen(0, 0);
  // X axis (red)
  ctx.strokeStyle = 'rgba(239, 68, 68, 0.4)';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(0, origin.y);
  ctx.lineTo(w, origin.y);
  ctx.stroke();
  // Y axis (green)
  ctx.strokeStyle = 'rgba(34, 197, 94, 0.4)';
  ctx.beginPath();
  ctx.moveTo(origin.x, 0);
  ctx.lineTo(origin.x, h);
  ctx.stroke();

  // Scale bar
  ctx.fillStyle = 'rgba(148, 163, 184, 0.6)';
  ctx.font = '10px Inter, monospace';
  ctx.textAlign = 'left';
  ctx.fillText(`${(1 / scale * 100).toFixed(0)}cm/px  Scale: ${scale.toFixed(0)} px/m`, 8, h - 8);
}

// ============================================================
//   2. OCCUPANCY MAP
// ============================================================

// Cache for occupancy map offscreen canvas
let _occMapCache = { scanCount: -1, width: 0, height: 0, canvas: null, lastUpdate: 0, originX: NaN, originY: NaN };

function _renderOccMapCanvas(grid) {
  const w = grid.width;
  const h = grid.height;
  const canvas = document.createElement('canvas');
  canvas.width = w;
  canvas.height = h;
  const ctx = canvas.getContext('2d');
  const imgData = ctx.createImageData(w, h);
  const data = imgData.data;

  for (let gy = 0; gy < h; gy++) {
    for (let gx = 0; gx < w; gx++) {
      const lo = grid.logOdds[gy * w + gx];
      if (lo > -0.3 && lo < 0.3) continue; // Unknown

      // Flip Y
      const canvasRow = h - 1 - gy;
      const pxIdx = (canvasRow * w + gx) * 4;

      if (lo > 0.5) {
        // Occupied — red with intensity
        const intensity = Math.min(1.0, (lo - 0.5) / 3.5);
        const alpha = Math.round((0.4 + 0.6 * intensity) * 255);
        data[pxIdx]     = 239;
        data[pxIdx + 1] = 68;
        data[pxIdx + 2] = 68;
        data[pxIdx + 3] = alpha;
      } else if (lo < -0.5) {
        // Free — dark gray
        data[pxIdx]     = 20;
        data[pxIdx + 1] = 24;
        data[pxIdx + 2] = 30;
        data[pxIdx + 3] = 120;
      }
    }
  }

  ctx.putImageData(imgData, 0, 0);
  return canvas;
}

export function drawOccupancyMap(ctx, w, h, viewport, grid) {
  if (!grid || !grid.logOdds) return;

  const { worldToScreen, scale } = viewport;
  const cellPx = grid.resolution * scale;
  if (cellPx < 0.5) return; // Too zoomed out

  // Cache: re-render when grid changes (including origin shifts from auto-expand)
  const sc = grid.scanCount || 0;
  const lu = grid.lastUpdate || 0;
  if (
    _occMapCache.scanCount !== sc ||
    _occMapCache.width !== grid.width ||
    _occMapCache.height !== grid.height ||
    _occMapCache.lastUpdate !== lu ||
    _occMapCache.originX !== grid.originX ||
    _occMapCache.originY !== grid.originY
  ) {
    _occMapCache.canvas = _renderOccMapCanvas(grid);
    _occMapCache.scanCount = sc;
    _occMapCache.width = grid.width;
    _occMapCache.height = grid.height;
    _occMapCache.lastUpdate = lu;
    _occMapCache.originX = grid.originX;
    _occMapCache.originY = grid.originY;
  }

  if (!_occMapCache.canvas) return;

  const topLeftWorld = {
    x: grid.originX,
    y: grid.originY + grid.height * grid.resolution,
  };
  const s = worldToScreen(topLeftWorld.x, topLeftWorld.y);
  const pixW = grid.width * grid.resolution * scale;
  const pixH = grid.height * grid.resolution * scale;

  const prevSmooth = ctx.imageSmoothingEnabled;
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(_occMapCache.canvas, s.x, s.y, pixW, pixH);
  ctx.imageSmoothingEnabled = prevSmooth;
}

// ============================================================
//   3. COSTMAP OVERLAY
// ============================================================

export function drawCostmap(ctx, w, h, viewport, grid) {
  if (!grid || !grid.logOdds) return;

  const { worldToScreen, scale } = viewport;
  const cellPx = grid.resolution * scale;
  if (cellPx < 2) return; // Skip at extreme zoom out

  // Simple inflation visualization
  for (let gy = 0; gy < grid.height; gy++) {
    for (let gx = 0; gx < grid.width; gx++) {
      const lo = grid.logOdds[gy * grid.width + gx];
      if (lo <= 0.3) continue; // Only near/occupied cells

      const worldX = grid.originX + gx * grid.resolution;
      const worldY = grid.originY + gy * grid.resolution;
      const s = worldToScreen(worldX, worldY + grid.resolution);
      
      const intensity = Math.min(1.0, lo / 5.0);
      const r = Math.round(59 + 180 * intensity);
      const g = Math.round(130 - 100 * intensity);
      const b = Math.round(246 - 178 * intensity);
      ctx.fillStyle = `rgba(${r}, ${g}, ${b}, ${0.15 + 0.35 * intensity})`;
      ctx.fillRect(s.x, s.y, cellPx + 0.5, cellPx + 0.5);
    }
  }
}

// ============================================================
//   4. WORLD SEGMENTS (Wall Outlines)
// ============================================================

export function drawWorldSegments(ctx, w, h, viewport, segments) {
  if (!segments || segments.length === 0) return;

  const { worldToScreen } = viewport;

  ctx.strokeStyle = COLORS.wallSegment;
  ctx.lineWidth = 1.5;
  ctx.beginPath();

  for (const seg of segments) {
    const s1 = worldToScreen(seg.x1, seg.y1);
    const s2 = worldToScreen(seg.x2, seg.y2);
    ctx.moveTo(s1.x, s1.y);
    ctx.lineTo(s2.x, s2.y);
  }

  ctx.stroke();
}

// ============================================================
//   5. NAVIGATION PATH
// ============================================================

export function drawPath(ctx, w, h, viewport, path) {
  if (!path || path.length < 2) return;

  const { worldToScreen } = viewport;

  // Path line
  ctx.strokeStyle = COLORS.path;
  ctx.lineWidth = 2;
  ctx.setLineDash(COLORS.pathDashed);
  ctx.beginPath();

  const first = worldToScreen(path[0].x, path[0].y);
  ctx.moveTo(first.x, first.y);

  for (let i = 1; i < path.length; i++) {
    const p = worldToScreen(path[i].x, path[i].y);
    ctx.lineTo(p.x, p.y);
  }
  ctx.stroke();
  ctx.setLineDash([]);

  // Goal marker (last point)
  const goal = worldToScreen(path[path.length - 1].x, path[path.length - 1].y);
  ctx.strokeStyle = COLORS.path;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(goal.x, goal.y, 8, 0, Math.PI * 2);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(goal.x - 4, goal.y);
  ctx.lineTo(goal.x + 4, goal.y);
  ctx.moveTo(goal.x, goal.y - 4);
  ctx.lineTo(goal.x, goal.y + 4);
  ctx.stroke();

  // Waypoint dots
  ctx.fillStyle = COLORS.path;
  for (let i = 0; i < path.length; i++) {
    const p = worldToScreen(path[i].x, path[i].y);
    ctx.beginPath();
    ctx.arc(p.x, p.y, 2.5, 0, Math.PI * 2);
    ctx.fill();
  }
}

// ============================================================
//   6. LASER SCAN POINTS
// ============================================================

export function drawLaserScan(ctx, w, h, viewport, robotX, robotY, robotTheta, lidarPts) {
  if (!lidarPts || lidarPts.length === 0) return;

  const { worldToScreen, scale } = viewport;
  const rScreen = worldToScreen(robotX, robotY);

  // 1. FoV Area (vùng rẻ quạt màu vàng nhạt)
  ctx.fillStyle = 'rgba(250, 204, 21, 0.08)'; // Vàng nhạt
  ctx.beginPath();
  ctx.moveTo(rScreen.x, rScreen.y);
  
  // Sắp xếp góc để vẽ polygon liên tục
  const sortedPts = [...lidarPts].sort((a, b) => a.a - b.a);
  for (const pt of sortedPts) {
    const distM = pt.d / 1000.0;
    if (distM < 0.05 || distM > 8.0) continue;

    const worldAngle = robotTheta + (pt.a * Math.PI) / 180;
    const hx = robotX + Math.cos(worldAngle) * distM;
    const hy = robotY + Math.sin(worldAngle) * distM;
    const hScreen = worldToScreen(hx, hy);
    ctx.lineTo(hScreen.x, hScreen.y);
  }
  ctx.closePath();
  ctx.fill();

  // 2. Laser rays (sparse — every 10th ray)
  ctx.strokeStyle = COLORS.laserRay;
  ctx.lineWidth = 0.5;
  ctx.beginPath();
  for (let i = 0; i < lidarPts.length; i += 10) {
    const pt = lidarPts[i];
    const distM = pt.d / 1000.0;
    if (distM < 0.05 || distM > 8.0) continue;

    const worldAngle = robotTheta + (pt.a * Math.PI) / 180;
    const hx = robotX + Math.cos(worldAngle) * distM;
    const hy = robotY + Math.sin(worldAngle) * distM;
    const hScreen = worldToScreen(hx, hy);

    ctx.moveTo(rScreen.x, rScreen.y);
    ctx.lineTo(hScreen.x, hScreen.y);
  }
  ctx.stroke();

  // Hit points (all)
  const dotSize = Math.max(1.5, Math.min(4, scale * 0.04));
  ctx.fillStyle = COLORS.laserHit;

  for (const pt of lidarPts) {
    const distM = pt.d / 1000.0;
    if (distM < 0.05 || distM > 8.0) continue;

    const worldAngle = robotTheta + (pt.a * Math.PI) / 180;
    const hx = robotX + Math.cos(worldAngle) * distM;
    const hy = robotY + Math.sin(worldAngle) * distM;
    const hScreen = worldToScreen(hx, hy);

    ctx.fillRect(hScreen.x - dotSize / 2, hScreen.y - dotSize / 2, dotSize, dotSize);
  }
}

// ============================================================
//   7. ROBOT POSE
// ============================================================

export function drawRobotPose(ctx, w, h, viewport, robotX, robotY, robotTheta, robotRadius = 0.12, linearVel = 0, odomTheta = null) {
  const { worldToScreen, scale } = viewport;
  const s = worldToScreen(robotX, robotY);
  const rPx = robotRadius * scale;

  // Robot circle (filled)
  ctx.beginPath();
  ctx.arc(s.x, s.y, rPx, 0, Math.PI * 2);
  ctx.fillStyle = COLORS.robotFill;
  ctx.fill();
  ctx.strokeStyle = COLORS.robot; // green viền robot
  ctx.lineWidth = 2;
  ctx.stroke();

  // Chiều dài mũi tên tỷ lệ thuận với vận tốc tức thời (tối thiểu = 1.8 * rPx)
  const speedScale = Math.max(0, Math.min(1.5, Math.abs(linearVel))); // Clamp để không dài quá
  const arrowLen = rPx * 1.8 + (speedScale * scale * 0.4);

  // 1. Mũi tên Odometry (xanh dương lớn)
  if (odomTheta !== null && odomTheta !== undefined) {
    const odomScreenAngle = -(odomTheta + Math.PI); // Y flipped + 180° heading correction
    const odomArrowX = s.x + Math.cos(odomScreenAngle) * arrowLen;
    const odomArrowY = s.y + Math.sin(odomScreenAngle) * arrowLen;

    ctx.strokeStyle = '#3b82f6'; // Xanh dương (blue-500)
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(s.x, s.y);
    ctx.lineTo(odomArrowX, odomArrowY);
    ctx.stroke();

    const headLen = 8;
    const headAngle = 0.5;
    ctx.beginPath();
    ctx.moveTo(odomArrowX, odomArrowY);
    ctx.lineTo(
      odomArrowX - headLen * Math.cos(odomScreenAngle - headAngle),
      odomArrowY - headLen * Math.sin(odomScreenAngle - headAngle)
    );
    ctx.moveTo(odomArrowX, odomArrowY);
    ctx.lineTo(
      odomArrowX - headLen * Math.cos(odomScreenAngle + headAngle),
      odomArrowY - headLen * Math.sin(odomScreenAngle + headAngle)
    );
    ctx.stroke();
  }

  // 2. Mũi tên Heading Corrected/Filtered (đỏ lớn)
  const screenAngle = -(robotTheta + Math.PI); // Y flipped + 180° heading correction
  const arrowX = s.x + Math.cos(screenAngle) * arrowLen;
  const arrowY = s.y + Math.sin(screenAngle) * arrowLen;

  ctx.strokeStyle = '#ef4444'; // Đỏ (red-500)
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(s.x, s.y);
  ctx.lineTo(arrowX, arrowY);
  ctx.stroke();

  // Arrowhead
  const headLen = 8;
  const headAngle = 0.5;
  ctx.beginPath();
  ctx.moveTo(arrowX, arrowY);
  ctx.lineTo(
    arrowX - headLen * Math.cos(screenAngle - headAngle),
    arrowY - headLen * Math.sin(screenAngle - headAngle)
  );
  ctx.moveTo(arrowX, arrowY);
  ctx.lineTo(
    arrowX - headLen * Math.cos(screenAngle + headAngle),
    arrowY - headLen * Math.sin(screenAngle + headAngle)
  );
  ctx.stroke();

  // Coordinate label
  ctx.fillStyle = 'rgba(148, 163, 184, 0.7)';
  ctx.font = '9px Inter, monospace';
  ctx.textAlign = 'center';
  ctx.fillText(
    `(${robotX.toFixed(2)}, ${robotY.toFixed(2)})`,
    s.x, s.y + rPx + 14
  );
}

// ============================================================
//   8. TF FRAMES (map → odom → base_link)
//   Giống RViz/ROS2: 3 coordinate frames với smooth connections
// ============================================================

export function drawTFFrames(ctx, w, h, viewport, odomPose, mapToOdom) {
  if (!odomPose) return;
  const tf = mapToOdom || { dx: 0, dy: 0, dTheta: 0 };

  const { worldToScreen, scale } = viewport;
  const axisLen = Math.max(15, Math.min(40, scale * 0.4));

  // ── Frame positions (world coords) ──────────────────────
  const mapFrame   = { x: 0, y: 0, theta: 0 };
  const odomFrame  = { x: tf.dx, y: tf.dy, theta: tf.dTheta };
  const cosO = Math.cos(odomFrame.theta);
  const sinO = Math.sin(odomFrame.theta);
  const baseFrame  = {
    x: odomFrame.x + cosO * odomPose.x - sinO * odomPose.y,
    y: odomFrame.y + sinO * odomPose.x + cosO * odomPose.y,
    theta: odomFrame.theta + odomPose.theta,
  };

  // ── Screen positions ────────────────────────────────────
  const mapS  = worldToScreen(mapFrame.x, mapFrame.y);
  const odomS = worldToScreen(odomFrame.x, odomFrame.y);
  const baseS = worldToScreen(baseFrame.x, baseFrame.y);

  // ── 1. Faint total chain: map → base_link (dashed blue) ─
  const dMB = Math.hypot(baseS.x - mapS.x, baseS.y - mapS.y);
  if (dMB > 5) {
    ctx.strokeStyle = 'rgba(96, 165, 250, 0.12)';
    ctx.lineWidth = 1;
    ctx.setLineDash([3, 4]);
    ctx.beginPath();
    ctx.moveTo(mapS.x, mapS.y);
    ctx.lineTo(baseS.x, baseS.y);
    ctx.stroke();
    ctx.setLineDash([]);
  }

  // ── 2. map → odom (yellow, SLAM correction) ────────────
  if (Math.hypot(odomS.x - mapS.x, odomS.y - mapS.y) > 3) {
    _tfCurve(ctx, mapS, odomS, '#f59e0b', 1.5);
  }

  // ── 3. odom → base_link (white, odometry, label Smooth) ─
  const dOB = Math.hypot(baseS.x - odomS.x, baseS.y - odomS.y);
  if (dOB > 5) {
    _tfCurve(ctx, odomS, baseS, 'rgba(255,255,255,0.7)', 1.5);
    // "Smooth" label at midpoint
    const mx = (odomS.x + baseS.x) / 2;
    const my = (odomS.y + baseS.y) / 2;
    ctx.fillStyle = 'rgba(255,255,255,0.55)';
    ctx.font = 'bold 11px Inter';
    ctx.textAlign = 'center';
    ctx.fillText('Smooth', mx + 10, my - 6);
  }

  // ── 4. Frame origin dots ────────────────────────────────
  _tfDot(ctx, mapS);
  _tfDot(ctx, odomS);
  _tfDot(ctx, baseS);

  // ── 5. Frame axes (X=red, Y=green) + labels ────────────
  _tfAxes(ctx, mapS,  0,                  axisLen, 'map',       '#f59e0b');
  _tfAxes(ctx, odomS, -odomFrame.theta,   axisLen, 'odom',      '#4ade80');
  _tfAxes(ctx, baseS, -baseFrame.theta,   axisLen, 'base_link', '#f1f5f9');
}

/* Smooth quadratic bezier arrow between two points */
function _tfCurve(ctx, from, to, color, lineW) {
  const dx = to.x - from.x;
  const dy = to.y - from.y;
  const dist = Math.hypot(dx, dy);
  if (dist < 2) return;

  const perpX = -dy / dist;
  const perpY =  dx / dist;
  const bulge = Math.min(dist * 0.18, 28);
  const cpx = (from.x + to.x) / 2 + perpX * bulge;
  const cpy = (from.y + to.y) / 2 + perpY * bulge;

  // curve
  ctx.strokeStyle = color;
  ctx.lineWidth = lineW;
  ctx.beginPath();
  ctx.moveTo(from.x, from.y);
  ctx.quadraticCurveTo(cpx, cpy, to.x, to.y);
  ctx.stroke();

  // arrowhead
  const t = 0.92;
  const ax = (1-t)*(1-t)*from.x + 2*(1-t)*t*cpx + t*t*to.x;
  const ay = (1-t)*(1-t)*from.y + 2*(1-t)*t*cpy + t*t*to.y;
  const ang = Math.atan2(to.y - ay, to.x - ax);
  const hl = 7;
  ctx.beginPath();
  ctx.moveTo(to.x, to.y);
  ctx.lineTo(to.x - hl*Math.cos(ang-0.45), to.y - hl*Math.sin(ang-0.45));
  ctx.moveTo(to.x, to.y);
  ctx.lineTo(to.x - hl*Math.cos(ang+0.45), to.y - hl*Math.sin(ang+0.45));
  ctx.stroke();
}

/* Black dot with subtle glow at frame origin */
function _tfDot(ctx, s) {
  ctx.fillStyle = 'rgba(0,0,0,0.9)';
  ctx.beginPath();
  ctx.arc(s.x, s.y, 4, 0, Math.PI * 2);
  ctx.fill();
  ctx.strokeStyle = 'rgba(255,255,255,0.3)';
  ctx.lineWidth = 1;
  ctx.stroke();
}

/* X (red) + Y (green) axes with label */
function _tfAxes(ctx, s, angle, len, label, labelColor) {
  // X axis — red
  ctx.strokeStyle = COLORS.tfX;
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  ctx.moveTo(s.x, s.y);
  ctx.lineTo(s.x + Math.cos(angle) * len, s.y + Math.sin(angle) * len);
  ctx.stroke();
  // X arrowhead
  const xTip = { x: s.x + Math.cos(angle) * len, y: s.y + Math.sin(angle) * len };
  _miniArrow(ctx, xTip, angle, COLORS.tfX);

  // Y axis — green (perpendicular, +90°)
  const yAng = angle - Math.PI / 2;
  ctx.strokeStyle = COLORS.tfY;
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  ctx.moveTo(s.x, s.y);
  ctx.lineTo(s.x + Math.cos(yAng) * len, s.y + Math.sin(yAng) * len);
  ctx.stroke();
  const yTip = { x: s.x + Math.cos(yAng) * len, y: s.y + Math.sin(yAng) * len };
  _miniArrow(ctx, yTip, yAng, COLORS.tfY);

  // Label
  ctx.fillStyle = labelColor;
  ctx.font = 'bold 10px Inter';
  ctx.textAlign = 'center';
  ctx.fillText(label, s.x, s.y - 10);
}

/* Tiny arrowhead at tip */
function _miniArrow(ctx, tip, angle, color) {
  const hl = 5;
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(tip.x, tip.y);
  ctx.lineTo(tip.x - hl*Math.cos(angle-0.5), tip.y - hl*Math.sin(angle-0.5));
  ctx.moveTo(tip.x, tip.y);
  ctx.lineTo(tip.x - hl*Math.cos(angle+0.5), tip.y - hl*Math.sin(angle+0.5));
  ctx.stroke();
}

// ============================================================
//   9. FRONTIER CELLS
// ============================================================

export function drawFrontiers(ctx, w, h, viewport, frontierCells, grid) {
  if (!frontierCells || frontierCells.length === 0 || !grid) return;

  const { worldToScreen, scale } = viewport;
  const dotSize = Math.max(2, Math.min(6, scale * 0.06));

  ctx.fillStyle = COLORS.frontier;

  for (const f of frontierCells) {
    const worldX = grid.originX + f.gx * grid.resolution;
    const worldY = grid.originY + f.gy * grid.resolution;
    const s = worldToScreen(worldX, worldY);
    ctx.fillRect(s.x - dotSize / 2, s.y - dotSize / 2, dotSize, dotSize);
  }
}

// ============================================================
//   MEASURE TOOL
// ============================================================

export function drawMeasureLine(ctx, viewport, p1, p2) {
  if (!p1 || !p2) return;

  const { worldToScreen } = viewport;
  const s1 = worldToScreen(p1.x, p1.y);
  const s2 = worldToScreen(p2.x, p2.y);

  // Line
  ctx.strokeStyle = COLORS.measure;
  ctx.lineWidth = 2;
  ctx.setLineDash([6, 3]);
  ctx.beginPath();
  ctx.moveTo(s1.x, s1.y);
  ctx.lineTo(s2.x, s2.y);
  ctx.stroke();
  ctx.setLineDash([]);

  // Endpoints
  ctx.fillStyle = COLORS.measure;
  ctx.beginPath();
  ctx.arc(s1.x, s1.y, 4, 0, Math.PI * 2);
  ctx.fill();
  ctx.beginPath();
  ctx.arc(s2.x, s2.y, 4, 0, Math.PI * 2);
  ctx.fill();

  // Distance label
  const dist = Math.hypot(p2.x - p1.x, p2.y - p1.y);
  const midX = (s1.x + s2.x) / 2;
  const midY = (s1.y + s2.y) / 2;
  ctx.fillStyle = '#f59e0b';
  ctx.font = 'bold 12px Inter, monospace';
  ctx.textAlign = 'center';
  ctx.fillText(`${(dist * 100).toFixed(1)} cm`, midX, midY - 8);
}

// ============================================================
//   NAV2-STYLE COSTMAP (Inflation Gradient)
//   Magenta-Cyan-Pink gradient giống RViz2 Nav2
//   Uses offscreen canvas caching for performance
// ============================================================

// Pre-compute Nav2-exact color LUT (RGBA bytes)
const _NAV2_COSTMAP_LUT = (() => {
  const lut = [];
  for (let i = 0; i < 256; i++) {
    if (i === 0) {
      lut.push(null); // Free — transparent
      continue;
    }
    let r, g, b, a;
    if (i >= 253) {
      // LETHAL — dark purple/black (obstacle cell itself)
      r = 100; g = 0; b = 50; a = 240;
    } else if (i >= 200) {
      // INSCRIBED — hot magenta/pink (robot WILL collide)
      const t = (i - 200) / 53;
      r = 255; g = Math.round(20 * (1 - t)); b = Math.round(100 + 60 * t); a = 210;
    } else if (i >= 128) {
      // HIGH INFLATION — magenta → deep pink
      const t = (i - 128) / 72;
      r = Math.round(180 + 75 * t); g = Math.round(30 * (1 - t)); b = Math.round(180 + 30 * t); a = Math.round(130 + 70 * t);
    } else if (i >= 50) {
      // MID INFLATION — cyan → magenta (the most visible gradient band)
      const t = (i - 50) / 78;
      r = Math.round(t * 180); g = Math.round(220 * (1 - t * 0.85)); b = Math.round(255 - 55 * t); a = Math.round(70 + 60 * t);
    } else {
      // LOW INFLATION — light cyan, semi-transparent
      const t = i / 50;
      r = 0; g = Math.round(180 + 40 * t); b = 255; a = Math.round(20 + 50 * t);
    }
    lut.push([r, g, b, a]);
  }
  return lut;
})();

// Cache key for offscreen canvas
let _costmapCache = { costmapVersion: -1, width: 0, height: 0, canvas: null, lastRenderTime: 0, originX: NaN, originY: NaN };

function _renderCostmapCanvas(grid) {
  const w = grid.width;
  const h = grid.height;
  const canvas = document.createElement('canvas');
  canvas.width = w;
  canvas.height = h;
  const ctx = canvas.getContext('2d');
  const imgData = ctx.createImageData(w, h);
  const data = imgData.data;

  for (let gy = 0; gy < h; gy++) {
    for (let gx = 0; gx < w; gx++) {
      const cost = grid.costmap[gy * w + gx];
      if (cost === 0) continue;

      const rgba = _NAV2_COSTMAP_LUT[Math.min(cost, 254)];
      if (!rgba) continue;

      // Flip Y: canvas row 0 is top, but grid row 0 is bottom
      const canvasRow = h - 1 - gy;
      const pxIdx = (canvasRow * w + gx) * 4;
      data[pxIdx]     = rgba[0];
      data[pxIdx + 1] = rgba[1];
      data[pxIdx + 2] = rgba[2];
      data[pxIdx + 3] = rgba[3];
    }
  }

  ctx.putImageData(imgData, 0, 0);
  return canvas;
}

export function drawCostmapNav2(ctx, w, h, viewport, grid) {
  if (!grid || !grid.costmap) return;

  const { worldToScreen, scale } = viewport;
  const cellPx = grid.resolution * scale;
  if (cellPx < 0.5) return; // Too zoomed out

  // Cache: re-render offscreen canvas when grid data changes
  // Invalidate on: costmapVersion change, grid resize, or origin shift (auto-expand)
  const cv = grid.costmapVersion || 0;
  const now = Date.now();
  const needsRedraw =
    _costmapCache.costmapVersion !== cv ||
    _costmapCache.width !== grid.width ||
    _costmapCache.height !== grid.height ||
    _costmapCache.originX !== grid.originX ||
    _costmapCache.originY !== grid.originY ||
    (grid._dirty && now - _costmapCache.lastRenderTime > 400);

  if (needsRedraw) {
    _costmapCache.canvas = _renderCostmapCanvas(grid);
    _costmapCache.costmapVersion = cv;
    _costmapCache.width = grid.width;
    _costmapCache.height = grid.height;
    _costmapCache.lastRenderTime = now;
    _costmapCache.originX = grid.originX;
    _costmapCache.originY = grid.originY;
  }

  if (!_costmapCache.canvas) return;

  // Draw cached texture scaled to viewport
  // Grid origin is bottom-left in world, canvas origin is top-left
  const topLeftWorld = {
    x: grid.originX,
    y: grid.originY + grid.height * grid.resolution,
  };
  const s = worldToScreen(topLeftWorld.x, topLeftWorld.y);
  const pixW = grid.width * grid.resolution * scale;
  const pixH = grid.height * grid.resolution * scale;

  // Use imageSmoothingEnabled=false for crisp pixels
  const prevSmooth = ctx.imageSmoothingEnabled;
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(_costmapCache.canvas, s.x, s.y, pixW, pixH);
  ctx.imageSmoothingEnabled = prevSmooth;
}

// ============================================================
//   GOAL MARKER (Nav2 Goal style)
// ============================================================

export function drawGoalMarker(ctx, viewport, goal) {
  if (!goal) return;

  const { worldToScreen } = viewport;
  const s = worldToScreen(goal.x, goal.y);
  const t = Date.now() / 1000;

  // Animated pulse ring
  const pulseR = 12 + 4 * Math.sin(t * 3);
  const pulseAlpha = 0.3 + 0.2 * Math.sin(t * 3);
  ctx.strokeStyle = `rgba(59, 130, 246, ${pulseAlpha})`;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(s.x, s.y, pulseR, 0, Math.PI * 2);
  ctx.stroke();

  // Outer ring
  ctx.strokeStyle = '#3b82f6';
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  ctx.arc(s.x, s.y, 10, 0, Math.PI * 2);
  ctx.stroke();

  // Inner filled dot
  ctx.fillStyle = 'rgba(59, 130, 246, 0.4)';
  ctx.beginPath();
  ctx.arc(s.x, s.y, 10, 0, Math.PI * 2);
  ctx.fill();

  // Crosshair
  ctx.strokeStyle = '#3b82f6';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(s.x - 15, s.y); ctx.lineTo(s.x - 5, s.y);
  ctx.moveTo(s.x + 5, s.y);  ctx.lineTo(s.x + 15, s.y);
  ctx.moveTo(s.x, s.y - 15); ctx.lineTo(s.x, s.y - 5);
  ctx.moveTo(s.x, s.y + 5);  ctx.lineTo(s.x, s.y + 15);
  ctx.stroke();

  // Label
  ctx.fillStyle = '#93c5fd';
  ctx.font = 'bold 10px Inter, monospace';
  ctx.textAlign = 'center';
  ctx.fillText(`Goal (${goal.x.toFixed(1)}, ${goal.y.toFixed(1)})`, s.x, s.y + 24);
}

// ============================================================
//   NAV STATUS OVERLAY (Top-right HUD)
// ============================================================

export function drawNavStatus(ctx, w, h, session) {
  if (!session) return;

  const STATUS_COLORS = {
    TRACK: '#22c55e',
    F_TURN: '#eab308',
    PAUSED: '#f97316',
    RECOVERY_SPIN: '#ef4444',
    RECOVERY_BACKUP: '#ef4444',
    RECOVERY_REPLAN: '#a855f7',
    DONE: '#3b82f6',
    ERROR: '#ef4444',
  };

  const color = STATUS_COLORS[session.status] || '#94a3b8';
  const wpIdx = session.currentWaypointIndex ?? 0;
  const wpTotal = session.path?.length ?? 0;
  const progress = wpTotal > 0 ? Math.round((wpIdx / wpTotal) * 100) : 0;

  // Background box
  const boxW = 180, boxH = 60;
  const boxX = w - boxW - 12, boxY = 12;

  ctx.fillStyle = 'rgba(15, 23, 42, 0.85)';
  ctx.strokeStyle = `${color}44`;
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.roundRect(boxX, boxY, boxW, boxH, 6);
  ctx.fill();
  ctx.stroke();

  // Status text
  ctx.fillStyle = color;
  ctx.font = 'bold 12px Inter';
  ctx.textAlign = 'left';
  ctx.fillText(`🧭 ${session.status}`, boxX + 10, boxY + 18);

  // Waypoint info
  ctx.fillStyle = '#94a3b8';
  ctx.font = '10px Inter, monospace';
  ctx.fillText(`WP: ${wpIdx}/${wpTotal}  (${progress}%)`, boxX + 10, boxY + 34);

  // Progress bar
  const barX = boxX + 10, barY = boxY + 42, barW = boxW - 20, barH = 6;
  ctx.fillStyle = 'rgba(255,255,255,0.1)';
  ctx.fillRect(barX, barY, barW, barH);
  ctx.fillStyle = color;
  ctx.fillRect(barX, barY, barW * (progress / 100), barH);
}

// ============================================================
//   ROBOT FOOTPRINT (Nav2 inscribed radius circle)
// ============================================================

export function drawRobotFootprint(ctx, viewport, robotX, robotY, robotRadius = 0.22) {
  const { worldToScreen, scale } = viewport;
  const s = worldToScreen(robotX, robotY);
  const rPx = robotRadius * scale;

  // Inscribed collision circle — dashed cyan
  ctx.strokeStyle = 'rgba(6, 182, 212, 0.5)';
  ctx.lineWidth = 1.5;
  ctx.setLineDash([4, 3]);
  ctx.beginPath();
  ctx.arc(s.x, s.y, rPx, 0, Math.PI * 2);
  ctx.stroke();
  ctx.setLineDash([]);

  // Fill with subtle cyan
  ctx.fillStyle = 'rgba(6, 182, 212, 0.06)';
  ctx.beginPath();
  ctx.arc(s.x, s.y, rPx, 0, Math.PI * 2);
  ctx.fill();

  // Preferred clearance ring — outer dashed yellow
  const clearancePx = 0.45 * scale; // preferredClearance
  ctx.strokeStyle = 'rgba(250, 204, 21, 0.2)';
  ctx.lineWidth = 1;
  ctx.setLineDash([2, 4]);
  ctx.beginPath();
  ctx.arc(s.x, s.y, clearancePx, 0, Math.PI * 2);
  ctx.stroke();
  ctx.setLineDash([]);
}

// ============================================================
//   LOCAL COSTMAP WINDOW (highlight around robot)
// ============================================================

export function drawLocalCostmapWindow(ctx, viewport, robotX, robotY, windowSize = 3.0) {
  const { worldToScreen, scale } = viewport;
  const half = windowSize / 2;
  const tl = worldToScreen(robotX - half, robotY + half);
  const br = worldToScreen(robotX + half, robotY - half);
  const w = br.x - tl.x;
  const h = br.y - tl.y;

  // Dashed cyan border
  ctx.strokeStyle = 'rgba(6, 182, 212, 0.35)';
  ctx.lineWidth = 1.5;
  ctx.setLineDash([6, 4]);
  ctx.beginPath();
  ctx.roundRect(tl.x, tl.y, w, h, 4);
  ctx.stroke();
  ctx.setLineDash([]);

  // Corner labels
  ctx.fillStyle = 'rgba(6, 182, 212, 0.4)';
  ctx.font = '8px Inter, monospace';
  ctx.textAlign = 'left';
  ctx.fillText('local_costmap', tl.x + 4, tl.y - 3);
}

// ============================================================
//   DWA TRAJECTORY PREVIEW (chosen path preview)
// ============================================================

export function drawDWATrajectory(ctx, viewport, trajectory) {
  if (!trajectory || trajectory.length < 2) return;

  const { worldToScreen } = viewport;
  const t = Date.now() / 1000;

  // Gradient from green (near) → cyan (far)
  const pts = trajectory.map(p => worldToScreen(p.x, p.y));

  // Glow effect
  ctx.strokeStyle = 'rgba(34, 197, 94, 0.15)';
  ctx.lineWidth = 8;
  ctx.lineCap = 'round';
  ctx.lineJoin = 'round';
  ctx.beginPath();
  ctx.moveTo(pts[0].x, pts[0].y);
  for (let i = 1; i < pts.length; i++) {
    ctx.lineTo(pts[i].x, pts[i].y);
  }
  ctx.stroke();

  // Main trajectory line — animated dash
  ctx.strokeStyle = '#22c55e';
  ctx.lineWidth = 2.5;
  ctx.setLineDash([6, 4]);
  ctx.lineDashOffset = -t * 30; // Animated marching ants
  ctx.beginPath();
  ctx.moveTo(pts[0].x, pts[0].y);
  for (let i = 1; i < pts.length; i++) {
    ctx.lineTo(pts[i].x, pts[i].y);
  }
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.lineDashOffset = 0;

  // Endpoint dot — where robot will be
  const last = pts[pts.length - 1];
  ctx.fillStyle = 'rgba(34, 197, 94, 0.7)';
  ctx.beginPath();
  ctx.arc(last.x, last.y, 4, 0, Math.PI * 2);
  ctx.fill();
}

// ============================================================
//   ANIMATED PATH (gradient + marching ants)
// ============================================================

export function drawAnimatedPath(ctx, w, h, viewport, path, robotX, robotY) {
  if (!path || path.length < 2) return;

  const { worldToScreen } = viewport;
  const t = Date.now() / 1000;

  // Convert to screen coords
  const pts = path.map(p => worldToScreen(p.x, p.y));

  // --- 1) Glow background ---
  ctx.strokeStyle = 'rgba(59, 130, 246, 0.1)';
  ctx.lineWidth = 10;
  ctx.lineCap = 'round';
  ctx.lineJoin = 'round';
  ctx.beginPath();
  ctx.moveTo(pts[0].x, pts[0].y);
  for (let i = 1; i < pts.length; i++) ctx.lineTo(pts[i].x, pts[i].y);
  ctx.stroke();

  // --- 2) Gradient path segments ---
  for (let i = 0; i < pts.length - 1; i++) {
    const progress = i / (pts.length - 1);
    // Green near robot → Blue → Purple at goal
    const r = Math.round(59 + 100 * progress);
    const g = Math.round(200 - 130 * progress);
    const b = Math.round(130 + 116 * progress);
    const alpha = 0.6 + 0.3 * Math.sin(t * 2 + i * 0.5);

    ctx.strokeStyle = `rgba(${r}, ${g}, ${b}, ${alpha})`;
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(pts[i].x, pts[i].y);
    ctx.lineTo(pts[i + 1].x, pts[i + 1].y);
    ctx.stroke();
  }

  // --- 3) Marching ants overlay ---
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.25)';
  ctx.lineWidth = 1.5;
  ctx.setLineDash([4, 8]);
  ctx.lineDashOffset = -t * 40;
  ctx.beginPath();
  ctx.moveTo(pts[0].x, pts[0].y);
  for (let i = 1; i < pts.length; i++) ctx.lineTo(pts[i].x, pts[i].y);
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.lineDashOffset = 0;

  // --- 4) Waypoint dots ---
  for (let i = 0; i < pts.length; i++) {
    const progress = i / (pts.length - 1);
    const dotR = 2 + progress * 2;
    ctx.fillStyle = `rgba(${Math.round(59 + 100 * progress)}, ${Math.round(200 - 130 * progress)}, ${Math.round(130 + 116 * progress)}, 0.8)`;
    ctx.beginPath();
    ctx.arc(pts[i].x, pts[i].y, dotR, 0, Math.PI * 2);
    ctx.fill();
  }
}

// ============================================================
//   GOAL POSE ARROW (Nav2-style large arrow at goal)
// ============================================================

export function drawGoalPoseArrow(ctx, viewport, goal, robotX, robotY) {
  if (!goal) return;

  const { worldToScreen, scale } = viewport;
  const s = worldToScreen(goal.x, goal.y);
  const t = Date.now() / 1000;

  // Compute heading from robot to goal for the arrow direction
  const heading = Math.atan2(goal.y - robotY, goal.x - robotX);
  const screenAngle = -heading; // Flip Y for screen

  // Animated pulse ring
  const pulseR = 14 + 5 * Math.sin(t * 2.5);
  ctx.strokeStyle = `rgba(168, 85, 247, ${0.2 + 0.15 * Math.sin(t * 2.5)})`;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(s.x, s.y, pulseR, 0, Math.PI * 2);
  ctx.stroke();

  // Outer ring — purple/magenta
  ctx.strokeStyle = '#a855f7';
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  ctx.arc(s.x, s.y, 12, 0, Math.PI * 2);
  ctx.stroke();

  // Inner fill
  ctx.fillStyle = 'rgba(168, 85, 247, 0.25)';
  ctx.beginPath();
  ctx.arc(s.x, s.y, 12, 0, Math.PI * 2);
  ctx.fill();

  // Large directional arrow
  const arrowLen = 24;
  const arrowTipX = s.x + Math.cos(screenAngle) * arrowLen;
  const arrowTipY = s.y + Math.sin(screenAngle) * arrowLen;

  ctx.strokeStyle = '#a855f7';
  ctx.lineWidth = 3;
  ctx.beginPath();
  ctx.moveTo(s.x, s.y);
  ctx.lineTo(arrowTipX, arrowTipY);
  ctx.stroke();

  // Arrowhead (filled triangle)
  const headLen = 10;
  const headAngle = 0.45;
  ctx.fillStyle = '#a855f7';
  ctx.beginPath();
  ctx.moveTo(arrowTipX, arrowTipY);
  ctx.lineTo(
    arrowTipX - headLen * Math.cos(screenAngle - headAngle),
    arrowTipY - headLen * Math.sin(screenAngle - headAngle)
  );
  ctx.lineTo(
    arrowTipX - headLen * Math.cos(screenAngle + headAngle),
    arrowTipY - headLen * Math.sin(screenAngle + headAngle)
  );
  ctx.closePath();
  ctx.fill();

  // Crosshair
  ctx.strokeStyle = 'rgba(168, 85, 247, 0.6)';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(s.x - 18, s.y); ctx.lineTo(s.x - 6, s.y);
  ctx.moveTo(s.x + 6, s.y);  ctx.lineTo(s.x + 18, s.y);
  ctx.moveTo(s.x, s.y - 18); ctx.lineTo(s.x, s.y - 6);
  ctx.moveTo(s.x, s.y + 6);  ctx.lineTo(s.x, s.y + 18);
  ctx.stroke();

  // Label
  ctx.fillStyle = '#c084fc';
  ctx.font = 'bold 10px Inter, monospace';
  ctx.textAlign = 'center';
  ctx.fillText(`🎯 Goal (${goal.x.toFixed(1)}, ${goal.y.toFixed(1)})`, s.x, s.y + 28);
}

// ============================================================
//   ROBOT TRAIL (breadcrumb history)
// ============================================================

export function drawRobotTrail(ctx, viewport, trail) {
  if (!trail || trail.length < 2) return;

  const { worldToScreen } = viewport;

  // Trail line — fading gradient
  for (let i = 1; i < trail.length; i++) {
    const progress = i / trail.length;
    const alpha = progress * 0.4; // Recent = brighter
    const s1 = worldToScreen(trail[i - 1].x, trail[i - 1].y);
    const s2 = worldToScreen(trail[i].x, trail[i].y);

    ctx.strokeStyle = `rgba(250, 204, 21, ${alpha})`;
    ctx.lineWidth = 1 + progress;
    ctx.beginPath();
    ctx.moveTo(s1.x, s1.y);
    ctx.lineTo(s2.x, s2.y);
    ctx.stroke();
  }

  // Trail dots — every 5th point
  for (let i = 0; i < trail.length; i += 5) {
    const progress = i / trail.length;
    const alpha = 0.1 + progress * 0.5;
    const s = worldToScreen(trail[i].x, trail[i].y);
    ctx.fillStyle = `rgba(250, 204, 21, ${alpha})`;
    ctx.beginPath();
    ctx.arc(s.x, s.y, 1.5, 0, Math.PI * 2);
    ctx.fill();
  }
}
