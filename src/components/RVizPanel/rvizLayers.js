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

export function drawOccupancyMap(ctx, w, h, viewport, grid) {
  if (!grid || !grid.logOdds) return;

  const { worldToScreen, scale } = viewport;
  const cellPx = grid.resolution * scale;
  
  // Skip if cells too small to see
  if (cellPx < 1) return;

  for (let gy = 0; gy < grid.height; gy++) {
    for (let gx = 0; gx < grid.width; gx++) {
      const lo = grid.logOdds[gy * grid.width + gx];
      
      // Skip unknown cells
      if (lo > -0.3 && lo < 0.3) continue;

      const worldX = grid.originX + gx * grid.resolution;
      const worldY = grid.originY + gy * grid.resolution;
      const s = worldToScreen(worldX, worldY + grid.resolution);

      if (lo > 0.5) {
        // Occupied — red with intensity
        const intensity = Math.min(1.0, (lo - 0.5) / 3.5);
        ctx.fillStyle = `rgba(239, 68, 68, ${0.4 + 0.6 * intensity})`;
      } else if (lo < -0.5) {
        // Free — dark gray
        ctx.fillStyle = COLORS.free;
      } else {
        continue;
      }

      ctx.fillRect(s.x, s.y, cellPx + 0.5, cellPx + 0.5);
    }
  }
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

  // Laser rays (sparse — every 10th ray)
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

export function drawRobotPose(ctx, w, h, viewport, robotX, robotY, robotTheta, robotRadius = 0.12) {
  const { worldToScreen, scale } = viewport;
  const s = worldToScreen(robotX, robotY);
  const rPx = robotRadius * scale;

  // Robot circle (filled)
  ctx.beginPath();
  ctx.arc(s.x, s.y, rPx, 0, Math.PI * 2);
  ctx.fillStyle = COLORS.robotFill;
  ctx.fill();
  ctx.strokeStyle = COLORS.robot;
  ctx.lineWidth = 2;
  ctx.stroke();

  // Heading arrow
  // In screen space: world theta → screen angle (flip Y)
  const screenAngle = -robotTheta; // Y is flipped
  const arrowLen = rPx * 1.8;
  const arrowX = s.x + Math.cos(screenAngle) * arrowLen;
  const arrowY = s.y + Math.sin(screenAngle) * arrowLen;

  ctx.strokeStyle = COLORS.robot;
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  ctx.moveTo(s.x, s.y);
  ctx.lineTo(arrowX, arrowY);
  ctx.stroke();

  // Arrowhead
  const headLen = 6;
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
    const worldX = grid.originX + (f.gx + 0.5) * grid.resolution;
    const worldY = grid.originY + (f.gy + 0.5) * grid.resolution;
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
//   Pink-Purple-Cyan gradient giống RViz2 Nav2
// ============================================================

export function drawCostmapNav2(ctx, w, h, viewport, grid) {
  if (!grid || !grid.costmap) return;

  const { worldToScreen, scale } = viewport;
  const cellPx = grid.resolution * scale;
  if (cellPx < 1.5) return; // Skip at extreme zoom out

  // Pre-compute color lookup table (0-254)
  if (!drawCostmapNav2._lut) {
    const lut = new Array(255);
    for (let i = 0; i < 255; i++) {
      if (i === 0) {
        lut[i] = null; // Free — transparent
      } else if (i >= 253) {
        // Lethal — solid dark red/black
        lut[i] = 'rgba(139, 0, 0, 0.85)';
      } else if (i >= 200) {
        // Inscribed — hot pink
        lut[i] = 'rgba(255, 0, 127, 0.7)';
      } else {
        // Inflation gradient: purple → cyan → light blue
        const t = i / 200; // 0 → 1
        const r = Math.round(180 * t);           // 0 → 180
        const g = Math.round(50 + 100 * (1 - t));  // 150 → 50
        const b = Math.round(220 + 35 * (1 - t));  // 255 → 220
        const a = (0.08 + 0.45 * t).toFixed(2);
        lut[i] = `rgba(${r}, ${g}, ${b}, ${a})`;
      }
    }
    drawCostmapNav2._lut = lut;
  }
  const lut = drawCostmapNav2._lut;

  for (let gy = 0; gy < grid.height; gy++) {
    for (let gx = 0; gx < grid.width; gx++) {
      const idx = gy * grid.width + gx;
      const cost = grid.costmap[idx];
      if (cost === 0) continue;

      const color = lut[Math.min(cost, 254)];
      if (!color) continue;

      const worldX = grid.originX + gx * grid.resolution;
      const worldY = grid.originY + gy * grid.resolution;
      const s = worldToScreen(worldX, worldY + grid.resolution);

      ctx.fillStyle = color;
      ctx.fillRect(s.x, s.y, cellPx + 0.5, cellPx + 0.5);
    }
  }
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
