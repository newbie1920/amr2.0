/**
 * AMR 2.0 — RViz-style LIDAR Grid Visualization (v2)
 * 
 * Cải tiến v2:
 *   - Dynamic positioning dựa trên grid.originX/Y
 *   - Alpha cao hơn, map nhìn thấy rõ ràng
 *   - Frontier cells hiển thị trên map (cyan)
 *   - Path overlay (planned path)
 */

import React, { useMemo, useRef, useEffect } from 'react';
import * as THREE from 'three';
import { useFrame } from '@react-three/fiber';

// ============================================================
//   RVIZ COLOR SCHEME (v2 — higher visibility)
// ============================================================

const COLORS = {
  free:     { r: 45,  g: 55,  b: 72,  a: 140 },   // Dark gray, solid
  unknown:  { r: 30,  g: 35,  b: 45,  a: 8 },      // Nearly transparent
  occupied: { r: 239, g: 68,  b: 68,  a: 230 },    // Bright red
  frontier: { r: 0,   g: 230, b: 255, a: 220 },    // Cyan
  trail:    0x3b82f6,
  laser:    0x22c55e,
};

// ============================================================
//   OCCUPANCY GRID FLOOR — Flat on XZ plane
// ============================================================

export function OccupancyGridVisualizer({ grid, opacity = 0.9, warehouseCX = 5, warehouseCZ = 5 }) {
  const meshRef = useRef(null);
  const lastScanRef = useRef(0);

  // Create DataTexture from grid
  const texture = useMemo(() => {
    if (!grid) return null;
    const data = new Uint8Array(grid.width * grid.height * 4);
    const tex = new THREE.DataTexture(data, grid.width, grid.height, THREE.RGBAFormat);
    tex.magFilter = THREE.NearestFilter;
    tex.minFilter = THREE.NearestFilter;
    tex.needsUpdate = true;
    return tex;
  }, [grid?.width, grid?.height]);

  // Update texture every frame (only when dirty)
  useFrame(() => {
    if (!grid || !texture) return;
    if (grid.scanCount === lastScanRef.current && !grid.isDirty()) return;
    lastScanRef.current = grid.scanCount;

    const data = texture.image.data;
    const w = grid.width;
    const h = grid.height;

    // Build frontier set for fast lookup
    const frontierSet = new Set();
    if (grid.frontierCells) {
      for (const f of grid.frontierCells) {
        frontierSet.add(f.gy * w + f.gx);
      }
    }

    for (let gy = 0; gy < h; gy++) {
      for (let gx = 0; gx < w; gx++) {
        const srcIdx = gy * w + gx;
        const lo = grid.logOdds[srcIdx];
        const dstIdx = (gy * w + gx) * 4;

        // Frontier cells → Cyan
        if (frontierSet.has(srcIdx)) {
          data[dstIdx]     = COLORS.frontier.r;
          data[dstIdx + 1] = COLORS.frontier.g;
          data[dstIdx + 2] = COLORS.frontier.b;
          data[dstIdx + 3] = COLORS.frontier.a;
        } else if (lo > 0.5) {
          // OCCUPIED — Red → White with confidence
          const intensity = Math.min(1.0, (lo - 0.5) / 3.5);
          data[dstIdx]     = Math.round(239 + 16 * intensity);
          data[dstIdx + 1] = Math.round(68 * (1 - intensity * 0.7));
          data[dstIdx + 2] = Math.round(68 * (1 - intensity * 0.7));
          data[dstIdx + 3] = Math.round(200 + 55 * intensity);
        } else if (lo < -0.5) {
          // FREE — Dark blue-gray, visible
          const intensity = Math.min(1.0, (-lo - 0.5) / 3.5);
          data[dstIdx]     = COLORS.free.r;
          data[dstIdx + 1] = COLORS.free.g;
          data[dstIdx + 2] = COLORS.free.b;
          data[dstIdx + 3] = Math.round(COLORS.free.a * (0.5 + 0.5 * intensity));
        } else {
          // UNKNOWN
          data[dstIdx]     = COLORS.unknown.r;
          data[dstIdx + 1] = COLORS.unknown.g;
          data[dstIdx + 2] = COLORS.unknown.b;
          data[dstIdx + 3] = COLORS.unknown.a;
        }
      }
    }

    texture.needsUpdate = true;
    grid.clearDirty();
  });

  if (!grid || !texture) return null;

  // === DYNAMIC POSITIONING ===
  // Grid center in world = originX + worldWidth/2, originY + worldHeight/2
  // Three.js warehouse center assumed at (warehouseCX, 0, -warehouseCZ)
  // We need grid center relative to Three.js world origin
  const gridCenterX = grid.originX + grid.worldWidth / 2;
  const gridCenterY = grid.originY + grid.worldHeight / 2;

  // Three.js: X = world X offset, Z = -world Y offset (Y-up convention)
  // Nếu warehouse center (5,5) là origin Three.js (0,0,0):
  const posX = gridCenterX - warehouseCX;
  const posZ = -(gridCenterY - warehouseCZ);

  return (
    <mesh
      ref={meshRef}
      position={[posX, 0.03, posZ]}
      rotation={[-Math.PI / 2, 0, 0]}
    >
      <planeGeometry args={[grid.worldWidth, grid.worldHeight]} />
      <meshBasicMaterial
        map={texture}
        transparent
        opacity={opacity}
        depthWrite={false}
        side={THREE.DoubleSide}
      />
    </mesh>
  );
}

// ============================================================
//   LASER SCAN OVERLAY
// ============================================================

export function LaserScanOverlay({ robotX, robotY, robotHeading, lidarPoints, warehouseCX = 5, warehouseCZ = 5 }) {
  const { hitPositions, linePositions } = useMemo(() => {
    if (!lidarPoints || lidarPoints.length === 0) {
      return { hitPositions: new Float32Array(0), linePositions: new Float32Array(0) };
    }

    const hits = [];
    const lines = [];
    // robotHeading is in degrees, convert to radians (2D convention: 0=+X, CCW)
    const headingRad = (robotHeading ?? 0) * Math.PI / 180;
    // Robot position in 3D coords
    const rx3D = robotX - warehouseCX;
    const rz3D = -(robotY - warehouseCZ);

    for (const pt of lidarPoints) {
      const distM = pt.d / 1000.0;
      if (distM < 0.05 || distM > 3.0) continue;

      const lidarRad = (pt.a * Math.PI) / 180.0;
      // World angle in 2D (same convention as simLidar: theta + localAngle)
      const worldAngle = headingRad + lidarRad;
      // Compute 2D world hit position first
      const hitX_2D = robotX + Math.cos(worldAngle) * distM;
      const hitY_2D = robotY + Math.sin(worldAngle) * distM;
      // Convert 2D → 3D
      const hx = hitX_2D - warehouseCX;
      const hz = -(hitY_2D - warehouseCZ);

      hits.push(hx, 0.08, hz);

      if (hits.length % 12 === 3) {
        lines.push(rx3D, 0.06, rz3D);
        lines.push(hx, 0.06, hz);
      }
    }

    return {
      hitPositions: new Float32Array(hits),
      linePositions: new Float32Array(lines),
    };
  }, [robotX, robotY, robotHeading, lidarPoints, warehouseCX, warehouseCZ]);

  if (hitPositions.length === 0) return null;

  return (
    <group>
      <points>
        <bufferGeometry>
          <bufferAttribute
            attach="attributes-position"
            array={hitPositions}
            count={hitPositions.length / 3}
            itemSize={3}
          />
        </bufferGeometry>
        <pointsMaterial color="#ef4444" size={0.06} sizeAttenuation depthWrite={false} transparent opacity={0.9} />
      </points>

      {linePositions.length > 0 && (
        <lineSegments>
          <bufferGeometry>
            <bufferAttribute
              attach="attributes-position"
              array={linePositions}
              count={linePositions.length / 3}
              itemSize={3}
            />
          </bufferGeometry>
          <lineBasicMaterial color={COLORS.laser} transparent opacity={0.15} depthWrite={false} />
        </lineSegments>
      )}
    </group>
  );
}

// ============================================================
//   ROBOT TRAIL
// ============================================================

const MAX_TRAIL_POINTS = 300;
const trailBuffer = [];

export function RobotTrail({ robotX, robotY, warehouseCX = 5, warehouseCZ = 5, isMapping = false }) {
  const lineRef = useRef(null);
  const lastPosRef = useRef({ x: 0, y: 0 });

  useFrame(() => {
    if (!isMapping) return;

    const dx = robotX - lastPosRef.current.x;
    const dy = robotY - lastPosRef.current.y;
    const dist = Math.sqrt(dx * dx + dy * dy);

    if (dist > 0.05) {
      const px = robotX - warehouseCX;
      const pz = -(robotY - warehouseCZ);
      trailBuffer.push(new THREE.Vector3(px, 0.04, pz));
      if (trailBuffer.length > MAX_TRAIL_POINTS) trailBuffer.shift();
      lastPosRef.current = { x: robotX, y: robotY };

      if (lineRef.current && trailBuffer.length > 1) {
        lineRef.current.geometry.setFromPoints(trailBuffer);
      }
    }
  });

  return (
    <line ref={lineRef}>
      <bufferGeometry />
      <lineBasicMaterial color={COLORS.trail} transparent opacity={0.6} linewidth={2} />
    </line>
  );
}

// ============================================================
//   OBSTACLE CONTOURS
// ============================================================

export function ObstacleContours({ grid, warehouseCX = 5, warehouseCZ = 5 }) {
  const points = useMemo(() => {
    if (!grid) return [];

    const linePoints = [];

    for (let y = 0; y < grid.height; y++) {
      for (let x = 0; x < grid.width; x++) {
        const lo = grid.logOdds[y * grid.width + x];
        if (lo <= 1.5) continue;

        let isContour = false;
        for (let dy = -1; dy <= 1 && !isContour; dy++) {
          for (let dx = -1; dx <= 1 && !isContour; dx++) {
            if (dx === 0 && dy === 0) continue;
            const nx = x + dx, ny = y + dy;
            if (nx >= 0 && nx < grid.width && ny >= 0 && ny < grid.height) {
              if (grid.logOdds[ny * grid.width + nx] < -0.5) {
                isContour = true;
              }
            }
          }
        }

        if (isContour) {
          // Use dynamic origin for correct positioning
          const worldX = grid.originX + (x + 0.5) * grid.resolution - warehouseCX;
          const worldZ = -(grid.originY + (y + 0.5) * grid.resolution - warehouseCZ);
          linePoints.push(new THREE.Vector3(worldX, 0.06, worldZ));
        }
      }
    }

    return linePoints;
  }, [grid?.scanCount, warehouseCX, warehouseCZ]);

  const lineRef = useRef(null);

  useEffect(() => {
    if (lineRef.current && points.length > 0) {
      lineRef.current.geometry.setFromPoints(points);
    }
  }, [points]);

  if (points.length === 0) return null;

  return (
    <points ref={lineRef}>
      <bufferGeometry />
      <pointsMaterial color="#f97316" size={0.08} sizeAttenuation transparent opacity={0.7} depthWrite={false} />
    </points>
  );
}

// ============================================================
//   GRID CELLS OVERLAY (backward compat)
// ============================================================

export function GridCellsOverlay({ grid, cellOpacity = 0.1 }) {
  return null;
}

export default OccupancyGridVisualizer;
