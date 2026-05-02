/**
 * AMR 2.0 — Warehouse Map Component (3D Version)
 * Bản đồ kho xưởng 3D sử dụng React Three Fiber
 */

import React, { useMemo, useState, useRef, useEffect, useCallback } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls, Grid, Text, Box, Cylinder, Sphere, Edges, Line, Cone } from '@react-three/drei';
import * as THREE from 'three';
import {
  WAREHOUSE_WIDTH, WAREHOUSE_HEIGHT,
  SHELVES, GATES, CHARGING_STATIONS
} from '../../core/warehouse.js';
import useRobotStore from '../../stores/robotStore.js';
import useMapStore from '../../stores/mapStore.js';
import useNavStore from '../../stores/navStore.js';
import { getExplorationPhase, getExplorationInfo } from '../../core/exploration.js';
import vi from '../../i18n/vi.js';
import MapManager from '../MapManager/MapManager.jsx';
import useInventoryStore from '../../stores/inventoryStore.js';

// ============================================================
//   OCCUPANCY GRID 3D MESH (Textured Plane)
// ============================================================

/**
 * OccupancyGridMesh — Render Occupancy Grid dưới dạng textured plane trong Three.js
 * Cập nhật texture realtime khi grid thay đổi
 */
function OccupancyGridMesh({ grid, cx, cz }) {
  const meshRef = useRef();
  const textureRef = useRef(null);
  const geoRef = useRef(null);

  // Tạo offscreen canvas đồng bộ để canvasTexture nhận được ngay từ lần render đầu tiên
  const canvas = useMemo(() => {
    const c = document.createElement('canvas');
    c.width = grid?.width || 80;
    c.height = grid?.height || 80;
    return c;
  }, []);

  // Cập nhật texture mỗi khi grid thay đổi
  useFrame(() => {
    if (!grid || !canvas) return;
    if (!grid.isDirty()) return;

    const ctx = canvas.getContext('2d');

    // Resize canvas nếu grid size thay đổi (auto-expand)
    if (canvas.width !== grid.width || canvas.height !== grid.height) {
      canvas.width = grid.width;
      canvas.height = grid.height;
    }

    // Render grid sang ImageData
    const imgData = grid.renderToImageData();
    ctx.putImageData(imgData, 0, 0);

    // Cập nhật Three.js texture
    if (textureRef.current) {
      textureRef.current.needsUpdate = true;
    }

    // Update geometry size nếu grid đã expand
    if (meshRef.current) {
      const posX = grid.originX + grid.worldWidth / 2 - cx;
      const posZ = -(grid.originY + grid.worldHeight / 2 - cz);
      meshRef.current.position.set(posX, 0.05, posZ);
    }

    grid.clearDirty();
  });

  if (!grid) return null;

  // Dynamic position: grid center in Three.js coords
  const posX = grid.originX + grid.worldWidth / 2 - cx;
  const posZ = -(grid.originY + grid.worldHeight / 2 - cz);

  return (
    <mesh
      ref={meshRef}
      position={[posX, 0.05, posZ]}
      rotation={[-Math.PI / 2, 0, 0]}
    >
      <planeGeometry ref={geoRef} args={[grid.worldWidth, grid.worldHeight]} />
      <meshBasicMaterial transparent depthWrite={false}>
        <canvasTexture
          ref={textureRef}
          attach="map"
          image={canvas}
          minFilter={THREE.NearestFilter}
          magFilter={THREE.NearestFilter}
        />
      </meshBasicMaterial>
    </mesh>
  );
}
import { OccupancyGridVisualizer, LaserScanOverlay, RobotTrail, ObstacleContours, GridCellsOverlay } from '../LidarGridVisualizer/LidarGridVisualizer.jsx';

// ============================================================
//   COLORS (3D Theme)
// ============================================================
const C_FLOOR = '#0f1923';
const C_SHELF = '#2d3b52';
const C_IMPORT = '#3b82f6';
const C_EXPORT = '#8b5cf6';
const C_CHARGER = '#f59e0b';
const C_ROBOT = '#10b981';

// ── Components ──────────────────────────────────────────────

function RobotModel({ position, rotation, battery, label, hasObs }) {
  const isHealthy = battery > 20;
  const color = isHealthy ? C_ROBOT : '#ef4444';

  return (
    <group position={position} rotation={rotation}>
      {/* Body */}
      <mesh castShadow position={[0, 0.15, 0]}>
        {/* Khung xe 30x30cm, cao 20cm */}
        <boxGeometry args={[0.3, 0.2, 0.3]} />
        <meshStandardMaterial color={color} metalness={0.6} roughness={0.2} />
        <Edges scale={1.05} threshold={15} color="#000" />
      </mesh>
      {/* Lidar/Sensor on top (front of robot) */}
      <mesh position={[0, 0.28, -0.05]}>
        <cylinderGeometry args={[0.06, 0.06, 0.06, 16]} />
        <meshStandardMaterial color="#ef4444" emissive={hasObs ? "#ef4444" : "#000"} emissiveIntensity={hasObs ? 1 : 0} />
      </mesh>
      {/* Wheels (along X axis — differential drive) */}
      <mesh position={[-0.17, 0.08, 0]} rotation={[0, 0, Math.PI / 2]}>
        <cylinderGeometry args={[0.08, 0.08, 0.04, 16]} />
        <meshStandardMaterial color="#1e293b" />
      </mesh>
      <mesh position={[0.17, 0.08, 0]} rotation={[0, 0, Math.PI / 2]}>
        <cylinderGeometry args={[0.08, 0.08, 0.04, 16]} />
        <meshStandardMaterial color="#1e293b" />
      </mesh>
      {/* Front Arrow (points to -Z in local space = forward direction) */}
      <mesh position={[0, 0.15, -0.16]} rotation={[-Math.PI / 2, 0, 0]}>
        <coneGeometry args={[0.06, 0.08, 16]} />
        <meshStandardMaterial color="#22c55e" />
      </mesh>

      {/* Label */}
      <Text
        position={[0, 0.8, 0]}
        fontSize={0.2}
        color="white"
        anchorX="center"
        anchorY="middle"
        rotation={[...rotation].map(r => -r)}
      >
        {label} ({battery}%) {hasObs ? "⚠️" : ""}
      </Text>
    </group>
  );
}

/**
 * WorldLidarPoints — Render lidar points in WORLD coordinates
 * 
 * simLidar outputs {a: localAngleDeg, d: distanceMM}
 * where a is angle relative to robot front (0° = forward).
 * 
 * Strategy: compute hit position in 2D world first, then convert to 3D.
 *   worldAngle = robotTheta + localAngle (in 2D math convention: 0=+X, CCW)
 *   hitX_2D = robotX_2D + cos(worldAngle) * dist
 *   hitY_2D = robotY_2D + sin(worldAngle) * dist
 *   
 *   3D_X = hitX_2D - warehouseCX
 *   3D_Z = -(hitY_2D - warehouseCZ)
 */
function WorldLidarPoints({ robotX2D, robotY2D, robotTheta, lidar, warehouseCX, warehouseCZ }) {
  if (!lidar || lidar.length === 0) return null;

  return (
    <group position={[0, 0.4, 0]}>
      {lidar.map((pt, i) => {
        // Local angle in radians
        const localRad = (pt.a * Math.PI) / 180.0;
        // World angle in 2D (same convention as simLidar: theta + local)
        const worldAngle = robotTheta + localRad;
        const dist_m = pt.d / 1000.0;
        // Hit position in 2D world coordinates
        const hitX = robotX2D + Math.cos(worldAngle) * dist_m;
        const hitY = robotY2D + Math.sin(worldAngle) * dist_m;
        // Convert 2D → 3D
        const wx = hitX - warehouseCX;
        const wz = -(hitY - warehouseCZ);
        return (
          <mesh key={i} position={[wx, 0, wz]}>
            <boxGeometry args={[0.04, 0.04, 0.04]} />
            <meshBasicMaterial color="#ef4444" />
          </mesh>
        );
      })}
    </group>
  );
}

function RealisticShelf({ shelf, cx, cz }) {
  const map2To3X = (x) => x - cx;
  const map2To3Z = (y) => -(y - cz);
  const inventory = useInventoryStore((s) => s.inventory);

  const w = (shelf.bounds.x2 - shelf.bounds.x1);
  const d = (shelf.bounds.y2 - shelf.bounds.y1);
  const x = map2To3X(shelf.bounds.x1 + w / 2);
  const z = map2To3Z(shelf.bounds.y1 + d / 2);

  const legSize = 0.1;
  const levelHeight = 1.0;
  const height = shelf.levels.length * levelHeight;

  return (
    <group position={[x, 0, z]}>
      {/* 4 Pillars */}
      <Box args={[legSize, height + 0.2, legSize]} position={[-w/2, height/2 + 0.1, -d/2]} castShadow>
        <meshStandardMaterial color="#475569" metalness={0.8} />
      </Box>
      <Box args={[legSize, height + 0.2, legSize]} position={[w/2, height/2 + 0.1, -d/2]} castShadow>
        <meshStandardMaterial color="#475569" metalness={0.8} />
      </Box>
      <Box args={[legSize, height + 0.2, legSize]} position={[-w/2, height/2 + 0.1, d/2]} castShadow>
        <meshStandardMaterial color="#475569" metalness={0.8} />
      </Box>
      <Box args={[legSize, height + 0.2, legSize]} position={[w/2, height/2 + 0.1, d/2]} castShadow>
        <meshStandardMaterial color="#475569" metalness={0.8} />
      </Box>

      {/* Base panel */}
      <Box args={[w + 0.05, 0.05, d + 0.05]} position={[0, 0.05, 0]} receiveShadow>
        <meshStandardMaterial color="#1e293b" metalness={0.5} roughness={0.8} />
      </Box>

      {shelf.levels.map((lvl, index) => {
        const yBase = index * levelHeight + 0.1;
        
        return (
          <group key={lvl.level}>
            {/* The horizontal panel above this level */}
            <Box args={[w + 0.05, 0.05, d + 0.05]} position={[0, yBase + levelHeight - 0.05, 0]} castShadow receiveShadow>
              <meshStandardMaterial color="#2d3b52" metalness={0.3} roughness={0.7} />
            </Box>

            {/* Slots / Bins rendering */}
            {lvl.slots.map((slot, sIdx) => {
              const slotW = w / lvl.slots.length;
              const slotCenterLocalX = -w/2 + slotW/2 + sIdx * slotW;
              const isOccupied = inventory.some(i => i.slot_id === slot.id);
              
              return (
                <group key={slot.id} position={[slotCenterLocalX, yBase + 0.4, 0]}>
                   {/* Bin Box */}
                   <Box args={[slotW - 0.05, 0.8, d - 0.1]} castShadow receiveShadow>
                     <meshStandardMaterial 
                       color={isOccupied ? "#ef4444" : "#1e293b"} 
                       emissive={isOccupied ? "#ef4444" : "#000000"}
                       emissiveIntensity={isOccupied ? 0.5 : 0}
                       transparent 
                       opacity={isOccupied ? 0.8 : 0.5} 
                       polygonOffset polygonOffsetFactor={1} 
                     />
                     <Edges color={isOccupied ? "#f87171" : "#475569"} />
                   </Box>
                   {/* Front Label */}
                   <Text position={[0, 0, d/2 - 0.04]} fontSize={0.12} color="#bae6fd" anchorY="bottom" anchorX="center">
                     T{lvl.level} - Ô{sIdx + 1}
                   </Text>
                   {/* Back Label (in case we rotate) */}
                   <Text position={[0, 0, -d/2 + 0.04]} fontSize={0.12} color="#bae6fd" anchorY="bottom" anchorX="center" rotation={[0, Math.PI, 0]}>
                     T{lvl.level} - Ô{sIdx + 1}
                   </Text>
                </group>
              );
            })}
          </group>
        );
      })}

      {/* Shelf Main Title */}
      <Text position={[0, height + 0.4, 0]} fontSize={0.3} color="white" rotation={[-Math.PI / 6, 0, 0]}>
        {shelf.name}
      </Text>
    </group>
  );
}

function WarehouseScene({ robots, activePath, mapType, occupancyGrid, selectedRobotId, mappingActive, layers }) {
  // Center the warehouse in the world
  const cx = WAREHOUSE_WIDTH / 2;
  const cz = WAREHOUSE_HEIGHT / 2; // y in 2D is z in 3D

  // Mapping from 2D coordinate system to 3D.
  const map2To3X = (x) => x - cx;
  const map2To3Z = (y) => -(y - cz);

  // Get the active occupancy grid for selected robot
  const activeGrid = selectedRobotId ? occupancyGrid[selectedRobotId] : null;

  return (
    <group>
      {/* Floor */}
      <Grid
        position={[0, 0.01, 0]}
        args={[WAREHOUSE_WIDTH, WAREHOUSE_HEIGHT]}
        sectionSize={1}
        cellColor="#1a2533"
        sectionColor="#243040"
        fadeDistance={30}
      />
      <mesh receiveShadow position={[0, 0, 0]} rotation={[-Math.PI / 2, 0, 0]}>
        <planeGeometry args={[WAREHOUSE_WIDTH, WAREHOUSE_HEIGHT]} />
        <meshStandardMaterial color={C_FLOOR} />
      </mesh>

      {/* LIDAR Occupancy Grid Visualization (Chỉ ở chế độ LIDAR) */}
      {mapType === 'lidar' && Object.entries(robots).map(([robotId, robot]) => {
        const grid = occupancyGrid?.[robotId];
        const isThisMapping = !!mappingActive?.[robotId];
        const telem = robot.telemetry || {};
        return (
          <group key={`lidar-${robotId}`}>
            {/* Occupancy Grid Floor */}
            {grid && layers.grid && (
              <OccupancyGridVisualizer grid={grid} opacity={0.7} warehouseCX={cx} warehouseCZ={cz} />
            )}
            {/* Obstacle Contours */}
            {grid && layers.contours && (
              <ObstacleContours grid={grid} warehouseCX={cx} warehouseCZ={cz} />
            )}
            {/* Real-time Laser Scan */}
            {layers.laser && telem.lidar && telem.lidar.length > 0 && (
              <LaserScanOverlay
                robotX={telem.x ?? 0}
                robotY={telem.y ?? 0}
                robotHeading={telem.heading ?? 0}
                lidarPoints={telem.lidar}
                warehouseCX={cx}
                warehouseCZ={cz}
              />
            )}
            {/* Robot Trail */}
            {layers.trail && (
              <RobotTrail
                robotX={telem.x ?? 0}
                robotY={telem.y ?? 0}
                warehouseCX={cx}
                warehouseCZ={cz}
                isMapping={isThisMapping}
              />
            )}
          </group>
        );
      })}

      {/* Shelves - Chỉ hiện ở chế độ 3D có sẵn */}
      {mapType === '3d' && SHELVES.map((shelf, idx) => (
        <RealisticShelf key={idx} shelf={shelf} cx={cx} cz={cz} />
      ))}

      {/* Gates - Chỉ hiện ở chế độ 3D có sẵn */}
      {mapType === '3d' && Object.values(GATES).map((gate, idx) => {
        const x = map2To3X(gate.x);
        const z = map2To3Z(gate.y);
        
        let color = C_IMPORT;
        if (gate.type === 'export') color = C_EXPORT;
        if (gate.type === 'error') color = '#ef4444'; // Redish for error

        return (
          <group key={idx} position={[x, 0, z]}>
            {/* Chân dế / Khung băng chuyền */}
             <Box args={[1.4, 0.3, 0.8]} position={[0, 0.15, 0]} castShadow receiveShadow>
               <meshStandardMaterial color="#334155" metalness={0.4} roughness={0.6} />
             </Box>
             
             {/* Trục Rulo băng chuyền (Rollers) */}
             {[...Array(6)].map((_, i) => (
               <Cylinder 
                 key={i} 
                 args={[0.06, 0.06, 1.3, 16]} 
                 position={[-0.5 + i * 0.2, 0.3 + 0.06, 0]} 
                 rotation={[Math.PI / 2, 0, 0]} 
                 castShadow
               >
                 <meshStandardMaterial color="#94a3b8" metalness={0.8} />
               </Cylinder>
             ))}

             {/* Đèn vạch cảnh báo màu theo loại cổng (Nhập = Xanh, Xuất = Tím) */}
             <Box args={[1.4, 0.05, 0.05]} position={[0, 0.3, 0.4]}>
               <meshStandardMaterial color={color} emissive={color} emissiveIntensity={0.8} />
             </Box>
             
            {/* Bảng tên hiển thị */}
            <Text position={[0, 0.6, 0]} fontSize={0.15} color="white" rotation={[-Math.PI / 4, 0, 0]}>
              {gate.name}
            </Text>
          </group>
        );
      })}

      {/* Charging Stations - Chỉ hiện ở chế độ 3D có sẵn */}
      {mapType === '3d' && CHARGING_STATIONS.map((charger, idx) => {
        const x = map2To3X(charger.x);
        const z = map2To3Z(charger.y);
        
        // Charger heading mapping. In 2D: charger 1 has heading 270 (facing -y down).
        // 3D rotation: 0 radians points +Z.
        // Let's just fix it facing +Z (down the screen) which is rotation Y = 0.
        // If heading is 270 (-Y in 2D), mapping to 3D makes it points +Z or -Z depending on flip.
        const rotY = (charger.heading === 270) ? 0 : 0; 

        return (
          <group key={idx} position={[x, 0, z]} rotation={[0, rotY, 0]}>
            {/* Base / Floor outline */}
            <mesh position={[0, 0.01, -0.1]} rotation={[-Math.PI / 2, 0, 0]}>
              <planeGeometry args={[1.0, 1.2]} />
              <meshBasicMaterial color={C_CHARGER} transparent opacity={0.15} />
            </mesh>

            {/* 2D Mũi tên và xe trong khu vực báo hiệu cho người đặt đúng */}
            <group position={[0, 0.02, -0.1]} rotation={[-Math.PI / 2, 0, 0]}>
               {/* 2D Outline Border */}
               <Line points={[[-0.3, -0.4, 0], [0.3, -0.4, 0], [0.3, 0.4, 0], [-0.3, 0.4, 0], [-0.3, -0.4, 0]]} color={C_CHARGER} lineWidth={2} />
               {/* 2D Arrow */}
               <Line points={[[0, 0, 0], [0, 0.4, 0], [-0.1, 0.3, 0], [0, 0.4, 0], [0.1, 0.3, 0]]} color={C_CHARGER} lineWidth={3} />
            </group>

            {/* Chữ U: Back wall */}
            <Box args={[1.0, 0.5, 0.2]} position={[0, 0.25, -0.6]} castShadow receiveShadow>
              <meshStandardMaterial color={C_CHARGER} />
            </Box>
            {/* Chữ U: Left wall */}
            <Box args={[0.2, 0.5, 0.6]} position={[-0.4, 0.25, -0.2]} castShadow receiveShadow>
              <meshStandardMaterial color={C_CHARGER} />
            </Box>
            {/* Chữ U: Right wall */}
            <Box args={[0.2, 0.5, 0.6]} position={[0.4, 0.25, -0.2]} castShadow receiveShadow>
              <meshStandardMaterial color={C_CHARGER} />
            </Box>
            
            <Text position={[0, 0.6, -0.6]} fontSize={0.2} color="white">
              ⚡ {charger.name}
            </Text>
          </group>
        );
      })}

      {/* Occupancy Grid (Lidar Map) */}
      {mapType === 'lidar' && activeGrid && (
        <OccupancyGridMesh grid={activeGrid} cx={cx} cz={cz} />
      )}

      {/* Active Paths for all robots */}
      {Object.values(useNavStore((s) => s.appNavigationSessions) || {}).map((session, index) => {
        if (!session || session.active !== true || !Array.isArray(session.path) || session.path.length <= 1) return null;
        return (
          <group key={session.robotId}>
            {session.path.map((pt, i) => {
              if (i === 0) return null;
              return (
                <Sphere key={i} args={[0.05]} position={[map2To3X(pt.x), 0.1, map2To3Z(pt.y)]}>
                  <meshStandardMaterial color="#3b82f6" emissive="#3b82f6" emissiveIntensity={1} />
                </Sphere>
              );
            })}
          </group>
        );
      })}

      {/* Robots */}
      {Object.values(robots).map(robot => {
        if (robot.status !== 'connected') return null;
        const x = map2To3X(robot.telemetry.x);
        const z = map2To3Z(robot.telemetry.y);
        // 2D theta to 3D rotY conversion:
        // 2D: theta=0 → +X (right), theta=PI/2 → +Y (up)
        // 3D: X stays +X, Y_2d maps to -Z_3d
        // Model front is -Z local, so:
        //   theta=0 → face +X → rotY = PI/2 (rotate -Z to +X)
        //   theta=PI/2 → face +Y → face -Z → rotY = PI (rotate -Z to -Z, i.e. 0... wait)
        // Correct formula: rotY = theta + PI/2
        // When theta=0: rotY=PI/2 → local -Z rotates to +X ✓
        // When theta=PI/2: rotY=PI → local -Z rotates to -Z (which is +Y_2d) ✓
        const theta2D = robot.telemetry.headingRad ?? 0;
        const rotY = theta2D + Math.PI / 2;
        
        return (
          <React.Fragment key={robot.id}>
            <RobotModel
              position={[x, 0, z]}
              rotation={[0, rotY, 0]}
              battery={robot.telemetry.battery}
              label={robot.name}
              hasObs={robot.telemetry.obs || false}
            />
            {/* Lidar points in world coordinates — pass 2D coords + theta */}
            <WorldLidarPoints
              robotX2D={robot.telemetry.x}
              robotY2D={robot.telemetry.y}
              robotTheta={theta2D}
              lidar={robot.telemetry.lidar || []}
              warehouseCX={cx}
              warehouseCZ={cz}
            />
          </React.Fragment>
        );
      })}
    </group>
  );
}

// ============================================================
//   CAMERA DIRECTOR
// ============================================================

function CameraDirector({ viewMode }) {
  const controlsRef = useRef(null);
  const targetPosition = useMemo(() => new THREE.Vector3(), []);
  const targetLookAt = useMemo(() => new THREE.Vector3(0, 0, 0), []);

  useFrame((state, delta) => {
    // 0 = free, 1 = top, 2 = iso
    if (viewMode === 'top') {
      targetPosition.set(0, 15, 0); // Directly above
      targetLookAt.set(0, 0, 0);
    } else if (viewMode === 'iso') {
      targetPosition.set(10, 10, 10); // Isometric angle
      targetLookAt.set(0, 0, 0);
    } else {
      // free mode: we don't force camera position, let OrbitControls handle it completely
      return; 
    }

    // Custom lerping for smooth transition
    state.camera.position.lerp(targetPosition, delta * 3);
    if (controlsRef.current) {
      controlsRef.current.target.lerp(targetLookAt, delta * 3);
      controlsRef.current.update();
    }
  });

  return (
    <OrbitControls 
      ref={controlsRef}
      makeDefault 
      maxPolarAngle={Math.PI / 2 - 0.1} // Prevent going under the floor
      minDistance={2}
      maxDistance={30}
    />
  );
}

export default function WarehouseMap({ activePath }) {
  const robots = useRobotStore((s) => s.robots);
  const occupancyGrid = useMapStore((s) => s.occupancyGrid);
  const mappingActive = useMapStore((s) => s.mappingActive);
  const selectedRobotId = useRobotStore((s) => s.selectedRobotId);
  const startMapping = useMapStore((s) => s.startMapping);
  const stopMapping = useMapStore((s) => s.stopMapping);
  const saveMap = useMapStore((s) => s.saveMap);
  const loadMap = useMapStore((s) => s.loadMap);
  const [viewMode, setViewMode] = useState('free'); // 'free', 'top', 'iso'
  const [mapType, setMapType] = useState('3d'); // '3d', 'lidar'
  const [showMapManager, setShowMapManager] = useState(false);
  const [showControls, setShowControls] = useState(true);
  const [layers, setLayers] = useState({ grid: true, laser: true, trail: true, contours: true });
  const fileInputRef = useRef(null);
  const toggleLayer = (key) => setLayers(prev => ({ ...prev, [key]: !prev[key] }));

  // Lấy robot đầu tiên nếu chưa chọn
  const activeRobotId = selectedRobotId || Object.keys(robots)[0] || null;
  const isMapping = activeRobotId ? !!mappingActive[activeRobotId] : false;
  const activeGrid = activeRobotId ? occupancyGrid[activeRobotId] : null;

  // Handler cho nút Bắt đầu/Dừng quét
  const handleToggleMapping = useCallback(() => {
    if (!activeRobotId) return;
    const getRobotStore = () => useRobotStore.getState();
    if (isMapping) {
      stopMapping(activeRobotId, getRobotStore);
    } else {
      startMapping(activeRobotId, getRobotStore);
    }
  }, [activeRobotId, isMapping, startMapping, stopMapping]);

  // Handler cho nút Lưu Map
  const handleSaveMap = useCallback(() => {
    if (!activeRobotId) return;
    saveMap(activeRobotId);
  }, [activeRobotId, saveMap]);

  // Handler cho Load Map
  const handleLoadMap = useCallback((e) => {
    const file = e.target.files[0];
    if (!file || !activeRobotId) return;
    const reader = new FileReader();
    reader.onload = (evt) => {
      try {
        const json = JSON.parse(evt.target.result);
        loadMap(activeRobotId, json);
      } catch (err) {
        console.error('Lỗi đọc file map:', err);
      }
    };
    reader.readAsText(file);
    // Reset input
    e.target.value = '';
  }, [activeRobotId, loadMap]);

  return (
    <div className="warehouse-map" style={{ width: '100%', height: '100%', background: C_FLOOR, position: 'relative' }}>
      
      {/* ── CAMERA & MAP CONTROLS UI ── */}
      <div style={{ position: 'absolute', top: 16, left: 16, zIndex: 10, display: 'flex', flexDirection: 'column', gap: '8px', background: 'rgba(15, 25, 35, 0.8)', padding: '10px', borderRadius: '8px', backdropFilter: 'blur(10px)' }}>
        
        {/* Nút Ẩn/Hiện Bảng Điều Khiển */}
        <div style={{ display: 'flex', justifyContent: 'flex-end', marginBottom: showControls ? '0px' : '-8px' }}>
          <button 
            className="btn btn--sm btn--ghost" 
            style={{ padding: '2px 8px', fontSize: '10px', color: '#94a3b8', background: 'rgba(255,255,255,0.05)' }}
            onClick={() => setShowControls(!showControls)}
          >
            {showControls ? '▲ Ẩn bớt' : '▼ Bảng Điều Khiển'}
          </button>
        </div>

        {showControls && (
          <>
            <div style={{ display: 'flex', gap: '8px' }}>
              <button 
                className={`btn btn--sm ${viewMode === 'top' ? 'btn--primary' : 'btn--ghost'}`}
                onClick={() => setViewMode('top')}
              >
                🛰️ 2D Kế hoạch
              </button>
              <button 
                className={`btn btn--sm ${viewMode === 'iso' ? 'btn--primary' : 'btn--ghost'}`}
                onClick={() => setViewMode('iso')}
              >
                🧩 3D Tổng quan
              </button>
              <button 
                className={`btn btn--sm ${viewMode === 'free' ? 'btn--primary' : 'btn--ghost'}`}
                onClick={() => setViewMode('free')}
              >
                👀 3D Tự do
              </button>
            </div>
            
            <div style={{ display: 'flex', gap: '8px', borderTop: '1px solid rgba(255,255,255,0.1)', paddingTop: '8px' }}>
               <button 
                className={`btn btn--sm ${mapType === '3d' ? 'btn--success' : 'btn--ghost'}`}
                onClick={() => setMapType('3d')}
                style={{ flex: 1 }}
              >
                🏢 Map Mô Phỏng (Có sẵn)
              </button>
              <button 
                className={`btn btn--sm ${mapType === 'lidar' ? 'btn--success' : 'btn--ghost'}`}
                onClick={() => setMapType('lidar')}
                style={{ flex: 1 }}
              >
                🔴 Map Lidar (Point Cloud)
              </button>
            </div>

            {/* MAPPING CONTROLS (Chỉ hiện khi ở chế độ Lidar) */}
            {mapType === 'lidar' && (
              <div style={{ display: 'flex', flexDirection: 'column', gap: '6px', borderTop: '1px solid rgba(255,255,255,0.1)', paddingTop: '8px', marginTop: '4px' }}>
                <div style={{ display: 'flex', gap: '8px' }}>
                  <button 
                    className={`btn btn--sm ${isMapping ? 'btn--danger' : 'btn--primary'}`}
                    style={{ flex: 1, fontSize: '11px' }}
                    onClick={handleToggleMapping}
                    disabled={!activeRobotId}
                  >
                    {isMapping ? '⏹ Dừng quét + Lưu map' : '🚀 Bắt đầu quét (Tự lái)'}
                  </button>
                </div>

                {/* Exploration Phase Status */}
                {isMapping && (
                  <div style={{ 
                    fontSize: '10px', padding: '4px 8px', borderRadius: '4px',
                    background: 'rgba(34,197,94,0.15)', color: '#4ade80', 
                    textAlign: 'center', fontWeight: 600,
                  }}>
                    {(() => {
                      const phase = getExplorationPhase();
                      const info = getExplorationInfo();
                      if (phase === 'init_spin') return '🔄 Đang xoay 360° quét xung quanh...';
                      if (phase === 'find_frontier') return '🔍 Đang tìm frontier...';
                      if (phase === 'navigate') return `🚗 Đang lái đến frontier (${info.clusterCount} vùng)...`;
                      if (phase === 'arrived_scan') return '📡 Đến nơi, quét thêm...';
                      if (phase === 'recovery_spin') return '🔄 Recovery: Xoay tìm vùng mới...';
                      if (phase === 'recovery_backup') return '↩️ Recovery: Lùi lại...';
                      if (phase === 'complete') return '✅ Map đã quét xong!';
                      return '⏳ Đang khởi động...';
                    })()}
                  </div>
                )}
                
                <div style={{ display: 'flex', gap: '8px' }}>
                  <button 
                    className="btn btn--sm btn--ghost"
                    style={{ flex: 1, fontSize: '11px', background: 'rgba(59,130,246,0.15)', color: '#93c5fd', border: '1px solid rgba(59,130,246,0.3)' }}
                    onClick={() => setShowMapManager(true)}
                  >
                    🗺️ Quản lý Bản đồ
                  </button>
                </div>

                {/* Layer Toggle — RViz-style */}
                <div style={{ borderTop: '1px solid rgba(255,255,255,0.06)', paddingTop: '6px', marginTop: '2px' }}>
                  <div style={{ fontSize: '9px', color: '#64748b', marginBottom: '4px', fontWeight: 600, textTransform: 'uppercase', letterSpacing: '0.5px' }}>
                    Layers
                  </div>
                  <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '3px' }}>
                    {[
                      { key: 'grid', icon: '🟩', label: 'Grid' },
                      { key: 'laser', icon: '📡', label: 'Laser' },
                      { key: 'trail', icon: '🛤️', label: 'Trail' },
                      { key: 'contours', icon: '🔶', label: 'Tường' },
                    ].map(l => (
                      <button
                        key={l.key}
                        onClick={() => toggleLayer(l.key)}
                        style={{
                          background: layers[l.key] ? 'rgba(59,130,246,0.2)' : 'rgba(255,255,255,0.03)',
                          border: layers[l.key] ? '1px solid rgba(59,130,246,0.4)' : '1px solid rgba(255,255,255,0.06)',
                          borderRadius: '5px',
                          padding: '3px 6px',
                          fontSize: '10px',
                          color: layers[l.key] ? '#93c5fd' : '#475569',
                          cursor: 'pointer',
                          transition: 'all 0.15s',
                        }}
                      >
                        {l.icon} {l.label}
                      </button>
                    ))}
                  </div>
                </div>

                {/* Mapping Stats HUD */}
                {activeGrid && (() => {
                  const totalCells = activeGrid.width * activeGrid.height;
                  let knownCells = 0;
                  let occupiedCells = 0;
                  for (let i = 0; i < activeGrid.logOdds.length; i++) {
                    const lo = activeGrid.logOdds[i];
                    if (lo > 0.5) { knownCells++; occupiedCells++; }
                    else if (lo < -0.5) { knownCells++; }
                  }
                  const coverage = ((knownCells / totalCells) * 100).toFixed(1);
                  const robot = activeRobotId && robots[activeRobotId];
                  const rx = robot?.telemetry?.x ?? 0;
                  const ry = robot?.telemetry?.y ?? 0;
                  const rh = robot?.telemetry?.heading ?? 0;

                  return (
                    <div style={{
                      background: 'rgba(0,0,0,0.3)',
                      borderRadius: '6px',
                      padding: '6px 8px',
                      fontSize: '10px',
                      color: '#94a3b8',
                      lineHeight: 1.5,
                      fontFamily: 'monospace',
                    }}>
                      <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                        <span>Scans</span>
                        <strong style={{ color: '#10b981' }}>{activeGrid.scanCount}</strong>
                      </div>
                      <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                        <span>Coverage</span>
                        <strong style={{ color: '#3b82f6' }}>{coverage}%</strong>
                      </div>
                      <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                        <span>Walls</span>
                        <strong style={{ color: '#ef4444' }}>{occupiedCells} cells</strong>
                      </div>
                      <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                        <span>Grid</span>
                        <span>{activeGrid.width}×{activeGrid.height} ({activeGrid.resolution}m)</span>
                      </div>
                      <div style={{ borderTop: '1px solid rgba(255,255,255,0.06)', marginTop: '3px', paddingTop: '3px' }}>
                        <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                          <span>Pos</span>
                          <span style={{ color: '#e2e8f0' }}>({rx.toFixed(2)}, {ry.toFixed(2)})</span>
                        </div>
                        <div style={{ display: 'flex', justifyContent: 'space-between' }}>
                          <span>Heading</span>
                          <span style={{ color: '#e2e8f0' }}>{rh.toFixed(1)}°</span>
                        </div>
                      </div>
                      {isMapping && (
                        <div style={{ 
                          color: '#ef4444', fontWeight: 'bold', textAlign: 'center', 
                          marginTop: '4px', animation: 'pulse 1.5s ease-in-out infinite',
                        }}>
                          🔴 ĐANG QUÉT...
                        </div>
                      )}
                    </div>
                  );
                })()}
              </div>
            )}
          </>
        )}
      </div>

      {showMapManager && (
        <MapManager onClose={() => setShowMapManager(false)} />
      )}

      <Canvas shadows={{ type: THREE.PCFShadowMap }} camera={{ position: [10, 10, 10], fov: 50 }}>
        <color attach="background" args={[C_FLOOR]} />
        <ambientLight intensity={0.4} />
        <directionalLight position={[10, 15, 10]} intensity={1} castShadow />
        <pointLight position={[0, 5, 0]} intensity={0.5} />
        
        <WarehouseScene robots={robots} activePath={activePath} mapType={mapType} occupancyGrid={occupancyGrid} selectedRobotId={activeRobotId} mappingActive={mappingActive} layers={layers} />
        
        <CameraDirector viewMode={viewMode} />
      </Canvas>
      
      {/* 2D Overlay Legends */}
      <div className="warehouse-map__legend">
        <div className="warehouse-map__legend-item">
          <div className="warehouse-map__legend-dot" style={{ background: C_ROBOT }} />
          <span>Robot</span>
        </div>
        <div className="warehouse-map__legend-item">
          <div className="warehouse-map__legend-dot" style={{ background: C_IMPORT }} />
          <span>{vi.warehouse.importGate}</span>
        </div>
        <div className="warehouse-map__legend-item">
          <div className="warehouse-map__legend-dot" style={{ background: C_EXPORT }} />
          <span>{vi.warehouse.exportGate}</span>
        </div>
        <div className="warehouse-map__legend-item">
          <div className="warehouse-map__legend-dot" style={{ background: C_CHARGER }} />
          <span>{vi.warehouse.chargingStation}</span>
        </div>
        <div className="warehouse-map__legend-item">
          <div className="warehouse-map__legend-dot" style={{ background: C_SHELF }} />
          <span>{vi.warehouse.shelf}</span>
        </div>
      </div>
    </div>
  );
}
