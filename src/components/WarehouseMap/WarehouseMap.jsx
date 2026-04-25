/**
 * AMR 2.0 — Warehouse Map Component (3D Version)
 * Bản đồ kho xưởng 3D sử dụng React Three Fiber
 */

import React, { useMemo, useState, useRef } from 'react';
import { Canvas, useFrame } from '@react-three/fiber';
import { OrbitControls, Grid, Text, Box, Cylinder, Sphere, Edges, Line, Cone } from '@react-three/drei';
import * as THREE from 'three';
import {
  WAREHOUSE_WIDTH, WAREHOUSE_HEIGHT,
  SHELVES, GATES, CHARGING_STATIONS
} from '../../core/warehouse.js';
import useRobotStore from '../../stores/robotStore.js';
import vi from '../../i18n/vi.js';

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

function RobotModel({ position, rotation, battery, label, lidar, hasObs }) {
  const isHealthy = battery > 20;
  const color = isHealthy ? C_ROBOT : '#ef4444';

  return (
    <group position={position} rotation={rotation}>
      {/* Body */}
      <mesh castShadow position={[0, 0.15, 0]}>
        <boxGeometry args={[0.6, 0.3, 0.8]} />
        <meshStandardMaterial color={color} metalness={0.6} roughness={0.2} />
        <Edges scale={1.05} threshold={15} color="#000" />
      </mesh>
      {/* Lidar/Sensor on top */}
      <mesh position={[0, 0.35, 0.1]}>
        <cylinderGeometry args={[0.1, 0.1, 0.1, 16]} />
        <meshStandardMaterial color="#ef4444" emissive={hasObs ? "#ef4444" : "#000"} emissiveIntensity={hasObs ? 1 : 0} />
      </mesh>
      {/* Wheels */}
      <mesh position={[-0.35, 0.1, 0]} rotation={[0, 0, Math.PI / 2]}>
        <cylinderGeometry args={[0.1, 0.1, 0.4, 16]} />
        <meshStandardMaterial color="#1e293b" />
      </mesh>
      <mesh position={[0.35, 0.1, 0]} rotation={[0, 0, Math.PI / 2]}>
        <cylinderGeometry args={[0.1, 0.1, 0.4, 16]} />
        <meshStandardMaterial color="#1e293b" />
      </mesh>
      {/* Front Arrow */}
      {/* Robot depth is 0.8, so front is at z = 0.4 */}
      <mesh position={[0, 0.15, 0.4]} rotation={[Math.PI / 2, 0, 0]}>
        <coneGeometry args={[0.15, 0.2, 16]} />
        <meshStandardMaterial color="#22c55e" />
      </mesh>

      {/* Lidar Point Cloud (Red dots) */}
      {lidar && lidar.length > 0 && (
        <group position={[0, 0.4, 0]}>
          {lidar.map((pt, i) => {
            // Front is +Z. Lidar angle 0 = front => +Z.
            // Lidar spins clockwise.
            const rad = -(pt.a * Math.PI) / 180.0;
            const dist_m = pt.d / 1000.0;
            const lx = Math.sin(rad) * dist_m;
            const lz = Math.cos(rad) * dist_m;
            return (
              <mesh key={i} position={[lx, 0, lz]}>
                <boxGeometry args={[0.04, 0.04, 0.04]} />
                <meshBasicMaterial color="#ef4444" />
              </mesh>
            );
          })}
        </group>
      )}

      {/* Label */}
      <Text
        position={[0, 0.8, 0]}
        fontSize={0.2}
        color="white"
        anchorX="center"
        anchorY="middle"
        rotation={[...rotation].map(r => -r)} // Look somewhat to camera or just fix it? Better use Billboard for text usually, but simple text is ok.
      >
        {label} ({battery}%) {hasObs ? "⚠️" : ""}
      </Text>
    </group>
  );
}

function RealisticShelf({ shelf, cx, cz }) {
  const map2To3X = (x) => x - cx;
  const map2To3Z = (y) => -(y - cz);

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
              
              return (
                <group key={slot.id} position={[slotCenterLocalX, yBase + 0.4, 0]}>
                   {/* Bin Box */}
                   <Box args={[slotW - 0.05, 0.8, d - 0.1]} castShadow receiveShadow>
                     <meshStandardMaterial color="#1e293b" transparent opacity={0.5} polygonOffset polygonOffsetFactor={1} />
                     <Edges color="#475569" />
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

function WarehouseScene({ robots, activePath }) {
  // Center the warehouse in the world
  const cx = WAREHOUSE_WIDTH / 2;
  const cz = WAREHOUSE_HEIGHT / 2; // y in 2D is z in 3D

  // Mapping from 2D coordinate system to 3D.
  // 2D (x,y) -> 3D (X - cx, 0, Z - cz)
  // We flip Z logically if needed, but standard top-down 2D mapping keeps x=x, z=-y typically.
  // Assuming Bottom-Left is (0,0) in 2D. In 3D Top-Down, +X is right, -Z is up.
  // Let's do: X = x - cx,  Z = -(y - cz)
  const map2To3X = (x) => x - cx;
  const map2To3Z = (y) => -(y - cz);

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

      {/* Shelves */}
      {SHELVES.map((shelf, idx) => (
        <RealisticShelf key={idx} shelf={shelf} cx={cx} cz={cz} />
      ))}

      {/* Gates */}
      {Object.values(GATES).map((gate, idx) => {
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

      {/* Charging Stations */}
      {CHARGING_STATIONS.map((charger, idx) => {
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

      {/* Active Path Line */}
      {activePath && activePath.length > 1 && (
        <group>
          {activePath.map((pt, i) => {
            if (i === 0) return null;
            const prev = activePath[i - 1];
            // Simple path geometry using multiple boxes or tubes (for simplicity just dots here)
            return (
              <Sphere key={i} args={[0.05]} position={[map2To3X(pt.x), 0.1, map2To3Z(pt.y)]}>
                <meshStandardMaterial color="#3b82f6" emissive="#3b82f6" emissiveIntensity={1} />
              </Sphere>
            );
          })}
        </group>
      )}

      {/* Robots */}
      {Object.values(robots).map(robot => {
        if (robot.status !== 'connected') return null;
        const x = map2To3X(robot.telemetry.x);
        const z = map2To3Z(robot.telemetry.y);
        // Heading rad needs to be mapped to 3D Y-axis rotation
        // 2D: 0 rad is +x (right), pi/2 is +y (up)
        // 3D: 0 rot is +z (down screen usually), so need offset
        const rotY = robot.telemetry.headingRad; // Adjust based on your firmware orientation
        
        return (
          <RobotModel
            key={robot.id}
            position={[x, 0, z]}
            rotation={[0, rotY, 0]}
            battery={robot.telemetry.battery}
            label={robot.name}
            lidar={robot.telemetry.lidar || []}
            hasObs={robot.telemetry.obs || false}
          />
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
  const [viewMode, setViewMode] = useState('iso'); // 'free', 'top', 'iso'

  return (
    <div className="warehouse-map" style={{ width: '100%', height: '100%', background: C_FLOOR, position: 'relative' }}>
      
      {/* ── CAMERA CONTROLS UI ── */}
      <div style={{ position: 'absolute', top: 16, left: 16, zIndex: 10, display: 'flex', gap: '8px', background: 'rgba(15, 25, 35, 0.8)', padding: '6px', borderRadius: '8px', backdropFilter: 'blur(10px)' }}>
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

      <Canvas shadows camera={{ position: [10, 10, 10], fov: 50 }}>
        <color attach="background" args={[C_FLOOR]} />
        <ambientLight intensity={0.4} />
        <directionalLight position={[10, 15, 10]} intensity={1} castShadow />
        <pointLight position={[0, 5, 0]} intensity={0.5} />
        
        <WarehouseScene robots={robots} activePath={activePath} />
        
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
