/**
 * GazeboTDTU — Simulation Control Panel
 * 
 * Panel UI điều khiển mô phỏng:
 *   - Spawn SimBot
 *   - Play / Pause / Reset
 *   - Speed control (0.5x - 5x)
 *   - Sim info (RTF, time, FPS)
 *   - Add/Remove obstacles
 */

import React, { useState, useCallback } from 'react';
import useRobotStore from '../../stores/robotStore.js';
import useSimStore from '../../stores/simStore.js';

// ============================================================
//   STYLES (inline để giữ tập trung)
// ============================================================

const panelStyle = {
  background: 'rgba(15, 25, 35, 0.95)',
  backdropFilter: 'blur(12px)',
  borderRadius: '12px',
  border: '1px solid rgba(139, 92, 246, 0.3)',
  padding: '16px',
  display: 'flex',
  flexDirection: 'column',
  gap: '12px',
  fontFamily: "'Inter', sans-serif",
  color: '#e2e8f0',
};

const sectionHeaderStyle = {
  fontSize: '10px',
  fontWeight: 700,
  textTransform: 'uppercase',
  letterSpacing: '1.2px',
  color: '#8b5cf6',
  borderBottom: '1px solid rgba(139, 92, 246, 0.2)',
  paddingBottom: '4px',
  marginBottom: '4px',
};

const btnStyle = (active = false, color = '#3b82f6') => ({
  background: active ? `${color}33` : 'rgba(255,255,255,0.04)',
  border: active ? `1px solid ${color}66` : '1px solid rgba(255,255,255,0.08)',
  borderRadius: '6px',
  padding: '6px 12px',
  fontSize: '11px',
  fontWeight: 600,
  color: active ? color : '#94a3b8',
  cursor: 'pointer',
  transition: 'all 0.15s ease',
  display: 'flex',
  alignItems: 'center',
  gap: '4px',
});

const statRowStyle = {
  display: 'flex',
  justifyContent: 'space-between',
  fontSize: '11px',
  padding: '2px 0',
};

const speedBtnStyle = (active) => ({
  ...btnStyle(active, '#f59e0b'),
  flex: 1,
  justifyContent: 'center',
  padding: '4px',
  fontSize: '10px',
});

// ============================================================
//   COMPONENT
// ============================================================

export default function SimControlPanel() {
  const addSimRobot = useRobotStore((s) => s.addSimRobot);
  const removeSimRobot = useRobotStore((s) => s.removeSimRobot);
  const simEngines = useSimStore((s) => s.simEngines);
  const simInfo = useSimStore((s) => s.simInfo);
  const simMode = useSimStore((s) => s.simMode);
  const robots = useRobotStore((s) => s.robots);
  const toggleSimPause = useSimStore((s) => s.toggleSimPause);
  const setSimSpeed = useSimStore((s) => s.setSimSpeed);
  const resetSimRobot = useSimStore((s) => s.resetSimRobot);
  const addSimObstacle = useSimStore((s) => s.addSimObstacle);
  const removeSimObstacle = useSimStore((s) => s.removeSimObstacle);

  const [spawnName, setSpawnName] = useState('SimBot');
  const [showObstacleEditor, setShowObstacleEditor] = useState(false);
  const [obsForm, setObsForm] = useState({ id: '', cx: 5, cy: 5, w: 0.5, h: 0.5 });

  const simRobots = Object.values(robots).filter(r => r._sim);
  const firstInfo = Object.values(simInfo)[0];
  const isRunning = firstInfo?.running ?? false;

  const handleSpawn = useCallback(() => {
    addSimRobot(spawnName || 'SimBot');
  }, [addSimRobot, spawnName]);

  const handleAddObstacle = useCallback(() => {
    const id = obsForm.id || `obs_${Date.now()}`;
    addSimObstacle(id, obsForm.cx, obsForm.cy, obsForm.w, obsForm.h);
    setObsForm({ id: '', cx: 5, cy: 5, w: 0.5, h: 0.5 });
  }, [addSimObstacle, obsForm]);

  return (
    <div style={panelStyle}>
      {/* ── HEADER ── */}
      <div style={{ display: 'flex', alignItems: 'center', gap: '8px' }}>
        <span style={{ fontSize: '20px' }}>🏭</span>
        <div>
          <div style={{ fontSize: '14px', fontWeight: 700, color: '#a78bfa' }}>GazeboTDTU</div>
          <div style={{ fontSize: '10px', color: '#64748b' }}>3D Physics Simulator</div>
        </div>
        <div style={{
          marginLeft: 'auto',
          background: isRunning ? 'rgba(34,197,94,0.2)' : 'rgba(239,68,68,0.2)',
          color: isRunning ? '#4ade80' : '#f87171',
          borderRadius: '12px',
          padding: '2px 8px',
          fontSize: '9px',
          fontWeight: 700,
        }}>
          {isRunning ? '● RUNNING' : '⏸ PAUSED'}
        </div>
      </div>

      {/* ── SPAWN ROBOT ── */}
      <div>
        <div style={sectionHeaderStyle}>🤖 Spawn Robot</div>
        <div style={{ display: 'flex', gap: '6px', alignItems: 'center' }}>
          <input
            type="text"
            value={spawnName}
            onChange={(e) => setSpawnName(e.target.value)}
            placeholder="Tên robot..."
            style={{
              flex: 1,
              background: 'rgba(255,255,255,0.06)',
              border: '1px solid rgba(255,255,255,0.1)',
              borderRadius: '6px',
              padding: '6px 10px',
              fontSize: '11px',
              color: '#e2e8f0',
              outline: 'none',
            }}
          />
          <button style={btnStyle(false, '#10b981')} onClick={handleSpawn}>
            ➕ Spawn
          </button>
        </div>
      </div>

      {/* ── ACTIVE SIM ROBOTS ── */}
      {simRobots.length > 0 && (
        <div>
          <div style={sectionHeaderStyle}>📋 Sim Robots ({simRobots.length})</div>
          {simRobots.map(robot => {
            const info = simInfo[robot.id];
            const telem = robot.telemetry;
            return (
              <div key={robot.id} style={{
                background: 'rgba(255,255,255,0.03)',
                borderRadius: '8px',
                padding: '8px',
                marginBottom: '6px',
                border: '1px solid rgba(255,255,255,0.06)',
              }}>
                <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '6px' }}>
                  <span style={{ fontSize: '12px', fontWeight: 600, color: '#a78bfa' }}>
                    🤖 {robot.name}
                  </span>
                  <div style={{ display: 'flex', gap: '4px' }}>
                    <button
                      style={{ ...btnStyle(false, '#f59e0b'), padding: '2px 6px', fontSize: '10px' }}
                      onClick={() => resetSimRobot(robot.id)}
                      title="Reset vị trí"
                    >
                      🔄
                    </button>
                    <button
                      style={{ ...btnStyle(false, '#ef4444'), padding: '2px 6px', fontSize: '10px' }}
                      onClick={() => removeSimRobot(robot.id)}
                      title="Xóa robot"
                    >
                      🗑️
                    </button>
                  </div>
                </div>
                
                {/* Stats */}
                <div style={{ fontFamily: 'monospace', fontSize: '10px', color: '#94a3b8' }}>
                  <div style={statRowStyle}>
                    <span>Pos</span>
                    <span style={{ color: '#e2e8f0' }}>
                      ({telem?.x?.toFixed(2)}, {telem?.y?.toFixed(2)})
                    </span>
                  </div>
                  <div style={statRowStyle}>
                    <span>Heading</span>
                    <span style={{ color: '#e2e8f0' }}>
                      {telem?.heading?.toFixed(1)}°
                    </span>
                  </div>
                  <div style={statRowStyle}>
                    <span>Velocity</span>
                    <span style={{ color: '#10b981' }}>
                      v={telem?.linearVel?.toFixed(3)} w={telem?.angularVel?.toFixed(3)}
                    </span>
                  </div>
                  <div style={statRowStyle}>
                    <span>Lidar pts</span>
                    <span style={{ color: '#ef4444' }}>
                      {telem?.lidar?.length ?? 0}
                    </span>
                  </div>
                </div>
              </div>
            );
          })}
        </div>
      )}

      {/* ── SIMULATION CONTROLS ── */}
      {simRobots.length > 0 && (
        <div>
          <div style={sectionHeaderStyle}>⏱️ Simulation Controls</div>
          
          {/* Play/Pause */}
          <div style={{ display: 'flex', gap: '6px', marginBottom: '8px' }}>
            <button style={{ ...btnStyle(isRunning, '#10b981'), flex: 1, justifyContent: 'center' }} onClick={toggleSimPause}>
              {isRunning ? '⏸ Pause' : '▶️ Play'}
            </button>
          </div>

          {/* Speed Control */}
          <div style={{ display: 'flex', gap: '4px', marginBottom: '8px' }}>
            {[0.5, 1, 2, 5].map(speed => (
              <button
                key={speed}
                style={speedBtnStyle(firstInfo?.speedFactor === speed)}
                onClick={() => setSimSpeed(speed)}
              >
                {speed}x
              </button>
            ))}
          </div>

          {/* Sim Info */}
          {firstInfo && (
            <div style={{
              background: 'rgba(0,0,0,0.3)',
              borderRadius: '6px',
              padding: '6px 8px',
              fontFamily: 'monospace',
              fontSize: '10px',
            }}>
              <div style={statRowStyle}>
                <span>Sim Time</span>
                <strong style={{ color: '#a78bfa' }}>{firstInfo.simTime?.toFixed(1)}s</strong>
              </div>
              <div style={statRowStyle}>
                <span>Real Time Factor</span>
                <strong style={{ color: '#f59e0b' }}>{firstInfo.rtf?.toFixed(2)}</strong>
              </div>
              <div style={statRowStyle}>
                <span>Steps</span>
                <span>{firstInfo.stepCount?.toLocaleString()}</span>
              </div>
              <div style={statRowStyle}>
                <span>World Segments</span>
                <span>{firstInfo.segments}</span>
              </div>
            </div>
          )}
        </div>
      )}

      {/* ── OBSTACLE EDITOR ── */}
      {simRobots.length > 0 && (
        <div>
          <div style={sectionHeaderStyle}>
            <span onClick={() => setShowObstacleEditor(!showObstacleEditor)} style={{ cursor: 'pointer' }}>
              🧱 Obstacles {showObstacleEditor ? '▲' : '▼'}
            </span>
          </div>
          
          {showObstacleEditor && (
            <div style={{ display: 'flex', flexDirection: 'column', gap: '4px' }}>
              <div style={{ display: 'grid', gridTemplateColumns: '1fr 1fr', gap: '4px' }}>
                <input
                  type="number" step="0.1" value={obsForm.cx}
                  onChange={(e) => setObsForm(f => ({ ...f, cx: +e.target.value }))}
                  placeholder="Center X"
                  style={{
                    background: 'rgba(255,255,255,0.06)', border: '1px solid rgba(255,255,255,0.1)',
                    borderRadius: '4px', padding: '4px 6px', fontSize: '10px', color: '#e2e8f0',
                  }}
                />
                <input
                  type="number" step="0.1" value={obsForm.cy}
                  onChange={(e) => setObsForm(f => ({ ...f, cy: +e.target.value }))}
                  placeholder="Center Y"
                  style={{
                    background: 'rgba(255,255,255,0.06)', border: '1px solid rgba(255,255,255,0.1)',
                    borderRadius: '4px', padding: '4px 6px', fontSize: '10px', color: '#e2e8f0',
                  }}
                />
                <input
                  type="number" step="0.1" value={obsForm.w}
                  onChange={(e) => setObsForm(f => ({ ...f, w: +e.target.value }))}
                  placeholder="Width"
                  style={{
                    background: 'rgba(255,255,255,0.06)', border: '1px solid rgba(255,255,255,0.1)',
                    borderRadius: '4px', padding: '4px 6px', fontSize: '10px', color: '#e2e8f0',
                  }}
                />
                <input
                  type="number" step="0.1" value={obsForm.h}
                  onChange={(e) => setObsForm(f => ({ ...f, h: +e.target.value }))}
                  placeholder="Height"
                  style={{
                    background: 'rgba(255,255,255,0.06)', border: '1px solid rgba(255,255,255,0.1)',
                    borderRadius: '4px', padding: '4px 6px', fontSize: '10px', color: '#e2e8f0',
                  }}
                />
              </div>
              <button style={btnStyle(false, '#f59e0b')} onClick={handleAddObstacle}>
                ➕ Thêm Vật Cản
              </button>
            </div>
          )}
        </div>
      )}

      {/* ── EMPTY STATE ── */}
      {simRobots.length === 0 && (
        <div style={{
          textAlign: 'center',
          padding: '20px 10px',
          color: '#64748b',
          fontSize: '11px',
        }}>
          <div style={{ fontSize: '32px', marginBottom: '8px' }}>🤖</div>
          <div>Chưa có robot mô phỏng.</div>
          <div style={{ marginTop: '4px' }}>Nhấn <strong>Spawn</strong> để tạo robot ảo.</div>
          <div style={{ marginTop: '8px', fontSize: '10px', color: '#475569' }}>
            Robot sẽ chạy hoàn toàn trong browser — không cần ESP32 thật.
          </div>
        </div>
      )}
    </div>
  );
}
