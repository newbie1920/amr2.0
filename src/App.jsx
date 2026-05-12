/**
 * AMR 2.0 — Main Application (NavTDTU Edition)
 * Trung Tâm Điều Khiển Robot Vận Chuyển Hàng
 * 
 * Layout: Split-pane giống RViz + Gazebo
 *   Left Sidebar:  Connection Panel + SimControlPanel
 *   Center:        GazeboTDTU (3D) | RVizTDTU (2D) — split pane
 *   Right Sidebar: Task Manager
 */

import { useState, useEffect, useRef, useCallback } from 'react';
import './App.css';
import WarehouseMap from './components/WarehouseMap/WarehouseMap.jsx';
import ConnectionPanel from './components/ConnectionPanel/ConnectionPanel.jsx';
import TaskManager from './components/TaskManager/TaskManager.jsx';
import TelemetryChart from './components/TelemetryChart/TelemetryChart.jsx';
import CollisionManager from './components/CollisionManager/CollisionManager.jsx';
import SimControlPanel from './components/SimControlPanel/SimControlPanel.jsx';
import RVizPanel from './components/RVizPanel/RVizPanel.jsx';
import DWATuningPanel from './components/DWATuningPanel/DWATuningPanel.jsx';
import useRobotStore from './stores/robotStore.js';
import vi from './i18n/vi.js';
import { startTrafficBroadcaster, stopTrafficBroadcaster } from './core/trafficManager.js';

// ── Split Pane Resizer ─────────────────────────────────────

function SplitPane({ left, right, defaultRatio = 0.55, minLeft = 300, minRight = 250 }) {
  const containerRef = useRef(null);
  const [ratio, setRatio] = useState(() => {
    const saved = localStorage.getItem('rviz_split_ratio');
    return saved ? parseFloat(saved) : defaultRatio;
  });
  const isDragging = useRef(false);

  const handleMouseDown = useCallback((e) => {
    e.preventDefault();
    isDragging.current = true;
    document.body.style.cursor = 'col-resize';
    document.body.style.userSelect = 'none';
  }, []);

  useEffect(() => {
    const handleMouseMove = (e) => {
      if (!isDragging.current || !containerRef.current) return;
      const rect = containerRef.current.getBoundingClientRect();
      const x = e.clientX - rect.left;
      const totalW = rect.width;
      let newRatio = x / totalW;
      // Enforce min sizes
      if (newRatio * totalW < minLeft) newRatio = minLeft / totalW;
      if ((1 - newRatio) * totalW < minRight) newRatio = 1 - minRight / totalW;
      newRatio = Math.max(0.2, Math.min(0.85, newRatio));
      setRatio(newRatio);
    };

    const handleMouseUp = () => {
      if (isDragging.current) {
        isDragging.current = false;
        document.body.style.cursor = '';
        document.body.style.userSelect = '';
        localStorage.setItem('rviz_split_ratio', ratio.toString());
      }
    };

    window.addEventListener('mousemove', handleMouseMove);
    window.addEventListener('mouseup', handleMouseUp);
    return () => {
      window.removeEventListener('mousemove', handleMouseMove);
      window.removeEventListener('mouseup', handleMouseUp);
    };
  }, [ratio, minLeft, minRight]);

  return (
    <div ref={containerRef} className="split-pane">
      <div className="split-pane__left" style={{ width: `${ratio * 100}%` }}>
        {left}
      </div>
      <div
        className="split-pane__resizer"
        onMouseDown={handleMouseDown}
      >
        <div className="split-pane__resizer-line" />
      </div>
      <div className="split-pane__right" style={{ width: `${(1 - ratio) * 100}%` }}>
        {right}
      </div>
    </div>
  );
}

// ── Main App ───────────────────────────────────────────────

function App() {
  const [activePath, setActivePath] = useState(null);
  const [viewMode, setViewMode] = useState('split'); // 'gazebo', 'rviz', 'split'
  const robots = useRobotStore((s) => s.robots);
  const loadStoredRobots = useRobotStore((s) => s.loadStoredRobots);
  const simMode = useRobotStore((s) => s.simMode);
  const simInfo = useRobotStore((s) => s.simInfo);
  const selectedRobotId = useRobotStore((s) => s.selectedRobotId);

  // Determine type of selected robot for context-sensitive UI
  const selectedRobot = selectedRobotId ? robots[selectedRobotId] : null;
  const selectedType = selectedRobot?._sim ? 'sim' 
    : (selectedRobot?.telemetry?.hitl || selectedRobot?.connection?.hitlEnabled) ? 'hitl'
    : selectedRobot ? 'real' : 'none';

  useEffect(() => {
    loadStoredRobots();
    
    // Start decentralized traffic broadcasting
    startTrafficBroadcaster(() => useRobotStore.getState());
    
    return () => {
      stopTrafficBroadcaster();
    };
  }, [loadStoredRobots]);
  const robotList = Object.values(robots);

  const onlineCount = robotList.filter(r => r.status === 'connected').length;
  const offlineCount = robotList.filter(r => r.status !== 'connected').length;
  const lowBatteryCount = robotList.filter(r => r.status === 'connected' && r.telemetry.battery < 20).length;
  const simCount = robotList.filter(r => r._sim).length;

  const { sendManualControl } = useRobotStore();

  const handleGlobalEStop = () => {
    robotList.filter(r => r.status === 'connected').forEach(robot => {
      sendManualControl(robot.id, 0, 0);
    });
    console.error("GLOBAL E-STOP TRIGGERED!");
  };

  // Sim info for footer
  const firstSimInfo = Object.values(simInfo)[0];

  return (
    <div className="app-layout">
      <CollisionManager />
      
      {/* ── HEADER ──────────────────────────────────────────── */}
      <header className="app-header">
        <div className="app-header__logo">
          <div className="app-header__logo-icon">🤖</div>
          <div>
            <div className="app-header__title">{vi.appTitle}</div>
            <div className="app-header__subtitle">
              NavTDTU — {selectedRobot 
                ? `${selectedRobot.name} (${selectedType === 'sim' ? '🏭 SIM' : selectedType === 'hitl' ? '🌐 HITL' : '🔌 REAL'})` 
                : 'Chưa chọn robot'}
            </div>
          </div>
        </div>

        <div className="app-header__center" style={{ display: 'flex', gap: '24px', alignItems: 'center' }}>
          <div style={{ display: 'flex', background: 'var(--bg-input)', padding: '4px', borderRadius: '8px', gap: '4px', border: '1px solid var(--border-subtle)' }}>
            <button 
              onClick={() => setViewMode('gazebo')}
              style={{ padding: '6px 12px', background: viewMode === 'gazebo' ? 'var(--bg-elevated)' : 'transparent', color: viewMode === 'gazebo' ? '#fff' : 'var(--text-muted)', border: 'none', borderRadius: '4px', cursor: 'pointer', fontWeight: 600, fontSize: '12px', transition: 'all 0.2s' }}
            >
              🌍 3D
            </button>
            <button 
              onClick={() => setViewMode('split')}
              style={{ padding: '6px 12px', background: viewMode === 'split' ? 'var(--bg-elevated)' : 'transparent', color: viewMode === 'split' ? '#fff' : 'var(--text-muted)', border: 'none', borderRadius: '4px', cursor: 'pointer', fontWeight: 600, fontSize: '12px', transition: 'all 0.2s' }}
            >
              🌗 Split
            </button>
            <button 
              onClick={() => setViewMode('rviz')}
              style={{ padding: '6px 12px', background: viewMode === 'rviz' ? 'var(--bg-elevated)' : 'transparent', color: viewMode === 'rviz' ? '#fff' : 'var(--text-muted)', border: 'none', borderRadius: '4px', cursor: 'pointer', fontWeight: 600, fontSize: '12px', transition: 'all 0.2s' }}
            >
              📊 RViz
            </button>
          </div>

          <button className="btn-estop" onClick={handleGlobalEStop}>
            <span className="btn-estop__text">E-STOP</span>
          </button>
        </div>

        <div className="app-header__status">

          {simCount > 0 && (
            <div className="status-badge">
              <div className="status-badge__dot status-badge__dot--sim" />
              <span>🏭 {simCount} Sim</span>
            </div>
          )}
          <div className="status-badge">
            <div className={`status-badge__dot ${onlineCount > 0 ? 'status-badge__dot--online' : 'status-badge__dot--offline'}`} />
            <span>{onlineCount} {vi.statusBar.robotsOnline}</span>
          </div>

          {offlineCount > 0 && (
            <div className="status-badge">
              <div className="status-badge__dot status-badge__dot--offline" />
              <span>{offlineCount} {vi.statusBar.robotsOffline}</span>
            </div>
          )}

          {lowBatteryCount > 0 && (
            <div className="status-badge">
              <div className="status-badge__dot status-badge__dot--warning" />
              <span>⚠️ {lowBatteryCount} {vi.statusBar.warnings}</span>
            </div>
          )}
        </div>
      </header>

      {/* ── SIDEBAR LEFT — Robot Panel + Sim Controls ──────── */}
      <aside className="sidebar-left">
        <SimControlPanel />
        <ConnectionPanel />
        <DWATuningPanel />
        <TelemetryChart />
      </aside>

      {/* ── MAIN — Split Pane: GazeboTDTU | RVizTDTU ────────── */}
      <main className="main-content">
        {viewMode === 'split' && (
          <SplitPane
            left={<WarehouseMap activePath={activePath} />}
            right={<RVizPanel activePath={activePath} />}
          />
        )}
        {viewMode === 'gazebo' && (
          <WarehouseMap activePath={activePath} />
        )}
        {viewMode === 'rviz' && (
          <RVizPanel activePath={activePath} />
        )}
      </main>

      {/* ── SIDEBAR RIGHT — Task Manager ────────────────────── */}
      <aside className="sidebar-right">
        <TaskManager onPathGenerated={(path) => setActivePath(path)} />
      </aside>

      {/* ── FOOTER ──────────────────────────────────────────── */}
      <footer className="app-footer">
        <div className="app-footer__item">
          📐 {vi.warehouse.dimensions}
        </div>
        <div className="app-footer__item">
          {selectedType === 'sim' ? '🏭 GazeboTDTU Physics (Browser)' :
           selectedType === 'hitl' ? '🌐 HITL — ESP32 + Browser Physics' :
           selectedType === 'real' ? '🔌 ESP32 WebSocket + Sensors' :
           '⏳ Chờ kết nối robot...'}
        </div>
        {firstSimInfo && (
          <div className="app-footer__item app-footer__sim-info">
            <span className="sim-info__label">Real Time Factor:</span>
            <span className={`sim-info__value ${firstSimInfo.rtf >= 0.9 ? 'sim-info__value--good' : 'sim-info__value--slow'}`}>
              {firstSimInfo.rtf?.toFixed(2)}
            </span>
            <span className="sim-info__divider">|</span>
            <span className="sim-info__label">Sim Time:</span>
            <span className="sim-info__value">{firstSimInfo.simTime?.toFixed(1)}s</span>
            <span className="sim-info__divider">|</span>
            <span className="sim-info__label">Iterations:</span>
            <span className="sim-info__value">{firstSimInfo.stepCount?.toLocaleString()}</span>
            <span className="sim-info__divider">|</span>
            <span className="sim-info__label">FPS:</span>
            <span className="sim-info__value">{(firstSimInfo.rtf * 50)?.toFixed(1)}</span>
          </div>
        )}
        <div className="app-footer__item" style={{ marginLeft: 'auto' }}>
          NavTDTU v1.0 © 2026
        </div>
      </footer>
    </div>
  );
}

export default App;
