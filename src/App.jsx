/**
 * AMR 2.0 — Main Application
 * Trung Tâm Điều Khiển Robot Vận Chuyển Hàng
 */

import { useState, useEffect } from 'react';
import './App.css';
import WarehouseMap from './components/WarehouseMap/WarehouseMap.jsx';
import ConnectionPanel from './components/ConnectionPanel/ConnectionPanel.jsx';
import TaskManager from './components/TaskManager/TaskManager.jsx';
import TelemetryChart from './components/TelemetryChart/TelemetryChart.jsx';
import CollisionManager from './components/CollisionManager/CollisionManager.jsx';
import useRobotStore from './stores/robotStore.js';
import vi from './i18n/vi.js';

function App() {
  const [activePath, setActivePath] = useState(null);
  const [leftSidebarOpen, setLeftSidebarOpen] = useState(true);
  const [rightSidebarOpen, setRightSidebarOpen] = useState(true);
  const toggleLeftSidebar = () => setLeftSidebarOpen(prev => !prev);
  const toggleRightSidebar = () => setRightSidebarOpen(prev => !prev);
  const robots = useRobotStore((s) => s.robots);
  const loadStoredRobots = useRobotStore((s) => s.loadStoredRobots);

  useEffect(() => {
    loadStoredRobots();
  }, [loadStoredRobots]);
  const robotList = Object.values(robots);

  const onlineCount = robotList.filter(r => r.status === 'connected').length;
  const offlineCount = robotList.filter(r => r.status !== 'connected').length;
  const lowBatteryCount = robotList.filter(r => r.status === 'connected' && r.telemetry.battery < 20).length;

  const { sendManualControl } = useRobotStore();

  const handleGlobalEStop = () => {
    // Send 0,0 velocity immediately to all connected robots
    robotList.filter(r => r.status === 'connected').forEach(robot => {
      sendManualControl(robot.id, 0, 0);
    });
    // Triggers an emergency toast or UI state ideally, but currently just logs and stops
    console.error("GLOBAL E-STOP TRIGGERED!");
  };


  return (
    <div className="app-layout">
      <CollisionManager />
      
      {/* ── HEADER ──────────────────────────────────────────── */}
      <header className="app-header">
        <div className="app-header__logo">
          <div className="app-header__logo-icon">🤖</div>
          <div>
            <div className="app-header__title">{vi.appTitle}</div>
            <div className="app-header__subtitle">v2.0 — Tauri + React + ESP32-S3</div>
          </div>
        </div>

        <div className="app-header__center">
          <button className="btn-estop" onClick={handleGlobalEStop}>
            <span className="btn-estop__text">E-STOP</span>
          </button>
        </div>

        <div className="app-header__status">
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

      {/* ── SIDEBAR LEFT — Robot Panel ──────────────────────── */}
      <aside className="sidebar-left">
        <ConnectionPanel />
        <TelemetryChart />
      </aside>

      {/* ── MAIN — Warehouse Map ────────────────────────────── */}
      <main className="main-content">
        <WarehouseMap activePath={activePath} />
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
          🔌 WebSocket Protocol
        </div>
        <div className="app-footer__item" style={{ marginLeft: 'auto' }}>
          AMR 2.0 © 2026
        </div>
      </footer>
    </div>
  );
}

export default App;
