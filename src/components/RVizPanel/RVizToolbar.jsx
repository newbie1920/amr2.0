/**
 * RVizTDTU — Toolbar Component
 * 
 * Tool buttons (giống RViz):
 *   - Move (pan) — default
 *   - 2D Pose Estimate
 *   - 2D Nav Goal
 *   - Measure
 * 
 * Layer toggles (bật/tắt visualization layers)
 */

import React from 'react';

// ============================================================
//   STYLES
// ============================================================

const toolbarStyle = {
  display: 'flex',
  flexDirection: 'column',
  gap: '2px',
  padding: '6px 4px',
  background: 'rgba(15, 25, 35, 0.95)',
  borderRight: '1px solid rgba(139, 92, 246, 0.2)',
  minWidth: '36px',
};

const toolBtnStyle = (active) => ({
  width: '28px',
  height: '28px',
  display: 'flex',
  alignItems: 'center',
  justifyContent: 'center',
  background: active ? 'rgba(139, 92, 246, 0.25)' : 'transparent',
  border: active ? '1px solid rgba(139, 92, 246, 0.5)' : '1px solid transparent',
  borderRadius: '4px',
  cursor: 'pointer',
  fontSize: '14px',
  color: active ? '#a78bfa' : '#64748b',
  transition: 'all 0.12s ease',
});

const dividerStyle = {
  height: '1px',
  background: 'rgba(255,255,255,0.06)',
  margin: '4px 0',
};

const layerToggleStyle = (enabled) => ({
  width: '28px',
  height: '20px',
  display: 'flex',
  alignItems: 'center',
  justifyContent: 'center',
  background: enabled ? 'rgba(34, 197, 94, 0.15)' : 'transparent',
  border: 'none',
  borderRadius: '3px',
  cursor: 'pointer',
  fontSize: '10px',
  color: enabled ? '#4ade80' : '#475569',
  transition: 'all 0.12s ease',
});

const tooltipWrapStyle = {
  position: 'relative',
};

// ============================================================
//   TOOL DEFINITIONS
// ============================================================

const TOOLS = [
  { id: 'move', icon: '✋', label: 'Move / Pan' },
  { id: 'pose', icon: '📍', label: '2D Pose Estimate' },
  { id: 'goal', icon: '🎯', label: '2D Nav Goal' },
  { id: 'measure', icon: '📏', label: 'Measure Distance' },
];

const LAYERS = [
  { id: 'grid', icon: '#', label: 'Grid' },
  { id: 'map', icon: '🗺', label: 'Occupancy Map' },
  { id: 'costmap', icon: 'CM', label: 'Costmap' },
  { id: 'walls', icon: '🧱', label: 'World Segments' },
  { id: 'laser', icon: '🔴', label: 'Laser Scan' },
  { id: 'path', icon: '➡', label: 'Nav Path' },
  { id: 'robot', icon: '🤖', label: 'Robot Pose' },
  { id: 'tf', icon: '🔗', label: 'TF Frames' },
  { id: 'frontier', icon: '🔵', label: 'Frontiers' },
];

// ============================================================
//   COMPONENT
// ============================================================

export default function RVizToolbar({ activeTool, onToolChange, layers, onToggleLayer, isMapping, onToggleMapping, dataSource = 'none' }) {
  // Mode-aware tooltips
  const toolTips = {
    move: 'Di chuyển / Kéo bản đồ',
    pose: dataSource === 'sim' ? '📍 Đặt vị trí robot (SimEngine reset)' 
        : dataSource === 'real' ? '📍 Đặt vị trí robot (gửi lệnh ESP32)'
        : '📍 2D Pose Estimate',
    goal: dataSource === 'sim' ? '🎯 Điểm đến (A* + DWA trên browser)'
        : dataSource === 'real' ? '🎯 Điểm đến (tuỳ NavMode: Onboard/PC)'
        : '🎯 2D Nav Goal',
    measure: '📏 Đo khoảng cách',
  };

  const mappingTip = dataSource === 'sim' 
    ? (isMapping ? 'Dừng Auto-Explore (SimLidar → Browser SLAM)' : 'Bắt đầu Auto-Explore (SimLidar)')
    : dataSource === 'real'
    ? (isMapping ? 'Dừng quét (ESP32 LiDAR → PC SLAM)' : 'Bắt đầu quét (ESP32 LiDAR)')
    : (isMapping ? 'Stop Mapping' : 'Start Mapping');
  return (
    <div style={toolbarStyle}>
      {/* Tools */}
      {TOOLS.map(tool => (
        <div key={tool.id} style={tooltipWrapStyle} title={toolTips[tool.id] || tool.label}>
          <button
            style={toolBtnStyle(activeTool === tool.id)}
            onClick={() => onToolChange(tool.id)}
          >
            {tool.icon}
          </button>
        </div>
      ))}

      <div style={dividerStyle} />

      {/* Actions */}
      <div style={tooltipWrapStyle} title={mappingTip}>
        <button
          style={{
            ...toolBtnStyle(isMapping),
            color: isMapping ? '#ef4444' : '#10b981',
            borderColor: isMapping ? 'rgba(239, 68, 68, 0.5)' : 'rgba(16, 185, 129, 0.3)',
            background: isMapping ? 'rgba(239, 68, 68, 0.15)' : 'rgba(16, 185, 129, 0.1)',
          }}
          onClick={onToggleMapping}
        >
          🚀
        </button>
      </div>

      <div style={dividerStyle} />

      {/* Layer Toggles */}
      {LAYERS.map(layer => (
        <div key={layer.id} style={tooltipWrapStyle} title={layer.label}>
          <button
            style={layerToggleStyle(layers[layer.id] !== false)}
            onClick={() => onToggleLayer(layer.id)}
          >
            {layer.icon}
          </button>
        </div>
      ))}
    </div>
  );
}

export { TOOLS, LAYERS };
